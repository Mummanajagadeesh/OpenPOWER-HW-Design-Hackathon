// fir_wb.v
// Wishbone-lite wrapper for the FIR accelerator
// - Typical Caravel-style wishbone signals (wbs_*)
// - Simple register map:
//   0x00 CONTROL: bit0 = START (pulse), bit1 = RESET_CORE
//   0x04 STATUS : bit0 = BUSY, bit1 = OUT_VALID (one-cycle), others reserved
//   0x08 TAPS   : number of taps (<= MAX_TAPS)
//   0x10..0x10+4*(MAX_TAPS-1): COEFS (word per coef)
//   0x80 IN_FIFO  (write-only) - write sample here
//   0x84 OUT_FIFO (read-only)  - read sample here (pops)
//
// Single-cycle ack, no byte enables support required here (wbs_sel_i ignored).

`timescale 1ns / 1ps
module fir_wb #(
    parameter DATA_W   = 32,
    parameter COEF_W   = 32,
    parameter ACC_W    = 64,
    parameter MAX_TAPS = 64,
    parameter FIFO_DEPTH = 32,
    parameter WB_BASE = 32'h0000_0000
)(
    input  wire           wb_clk_i,
    input  wire           wb_rst_i,

    // Wishbone slave (Caravel style)
    input  wire           wbs_cyc_i,
    input  wire           wbs_stb_i,
    input  wire           wbs_we_i,
    input  wire [3:0]     wbs_sel_i,
    input  wire [31:0]    wbs_adr_i,
    input  wire [31:0]    wbs_dat_i,
    output reg  [31:0]    wbs_dat_o,
    output reg            wbs_ack_o,

    // IRQ output
    output reg            irq_o
);

    // local addresses (relative)
    localparam ADDR_CONTROL  = 32'h00;
    localparam ADDR_STATUS   = 32'h04;
    localparam ADDR_TAPS     = 32'h08;
    localparam ADDR_COEF_BASE= 32'h10;
    localparam ADDR_IN_FIFO  = 32'h80;
    localparam ADDR_OUT_FIFO = 32'h84;

    // internal regs
    reg [31:0] control;
    reg [31:0] status;
    reg [31:0] taps_reg;

    // register to pulse FIR's start
    reg start_pulse;

    // FIFOs
    wire in_full;
    wire in_empty;
    wire out_full;
    wire out_empty;
    wire [31:0] out_fifo_read_data;
    wire [31:0] in_fifo_write_data;
    reg in_fifo_wr;
    reg out_fifo_rd;

    // instantiate fifos
    fifo_sync #(.WIDTH(DATA_W), .DEPTH(FIFO_DEPTH), .PTR_W(6)) in_fifo (
        .clk(wb_clk_i),
        .rst_n(!wb_rst_i),
        .wr_en(in_fifo_wr),
        .wr_data(in_fifo_write_data),
        .full(in_full),
        .rd_en(1'b0), // internal consumer (fir_core wrapper) reads differently - we'll expose internal pop via read port (not using rd_en)
        .rd_data(), // unused
        .empty(in_empty),
        .level()
    );

    // For out FIFO we will instantiate a fifo for outputs and read via rd_en to pop
    fifo_sync #(.WIDTH(DATA_W), .DEPTH(FIFO_DEPTH), .PTR_W(6)) out_fifo (
        .clk(wb_clk_i),
        .rst_n(!wb_rst_i),
        .wr_en(1'b0), // written by wrapper logic below
        .wr_data(32'd0),
        .full(out_full),
        .rd_en(out_fifo_rd),
        .rd_data(out_fifo_read_data),
        .empty(out_empty),
        .level()
    );

    // Note: Because fir_core outputs directly, we will connect output pushes to a small register FIFO logic
    // Rather than reworking the fifo_sync instance after instantiation (since one instance above was created),
    // we'll implement small internal logic to push fir_core outputs into the out_fifo instance using a handshake.

    // To enable simpler integration, we'll instead implement internal push/pop FIFOs with simple arrays here:
    // (BUT to keep code simple and deterministic, use small internal queue regs)
    // For clarity and safety, we will not try to modify the instantiated out_fifo's internals here.
    // Instead, we'll connect fir_core output directly to a small internal queue ('out_q') and let reads pop that.

    // Internal 'out queue' - depth 8
    reg signed [DATA_W-1:0] out_q [0:7];
    reg [2:0] out_q_wr_ptr, out_q_rd_ptr;
    reg [3:0] out_q_count;

    // Input queue (from IN_FIFO instance above): we can't directly read in_fifo from outside instance.
    // So, for reliability and simplicity: we won't rely on the in_fifo instance above; we will implement small internal IN queue here
    reg signed [DATA_W-1:0] in_q [0:FIFO_DEPTH-1];
    integer in_q_wr, in_q_rd, in_q_cnt;

    // Hook up of fir_core (internal instance)
    wire signed [DATA_W-1:0] core_in_sample;
    wire core_in_valid;
    reg core_in_ready;
    wire core_out_valid;
    wire signed [DATA_W-1:0] core_out_sample;
    reg core_start;

    // Coef write bus to core
    reg coef_wr;
    reg [5:0] coef_wr_addr;
    reg signed [COEF_W-1:0] coef_wr_data;

    // instantiate FIR core
    fir_core #(
        .DATA_W(DATA_W),
        .COEF_W(COEF_W),
        .ACC_W(ACC_W),
        .MAX_TAPS(MAX_TAPS),
        .TAPS_W(6)
    ) core (
        .clk(wb_clk_i),
        .rst_n(!wb_rst_i),
        .taps(taps_reg[5:0]),
        .start_proc(core_start),
        .coef_wr(coef_wr),
        .coef_wr_addr(coef_wr_addr),
        .coef_wr_data(coef_wr_data),
        .in_valid(core_in_valid),
        .in_sample(core_in_sample),
        .in_ready(core_in_ready),
        .out_valid(core_out_valid),
        .out_sample(core_out_sample)
    );

    // Quality-of-life: map core_in_valid to when we have data in in_q and core_in_ready high
    assign core_in_valid = (in_q_cnt > 0) && core_in_ready;
    assign core_in_sample = in_q[in_q_rd];

    // When core accepts sample, pop in_q
    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            in_q_wr <= 0;
            in_q_rd <= 0;
            in_q_cnt <= 0;
        end else begin
            if (in_fifo_wr) begin
                // NOTE: Caller writes using IN FIFO register path; we capture into in_q here
                // but the earlier in_fifo instance is unused - to avoid confusion we trust in_fifo_wr pushes
                // data here as `in_fifo_write_data` and we push into in_q.
                in_q[in_q_wr] <= in_fifo_write_data;
                in_q_wr <= (in_q_wr + 1) % FIFO_DEPTH;
                if (in_q_cnt < FIFO_DEPTH) in_q_cnt <= in_q_cnt + 1;
            end
            // core acceptance
            if (core_in_valid && core_in_ready) begin
                // pop
                in_q_rd <= (in_q_rd + 1) % FIFO_DEPTH;
                if (in_q_cnt > 0) in_q_cnt <= in_q_cnt - 1;
            end
        end
    end

    // take core outputs and push into out_q
    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            out_q_wr_ptr <= 0;
            out_q_rd_ptr <= 0;
            out_q_count <= 0;
        end else begin
            if (core_out_valid) begin
                if (out_q_count < 8) begin
                    out_q[out_q_wr_ptr] <= core_out_sample;
                    out_q_wr_ptr <= out_q_wr_ptr + 1;
                    out_q_count <= out_q_count + 1;
                end
                // irq fire if desired
                irq_o <= 1'b1;
            end else begin
                irq_o <= 1'b0;
            end

            if (out_fifo_rd && out_q_count > 0) begin
                // pop and present as wbs_dat_o on read cycle (see read logic)
                out_q_rd_ptr <= out_q_rd_ptr + 1;
                out_q_count <= out_q_count - 1;
            end
        end
    end

    // Wishbone single-cycle ack logic and register access
    always @(posedge wb_clk_i) begin
        if (wb_rst_i) begin
            wbs_ack_o <= 0;
            wbs_dat_o <= 32'd0;
            control <= 0;
            status <= 0;
            taps_reg <= 1; // default 1 tap
            start_pulse <= 0;
            coef_wr <= 0;
            coef_wr_addr <= 0;
            coef_wr_data <= 0;
            in_fifo_wr <= 0;
            in_fifo_write_data <= 32'd0;
            out_fifo_rd <= 0;
            core_start <= 0;
            core_in_ready <= 1;
        end else begin
            // default deasserts
            wbs_ack_o <= 0;
            coef_wr <= 0;
            in_fifo_wr <= 0;
            out_fifo_rd <= 0;
            core_start <= 0;
            core_in_ready <= 1;

            if (wbs_cyc_i && wbs_stb_i && !wbs_ack_o) begin
                wbs_ack_o <= 1;
                if (wbs_we_i) begin
                    // write operations
                    case (wbs_adr_i - WB_BASE)
                        ADDR_CONTROL: begin
                            control <= wbs_dat_i;
                            if (wbs_dat_i[0]) begin
                                // start pulse - begin processing next sample in core
                                core_start <= 1;
                            end
                            if (wbs_dat_i[1]) begin
                                // reset (soft)
                                // no-op for now
                            end
                        end
                        ADDR_TAPS: begin
                            taps_reg <= wbs_dat_i;
                        end
                        default: begin
                            if ((wbs_adr_i - WB_BASE) >= ADDR_COEF_BASE && (wbs_adr_i - WB_BASE) < (ADDR_COEF_BASE + 4*MAX_TAPS)) begin
                                // coefficient write (word addressed)
                                coef_wr <= 1;
                                coef_wr_addr <= (wbs_adr_i - WB_BASE - ADDR_COEF_BASE) >> 2;
                                coef_wr_data <= wbs_dat_i;
                            end else if ((wbs_adr_i - WB_BASE) == ADDR_IN_FIFO) begin
                                // push sample into input queue
                                in_fifo_write_data <= wbs_dat_i;
                                in_fifo_wr <= 1;
                            end
                        end
                    endcase
                end else begin
                    // read operations
                    case (wbs_adr_i - WB_BASE)
                        ADDR_STATUS: begin
                            // status: [0]=core busy? [1]=out_valid pending?
                            wbs_dat_o <= {30'd0, (core_out_valid?1'b1:1'b0), (mac_busy_internal()?1'b1:1'b0)};
                        end
                        ADDR_TAPS: begin
                            wbs_dat_o <= taps_reg;
                        end
                        ADDR_CONTROL: begin
                            wbs_dat_o <= control;
                        end
                        ADDR_OUT_FIFO: begin
                            // read from out_q if non-empty
                            if (out_q_count > 0) begin
                                // present data and indicate pop
                                wbs_dat_o <= out_q[out_q_rd_ptr];
                                out_fifo_rd <= 1; // pop in the same cycle (consumer must read wbs_dat_o)
                            end else begin
                                wbs_dat_o <= 32'hDEAD_BEEF;
                            end
                        end
                        default: begin
                            if ((wbs_adr_i - WB_BASE) >= ADDR_COEF_BASE && (wbs_adr_i - WB_BASE) < (ADDR_COEF_BASE + 4*MAX_TAPS)) begin
                                // read coefficient
                                wbs_dat_o <= $signed(get_coef_word((wbs_adr_i - WB_BASE - ADDR_COEF_BASE) >> 2));
                            end else begin
                                wbs_dat_o <= 32'd0;
                            end
                        end
                    endcase
                end
            end
        end
    end

    // helper function to retrieve coef word (since coefs in child module are not visible here, we maintain a shadow memory if required)
    // For now, return zero (readback of coefficients not required by core functionality).
    function [31:0] get_coef_word;
        input integer idx;
        begin
            get_coef_word = 32'd0;
        end
    endfunction

    // helper to check if core busy (we can't access mac_busy directly - approximate via in_q_cnt > 0 or core_out_valid)
    function mac_busy_internal;
        begin
            mac_busy_internal = (in_q_cnt > 0) || core_out_valid;
        end
    endfunction

endmodule
