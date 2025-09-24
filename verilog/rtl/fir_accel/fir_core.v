// fir_core.v
// Parameterizable sequential FIR core.
// - Accepts input sample when in_valid && !busy
// - Takes `taps` cycles (one MAC per cycle) to compute output
// - Output available when out_valid asserted for one cycle
// - Coefficients are held in 'coefs' memory (written by wrapper)

`timescale 1ns / 1ps
module fir_core #(
    parameter DATA_W   = 32,
    parameter COEF_W   = 32,
    parameter ACC_W    = 64,
    parameter MAX_TAPS = 64,    // compile-time max taps
    parameter TAPS_W   = 6      // log2(MAX_TAPS) bits to index taps
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // config / control
    input  wire [TAPS_W-1:0]        taps,       // active taps (1..MAX_TAPS)
    input  wire                     start_proc, // start computing for the input sample last written

    // coefficient write interface (wrapper writes into coefs via this bus)
    input  wire                     coef_wr,
    input  wire [TAPS_W-1:0]        coef_wr_addr,
    input  wire signed [COEF_W-1:0] coef_wr_data,

    // streaming input
    input  wire                     in_valid,
    input  wire signed [DATA_W-1:0] in_sample,
    output reg                      in_ready,   // accepted when high

    // streaming output
    output reg                      out_valid,
    output reg signed [DATA_W-1:0]  out_sample // saturated/truncated to DATA_W
);

    // coefficient memory
    reg signed [COEF_W-1:0] coefs [0:MAX_TAPS-1];
    integer i;
    // internal sample shift register (holds most recent MAX_TAPS samples)
    reg signed [DATA_W-1:0] sample_shift [0:MAX_TAPS-1];

    // snapshot buffer used during MAC so incoming shifts do not corrupt in-progress MAC
    reg signed [DATA_W-1:0] sample_buf [0:MAX_TAPS-1];

    // MAC state
    reg [TAPS_W-1:0] mac_idx;
    reg mac_busy;
    reg signed [ACC_W-1:0] acc;

    // coef write
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i=0;i<MAX_TAPS;i=i+1) coefs[i] <= 0;
        end else begin
            if (coef_wr) begin
                if (coef_wr_addr < MAX_TAPS)
                    coefs[coef_wr_addr] <= coef_wr_data;
            end
        end
    end

    // sample shift and accept
    always @(posedge clk) begin
        if (!rst_n) begin
            for (i=0;i<MAX_TAPS;i=i+1) sample_shift[i] <= 0;
            in_ready <= 1;
        end else begin
            // by default allow new sample when not busy computing
            in_ready <= !mac_busy;
            if (in_valid && in_ready) begin
                // shift down the array: highest index <- previous
                for (i=MAX_TAPS-1;i>0;i=i-1) sample_shift[i] <= sample_shift[i-1];
                sample_shift[0] <= in_sample;
                // signal to start processing: wrapper will assert start_proc if it wants processing
                // (processing will be started by start_proc signal)
            end
        end
    end

    // Start processing on start_proc (snapshot current shift register into sample_buf)
    always @(posedge clk) begin
        if (!rst_n) begin
            mac_busy <= 0;
            mac_idx <= 0;
            acc <= 0;
        end else begin
            if (start_proc && !mac_busy) begin
                // copy sample_shift into local buffer for stable MAC
                for (i=0;i<MAX_TAPS;i=i+1) sample_buf[i] <= sample_shift[i];
                mac_busy <= 1;
                mac_idx <= 0;
                acc <= 0;
            end else if (mac_busy) begin
                // perform one MAC per cycle
                // only accumulate up to 'taps' entries
                if (mac_idx < taps) begin
                    acc <= acc + $signed(sample_buf[mac_idx]) * $signed(coefs[mac_idx]);
                    mac_idx <= mac_idx + 1;
                end
                if ((mac_idx + 1) >= taps) begin
                    // last accumulation will complete next cycle: schedule completion
                    // We let completion be handled below when mac_idx == taps
                end
                if (mac_idx == taps) begin
                    // finished
                    mac_busy <= 0;
                end
            end
        end
    end

    // produce output when MAC finishes - register output one cycle after done
    reg signed [ACC_W-1:0] acc_reg;
    always @(posedge clk) begin
        if (!rst_n) begin
            out_valid <= 0;
            out_sample <= 0;
            acc_reg <= 0;
        end else begin
            if (!mac_busy && (mac_idx == taps) && (taps != 0)) begin
                // capture acc into acc_reg and make out_valid next cycle
                acc_reg <= acc;
                out_valid <= 1;
                // saturation/truncate to DATA_W signed
                // simple saturation:
                if (acc_reg > $signed({1'b0, {(ACC_W-DATA_W){1'b1}}, {DATA_W{1'b0}}})) begin
                    out_sample <= $signed({1'b0, {(DATA_W-1){1'b1}}});
                end else if (acc_reg < -$signed({1'b0, {(ACC_W-DATA_W){1'b1}}, {DATA_W-1{1'b0}}})) begin
                    out_sample <= $signed({1'b1, {(DATA_W-1){1'b0}}});
                end else begin
                    out_sample <= acc_reg[DATA_W-1:0];
                end
                // reset mac_idx to avoid repeating output
                mac_idx <= 0;
            end else begin
                out_valid <= 0;
            end
        end
    end

endmodule
