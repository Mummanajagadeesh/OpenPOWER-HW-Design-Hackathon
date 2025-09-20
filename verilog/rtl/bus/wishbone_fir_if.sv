// verilog/rtl/bus/wishbone_fir_if.sv
// Minimal Wishbone-lite slave wrapper for the FIR accelerator.
// This handles register reads/writes and sends coeff writes to the core.
//
// Address map (offsets):
// 0x00 - CTRL (write start bit[0])
// 0x04 - DATAIN (write sample input)
// 0x08 - DATAOUT (read result)
// 0x0C - STATUS (bit0 = done)
// 0x10 + 4*i - COEFF i (write coeffs)
module wishbone_fir_if #(
  parameter integer BASE = 32'h4000_0000
)(
  input  logic         clk,
  input  logic         rst_n,

  // Wishbone-lite signals (single-cycle ack)
  input  logic         wb_cyc_i,
  input  logic         wb_stb_i,
  input  logic         wb_we_i,
  input  logic [31:0]  wb_adr_i,
  input  logic [31:0]  wb_wdata_i,
  output logic [31:0]  wb_rdata_o,
  output logic         wb_ack_o,

  // FIR core interface
  output logic                     start,
  output logic signed [31:0]       sample_in,
  input  logic signed [31:0]       sample_out,
  input  logic                     done,

  // coeff write interface
  output logic                     coeff_wr_en,
  output logic [3:0]               coeff_wr_addr, // supports up to 16 taps for safety
  output logic signed [31:0]       coeff_wr_data
);

  // local regs
  logic [31:0] reg_ctrl;
  logic [31:0] reg_data_in;

  // default outputs
  assign start = reg_ctrl[0];

  // simple ack logic
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wb_ack_o <= 0;
      wb_rdata_o <= 32'h0;
      reg_ctrl <= 32'h0;
      reg_data_in <= 32'h0;
      coeff_wr_en <= 0;
      coeff_wr_addr <= 0;
      coeff_wr_data <= 0;
      sample_in <= 0;
    end else begin
      wb_ack_o <= 0;
      coeff_wr_en <= 0;
      if (wb_cyc_i && wb_stb_i && !wb_ack_o) begin
        wb_ack_o <= 1;
        wb_rdata_o <= 32'h0;
        case (wb_adr_i[7:0])
          8'h00: begin // CTRL
            if (wb_we_i) begin
              reg_ctrl <= wb_wdata_i;
            end
            wb_rdata_o <= reg_ctrl;
          end
          8'h04: begin // DATAIN
            if (wb_we_i) begin
              reg_data_in <= wb_wdata_i;
              sample_in <= wb_wdata_i;
            end
            wb_rdata_o <= 32'h0;
          end
          8'h08: begin // DATAOUT
            wb_rdata_o <= sample_out;
          end
          8'h0C: begin // STATUS
            wb_rdata_o <= {31'b0, done};
          end
          default: begin
            // coefficient region starting at 0x10
            if (wb_adr_i[7:0] >= 8'h10) begin
              int idx;
              idx = (wb_adr_i[7:0] - 8'h10) >> 2;
              if (wb_we_i) begin
                coeff_wr_en <= 1;
                coeff_wr_addr <= idx[3:0];
                coeff_wr_data <= wb_wdata_i;
              end
              wb_rdata_o <= 32'h0;
            end else begin
              wb_rdata_o <= 32'hDEAD_BEEF;
            end
          end
        endcase
      end
    end
  end

endmodule
