// user_proj_fir.v
// Minimal top-level wrapper to be instantiated inside the OpenFrame/user_project area.
// Exposes the typical Caravel Wishbone slave port names (wbs_*).
`timescale 1ns / 1ps

module user_proj_fir (
`ifdef USE_POWER_PINS
    inout vdda1,
    inout vdda2,
    inout vssa1,
    inout vssa2,
    inout vccd1,
    inout vccd2,
    inout vssd1,
    inout vssd2,
`endif

    // Wishbone Slave ports (connected from openframe wrapper)
    input  wire          wbs_clk_i,
    input  wire          wbs_rst_i,
    input  wire          wbs_cyc_i,
    input  wire          wbs_stb_i,
    input  wire          wbs_we_i,
    input  wire  [3:0]   wbs_sel_i,
    input  wire  [31:0]  wbs_adr_i,
    input  wire  [31:0]  wbs_dat_i,
    output wire  [31:0]  wbs_dat_o,
    output wire          wbs_ack_o,

    // GPIOs left unconnected for now — expose if you want to route debug/IRQ to pins.
    output wire          user_irq
);

    // Instantiate fir_wb using internal clock (use wbs_clk_i)
    fir_wb #(
        .DATA_W(32),
        .COEF_W(32),
        .ACC_W(64),
        .MAX_TAPS(64),
        .FIFO_DEPTH(64)
    ) FIR0 (
        .wb_clk_i (wbs_clk_i),
        .wb_rst_i (wbs_rst_i),
        .wbs_cyc_i(wbs_cyc_i),
        .wbs_stb_i(wbs_stb_i),
        .wbs_we_i (wbs_we_i),
        .wbs_sel_i(wbs_sel_i),
        .wbs_adr_i(wbs_adr_i),
        .wbs_dat_i(wbs_dat_i),
        .wbs_dat_o(wbs_dat_o),
        .wbs_ack_o(wbs_ack_o),
        .irq_o(user_irq)
    );

endmodule
