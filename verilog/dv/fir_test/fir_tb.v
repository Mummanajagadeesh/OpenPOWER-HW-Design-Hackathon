// fir_tb.v
`timescale 1ns/1ps
module fir_tb;

    reg clk;
    reg rst;

    // Wishbone signals
    reg cyc, stb, we;
    reg [3:0] sel;
    reg [31:0] adr;
    reg [31:0] dat_i;
    wire [31:0] dat_o;
    wire ack;
    wire irq;

    // FIR wrapper
    fir_wb #(
        .DATA_W(32),
        .COEF_W(32),
        .ACC_W(64),
        .MAX_TAPS(8),
        .FIFO_DEPTH(32)
    ) dut (
        .wb_clk_i(clk),
        .wb_rst_i(rst),
        .wbs_cyc_i(cyc),
        .wbs_stb_i(stb),
        .wbs_we_i(we),
        .wbs_sel_i(sel),
        .wbs_adr_i(adr),
        .wbs_dat_i(dat_i),
        .wbs_dat_o(dat_o),
        .wbs_ack_o(ack),
        .irq_o(irq)
    );

    integer i, j;
    reg signed [31:0] coefs [0:7];
    reg signed [31:0] samples [0:31];
    reg signed [31:0] sample_window [0:7];
    reg [31:0] outval;
    reg [31:0] status;
    integer golden;

    parameter NUM_TAPS = 3;

    // Wishbone write
    task wb_write(input [31:0] a, input [31:0] d);
    begin
        cyc = 1; stb = 1; we = 1; sel = 4'hF; adr = a; dat_i = d;
        @(posedge clk);
        while (!ack) @(posedge clk);
        cyc = 0; stb = 0; we = 0; sel = 0; adr = 0; dat_i = 0;
        @(posedge clk);
    end
    endtask

    // Wishbone read
    task wb_read(input [31:0] a, output [31:0] dout);
    begin
        cyc = 1; stb = 1; we = 0; sel = 4'hF; adr = a;
        @(posedge clk);
        while (!ack) @(posedge clk);
        dout = dat_o;
        cyc = 0; stb = 0; adr = 0;
        @(posedge clk);
    end
    endtask

    // Clock
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        // Reset
        rst = 1;
        cyc = 0; stb = 0; we = 0; sel = 0; adr = 0; dat_i = 0;
        #100;
        rst = 0;

        // Configure taps
        wb_write(32'h08, NUM_TAPS);

        // Write coefficients
        wb_write(32'h10 + 0*4, 32'd1);
        wb_write(32'h10 + 1*4, 32'd2);
        wb_write(32'h10 + 2*4, 32'd3);
        coefs[0] = 1; coefs[1] = 2; coefs[2] = 3;

        // Generate sample vector
        for (i=0; i<8; i=i+1) samples[i] = i;

        // Initialize sample window
        for (j=0;j<NUM_TAPS;j=j+1) sample_window[j] = 0;

        // Send samples
        for (i=0; i<8; i=i+1) begin
            // Shift sample window
            for (j=NUM_TAPS-1;j>0;j=j-1) sample_window[j] = sample_window[j-1];
            sample_window[0] = samples[i];

            // Push sample
            wb_write(32'h80, samples[i]);
            wb_write(32'h00, 32'h1); // start FIR

            // Wait for FIR to compute (poll OUT_VALID)
            status = 0;
            while ((status & 2'b10) == 0) begin
                wb_read(32'h04, status);
                @(posedge clk);
            end

            // Read FIR output
            wb_read(32'h84, outval);

            // Compute golden aligned with FIR latency
            golden = 0;
            for (j=0;j<NUM_TAPS;j=j+1)
                golden = golden + sample_window[j]*coefs[j];

            $display("i=%0d out=%0d golden=%0d", i, $signed(outval), golden);
        end

        $display("Test finished");
        #100;
        $finish;
    end

endmodule
