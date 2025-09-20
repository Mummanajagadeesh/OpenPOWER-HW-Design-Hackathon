// verilog/dv/tb_fir_top.sv
`timescale 1ns/1ps

module tb_fir_top;
  logic clk = 0;
  logic rst_n = 0;

  // fir signals
  logic start;
  logic signed [31:0] sample_in;
  logic signed [31:0] sample_out;
  logic done;

  // coeff write ports
  logic coeff_wr_en;
  logic [2:0] coeff_wr_addr;
  logic signed [31:0] coeff_wr_data;

  // Instantiate the FIR core (TAPS=8)
  fir_accel #(.TAPS(8), .WIDTH(32)) uut (
    .clk(clk),
    .rst_n(rst_n),
    .start(start),
    .sample_in(sample_in),
    .sample_out(sample_out),
    .done(done),
    .coeff_wr_en(coeff_wr_en),
    .coeff_wr_addr(coeff_wr_addr),
    .coeff_wr_data(coeff_wr_data)
  );

  // local model storage
  int signed coeffs[0:7];
  int signed samples[0:7]; // circular buffer
  int sample_ptr = 0;

  // clock
  always #5 clk = ~clk; // 100 MHz-ish

  // helper: write coeff
  task automatic write_coeff(input int idx, input int signed val);
    begin
      coeffs[idx] = val; // mirror in TB
      @(posedge clk);
      coeff_wr_en = 1;
      coeff_wr_addr = idx[2:0];
      coeff_wr_data = val;
      @(posedge clk);
      coeff_wr_en = 0;
      coeff_wr_addr = 0;
      coeff_wr_data = 0;
    end
  endtask

  // helper: golden FIR calculation (mask to lower 32 bits)
// golden FIR with saturation
function automatic int signed golden(int signed new_sample);
  int signed acc;
  int i;
  int signed sat_out;
  begin
    samples[sample_ptr] = new_sample;
    acc = 0;
    for (i=0; i<8; i++) begin
      int idx = (sample_ptr - i);
      if (idx < 0) idx = idx + 8;
      acc += samples[idx] * coeffs[i];
    end
    sample_ptr = (sample_ptr + 1) % 8;

    // saturate to 32-bit signed
    if (acc > 32'sh7FFFFFFF)
      sat_out = 32'sh7FFFFFFF;
    else if (acc < -32'sh80000000)
      sat_out = -32'sh80000000;
    else
      sat_out = acc;

    return sat_out;
  end
endfunction


  // helper: feed sample, check output
  task automatic feed_sample(input int signed s);
    int signed exp, got;
    begin
      exp = golden(s);

      @(posedge clk);
      sample_in = s;
      start = 1;
      @(posedge clk);
      start = 0;

      wait (done == 1);
      @(posedge clk);
      got = sample_out;

      if (got === exp)
        $display("PASS @%0t: sample_in=%0d => got=%0d (expected=%0d)", $time, s, got, exp);
      else
        $display("FAIL @%0t: sample_in=%0d => got=%0d (expected=%0d)", $time, s, got, exp);
    end
  endtask

  initial begin
    // dump waves
    $dumpfile("fir_tb.vcd");
    $dumpvars(0, tb_fir_top);

    // reset
    rst_n = 0;
    coeff_wr_en = 0;
    coeff_wr_addr = 0;
    coeff_wr_data = 0;
    sample_in = 0;
    start = 0;
    for (int i=0; i<8; i++) samples[i] = 0;
    #20;
    rst_n = 1;
    #20;

    // === Test 1: Moving average (coeffs = 1,1,1,1,0,0,0,0) ===
    $display("=== Test 1: Moving average ===");
    for (int i=0; i<8; i++) write_coeff(i, (i<4)?1:0);
    feed_sample(100);
    feed_sample(200);
    feed_sample(300);
    feed_sample(400);

    // === Test 2: All ones (simple sum) ===
    $display("=== Test 2: All ones ===");
    for (int i=0; i<8; i++) write_coeff(i, 1);
    feed_sample(10);
    feed_sample(20);
    feed_sample(30);
    feed_sample(40);

    // === Test 3: Alternating (+1, -1, +1, -1, ...) ===
    $display("=== Test 3: Alternating ===");
    for (int i=0; i<8; i++) write_coeff(i, (i%2==0)?1:-1);
    feed_sample(50);
    feed_sample(-50);
    feed_sample(25);
    feed_sample(-25);

    $display("All tests done");
    #50;
    $finish;
  end

endmodule
