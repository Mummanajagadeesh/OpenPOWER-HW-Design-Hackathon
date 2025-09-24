// fifo_sync.v
// Simple synchronous FIFO (write-side and read-side same clock).
// Depth must be a power of two for simple pointer wrap (but it isn't required strictly).

`timescale 1ns / 1ps
module fifo_sync #(
    parameter WIDTH = 32,
    parameter DEPTH = 16,
    parameter PTR_W = 4
)(
    input  wire            clk,
    input  wire            rst_n,
    // write interface
    input  wire            wr_en,
    input  wire [WIDTH-1:0] wr_data,
    output wire            full,
    // read interface
    input  wire            rd_en,
    output reg [WIDTH-1:0] rd_data,
    output wire            empty,
    output wire [PTR_W:0]  level  // optional occupancy
);
    reg [WIDTH-1:0] mem [0:DEPTH-1];
    reg [PTR_W-1:0] wr_ptr;
    reg [PTR_W-1:0] rd_ptr;
    reg [PTR_W:0]   count;

    integer i;
    always @(posedge clk) begin
        if (!rst_n) begin
            wr_ptr <= 0;
            rd_ptr <= 0;
            count <= 0;
            for (i=0;i<DEPTH;i=i+1) mem[i] <= 0;
            rd_data <= 0;
        end else begin
            if (wr_en && (count < DEPTH)) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
                count <= count + 1;
            end
            if (rd_en && (count > 0)) begin
                rd_data <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1;
                count <= count - 1;
            end
        end
    end

    assign full = (count == DEPTH);
    assign empty = (count == 0);
    assign level = count;

endmodule
