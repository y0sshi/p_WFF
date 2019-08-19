`default_nettype none
`timescale 1ns/1ns

module BRAM_FIFO
  #(
    parameter integer FIFO_WIDTH = -1,
    parameter integer W_WIDTH = -1,
    parameter integer DATA_WIDTH = -1
  )
  (
    clk,
    n_rst,
    w_en,
    r_en,
    w_data,
    //full,
    r_data
  );

  localparam MEM_WIDTH = log2(FIFO_WIDTH);

  input wire                   clk;
  input wire                   n_rst;
  input wire                   w_en;
  input wire                   r_en;
  input wire  [DATA_WIDTH-1:0] w_data;
  //output wire                  full;
  output wire [DATA_WIDTH-1:0] r_data;

  //---register and wire declaration---//
    reg  [MEM_WIDTH-1:0] w_addr = 16'd0;
    reg  [MEM_WIDTH-1:0] r_addr = 16'd0;
    reg  [MEM_WIDTH-1:0] w_count = 16'd0;
    reg                     ena = 1'b1;
    wire [DATA_WIDTH-1:0]   doa ;
    reg                     full_flag = 1'b0;
  //---register and wire declaration---//

  BRAM
  #(
    .FIFO_WIDTH(FIFO_WIDTH),
    .MEM_WIDTH(MEM_WIDTH),
    .DATA_WIDTH(DATA_WIDTH)
  )
  RAM
  (
    .clk(clk),
    .n_rst(n_rst),
    .enb(r_en),
    .wea(w_en),
    .addra(w_addr),
    .addrb(r_addr),
    .dia(w_data),
    .dob(r_data),
    .ena(ena),
    .doa(doa)
  );

  always @(posedge clk or negedge n_rst) begin
    if(!n_rst)begin
      w_addr <= 16'd0;
      r_addr <= 16'd0;
      full_flag <= 1'b0;
      w_count <= 16'd0;
    end
    else begin
      w_count <= w_addr - r_addr;
      if (w_count >= W_WIDTH - 4) begin
        full_flag <= 1'b1;
      end
      else begin
        full_flag <= 1'b0;
      end
      if(w_en)begin
        if (FIFO_WIDTH == 1) begin
          w_addr <= 'd0;
        end
        else if(w_addr < FIFO_WIDTH - 1)begin
          w_addr <= w_addr + 1'b1;
        end
        else begin
          w_addr <= 'd0;
        end
      end
      if(r_en)begin
        if (FIFO_WIDTH == 1) begin
          r_addr <= 'd0;
        end
        else if(r_addr < FIFO_WIDTH - 1)begin
          r_addr <= r_addr + 1'b1;
        end
        else begin
          r_addr <= 16'd0;
        end
      end
    end
  end

  function integer log2;
    input integer value;
    begin
      if (value == 1) begin
        log2 = 1;
      end
      else begin
        value = value - 1;
        for (log2 = 0; value > 0; log2 = log2 + 1) begin
          value = value >> 1;
        end
      end
    end
  endfunction // log2
endmodule

`default_nettype wire
