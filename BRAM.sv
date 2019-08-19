`default_nettype none
`timescale 1ns/1ns

module BRAM
  #(
    parameter integer FIFO_WIDTH = -1,
    parameter integer MEM_WIDTH = log2(FIFO_WIDTH),
    parameter integer DATA_WIDTH = -1
  )
  (
    clk,
    n_rst,
    ena,
    enb,
    wea,
    addra,
    addrb,
    dia,
    doa,
    dob
  );

  //localparam MEM_WIDTH = log2(WIDTH);

  input wire         clk;
  input wire         n_rst;
  input wire         ena;
  input wire         enb;
  input wire         wea;
  input wire  [MEM_WIDTH-1:0] addra;
  input wire  [MEM_WIDTH-1:0] addrb;
  input wire  [DATA_WIDTH-1:0] dia;
  output wire [DATA_WIDTH-1:0] doa;
  output wire [DATA_WIDTH-1:0] dob;

  //---register and wire declaration---//
  reg [DATA_WIDTH-1:0] RAM [FIFO_WIDTH-1:0];
  //reg [MEM_WIDTH-1:0] read_addra;
  reg [MEM_WIDTH-1:0] read_addra;
  reg [MEM_WIDTH-1:0] read_addrb;
  //---register and wire declaration---//

  //-------output assign-------//
  assign doa = RAM[read_addra];
  assign dob = RAM[read_addrb];
  //-------output assign-------//
  
  always @(posedge clk) begin
    if(!n_rst)begin
      //assign_odata <= 16'd0;
      //read_addra <= 1'b0;
      //read_addrb <= 1'b0;
    end
    else begin
      if(ena)begin
        if(wea)begin
          RAM[addra] <= dia;
        end
        read_addra <= addra;
      end
      if(enb)begin
        read_addrb <= addrb;
      end
    end
  end

  function integer log2;
    input integer value;
    begin
      value = value - 1;
      for (log2 = 0; value > 0; log2 = log2 + 1) begin
        value = value >> 1;
      end
    end
  endfunction // log2
endmodule

`default_nettype wire
