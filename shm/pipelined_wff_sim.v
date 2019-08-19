`default_nettype none
`timescale 1ns/1ns

`define SIM4

module p_wff_sim();
  localparam integer CYCLE    = 5; //200MHz
  localparam [100:0] N_REPEAT = 1000000000;


  `ifdef SIM0 //12x12x7_0
  localparam integer MAX_FLOW = 3521; 
  localparam integer INIT_FLOW = 3620;
  localparam integer X = 12;
  localparam integer Y = 12;
  localparam integer Z = 7;
  localparam integer CNT_LIMIT = 50;

  `elsif SIM1 //12x12x7_1
  localparam integer MAX_FLOW = 3683; 
  localparam integer INIT_FLOW = 3788;
  localparam integer X = 12;
  localparam integer Y = 12;
  localparam integer Z = 7;
  localparam integer CNT_LIMIT = 90;

  `elsif SIM2 //12x12x7_2
  localparam integer MAX_FLOW = 52501; 
  localparam integer INIT_FLOW = 56887;
  localparam integer X = 12;
  localparam integer Y = 12;
  localparam integer Z = 7;
  localparam integer CNT_LIMIT = 20;

  `elsif SIM3 //64x64x8_0
  localparam integer MAX_FLOW = 650207; 
  localparam integer INIT_FLOW = 766324;
  localparam integer X = 64;
  localparam integer Y = 64;
  localparam integer Z = 8;
  localparam integer CNT_LIMIT = 86;
  //localparam integer CNT_LIMIT = 200;

  `elsif SIM4 // 129x129x16_0
  localparam integer MAX_FLOW = 545133; 
  localparam integer INIT_FLOW = 1971855;
  localparam integer X = 129;
  localparam integer Y = 129;
  localparam integer Z = 16;
  localparam integer CNT_LIMIT = 211;
  //localparam integer CNT_LIMIT = 400;

  `elsif SIM5 // 129x129x16_1
  localparam integer MAX_FLOW = 597455; 
  localparam integer INIT_FLOW = 1454850;
  localparam integer X = 129;
  localparam integer Y = 129;
  localparam integer Z = 16;
  //localparam integer CNT_LIMIT = 250;
  localparam integer CNT_LIMIT = 400;

  `elsif SIM6 // 129x129x16_2
  localparam integer MAX_FLOW = 1621425;
  localparam integer INIT_FLOW = 1837329;
  localparam integer X = 129;
  localparam integer Y = 129;
  localparam integer Z = 16;
  localparam integer CNT_LIMIT = 400;

  `elsif SIM //default
  localparam integer MAX_FLOW = 52501; 
  localparam integer INIT_FLOW = 56887;
  localparam integer X = 12;
  localparam integer Y = 12;
  localparam integer Z = 7;
  //localparam integer CNT_LIMIT = 50;
  localparam integer CNT_LIMIT = 20;
  `endif


  localparam integer XY = X*Y;
  localparam integer NODE_NUM = X*Y*Z;
  localparam integer EX_WIDTH = 14;
  localparam integer WV_WIDTH = 10;
  localparam integer CF_WIDTH = 10;
  localparam integer MF_WIDTH = 22;
  localparam integer ADDR_WIDTH = log2(NODE_NUM);
  localparam integer DIVIDE = 7;

  localparam integer DATA_WIDTH = EX_WIDTH+WV_WIDTH+(CF_WIDTH*4)+8;


  reg 		      clk;
  reg 		      n_rst;

  reg [MF_WIDTH-1:0] max_flow = INIT_FLOW;   
  reg [ADDR_WIDTH-1:0] cnt = 0;

  reg     start = 1;
  wire 		finish;
  wire 		out_flag;


  /*clock*/
  initial begin
    clk <= 1'b1;   
    forever
    #(CYCLE/'d2)
    clk <= ~clk;
  end

  /*reset*/
  initial begin
    n_rst <= 1'b1;   
    repeat('d2)
    #(CYCLE) 
    start <= 1;
    n_rst <= ~n_rst;
  end

  /* repeat time */
  initial begin
    #N_REPEAT
    $finish;
  end

  p_wff
  #(
    .X(X),
    .Y(Y),
    .Z(Z),
    .DIVIDE(DIVIDE),
    .CNT_LIMIT(CNT_LIMIT)
  )
  p_wff_inst0(
    .clk(clk),
    .n_rst(n_rst),
    .start(1'b1),
    .out_flag(out_flag),
    .finish(finish)
  );

  genvar i,j;

  always @(posedge clk)begin
    if( finish )begin
      cnt <= (cnt == XY) ? cnt : cnt + 1;
      //if( check_exf(p_wff_inst0.BRAM_FIFO_blk[Z-1].FIFO.RAM.RAM[cnt][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH])&(cnt < XY) )begin
      //  max_flow <= add(max_flow, p_wff_inst0.BRAM_FIFO_blk[Z-1].FIFO.RAM.RAM[cnt][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH]);
      //end
      if( check_exf(p_wff_inst0.e_r_sink[cnt])&(cnt < XY) )begin
        max_flow <= add(max_flow, p_wff_inst0.e_r_sink[cnt]);
      end

      if( cnt == XY )begin
        $display("============ FINISH ===============\n");
        if( max_flow == MAX_FLOW )begin
          $display($time,,,"SUCCESS : max_flow = %d\n", max_flow);
        end
        else begin
          $display($time,,,"ERROR : max_flow = %d\n", max_flow);
        end
        $finish;
      end
    end
    else begin
    end
  end

  //reg signed [MF_WIDTH+100:0] total_flow_reg;
  //wire signed [MF_WIDTH+100:0] total_flow_sim [NODE_NUM-1:0];

  //assign total_flow_sim[0] = $signed(p_wff_inst0.e_r_sink[0]);
  //generate
  //for (i=1; i<NODE_NUM; i=i+1) begin
  //  assign total_flow_sim[i] = $signed(p_wff_inst0.e_r_sim[i]) + total_flow_sim[i-1];
  //end
  //endgenerate

  //always @(posedge clk) begin
  //  if ((p_wff_inst0.state==2)&&(p_wff_inst0.node_cnt_manage==0)) begin
  //    total_flow_reg <= total_flow_sim[NODE_NUM-1];
  //  end
  //end

  // BRAM_FIFO_blk[0]
  //wire signed [MF_WIDTH+100:0] total_flow_sim_0 [XY-1:0];
  //assign total_flow_sim_0[0] = $signed(p_wff_inst0.BRAM_FIFO_blk[0].FIFO.RAM.RAM[0][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH]);
  //for (j=1; j<XY; j=j+1) begin
  //  assign total_flow_sim_0[j] = $signed(p_wff_inst0.BRAM_FIFO_blk[0].FIFO.RAM.RAM[j][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH]) + total_flow_sim_0[j-1];
  //end
  //assign total_flow_sim[0] = total_flow_sim_0[XY-1];

  // BRAM_FIFO_blk[i] (i > 0)
  //generate
  //for (i=1; i<Z; i=i+1) begin
  //  wire signed [MF_WIDTH+100:0] total_flow_sim_z [XY-1:0];
  //  assign total_flow_sim_z[0] = $signed(p_wff_inst0.BRAM_FIFO_blk[i].FIFO.RAM.RAM[0][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH]);
  //  for (j=1; j<XY; j=j+1) begin
  //    assign total_flow_sim_z[j] = $signed(p_wff_inst0.BRAM_FIFO_blk[i].FIFO.RAM.RAM[j][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH]) + total_flow_sim_z[j-1];
  //  end
  //  assign total_flow_sim[i] = total_flow_sim[i-1] + total_flow_sim_z[XY-1];
  //end
  //endgenerate


  initial begin
    $shm_open("p_wff.shm");
    $shm_probe("AM",p_wff_inst0);
  end

  //function ==================================================================
  //check function ------------------------------------------------------------
  function check_exf;// e < 0
    input signed [EX_WIDTH-1:0] a;
    begin
      check_exf = (a[EX_WIDTH-1]) ? 1 : 0;
    end
  endfunction

  function [MF_WIDTH-1:0] add;
    input signed [MF_WIDTH-1:0] a;
    input signed [EX_WIDTH-1:0] b;
    begin
      add = a + b;
    end
  endfunction

 function integer log2;
   input integer   value;
  begin
    value = value-1;
    for (log2=0; value>0; log2=log2+1)
      value = value>>1;
    end
  endfunction
endmodule
