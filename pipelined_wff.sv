//=============================================================================//
// Pipelined Wave Front Fetch module
// Date: 2019/01/28
// Note: one clock processing
// Version 2.11
//=============================================================================//

//================================== PLACE ====================================//
//                         Sside     Middle    Tside       x-z
//                        | | | |   | |9| |   | | | |    |5|3|7|
//                        |6|2|4|   |0|u|1|   |5|3|7| => |0|u|1|
//                        | | | |   | |8| |   | | | |    |6|2|4|
//=============================================================================//

//============================== Data Structure ===============================//
//  
//      __________________________________________________________________
//     | excess flow | wave_num | cf_9 | cf_7 | cf_5 | cf_3 | cf_2 | cf_1 |
//     |      14     |    10    |   4  |  10  |  10  |  10  |  10  |   4  |
//     |<------------------------------ 72 ------------------------------>|
//
//=============================================================================//

`default_nettype none
`timescale 1ns/1ns
`define SIM4

module p_wff
  #(
    parameter X         = 129, // WIDTH
    parameter Y         = 129, // HEIGHT
    parameter Z         = 16, // DEPTH
    parameter DIVIDE    = 7, // divide nodes
    parameter WV_WIDTH  = 10,
    parameter CF_WIDTH  = 10,
    parameter EX_WIDTH  = 14,
    parameter CNT_LIMIT = 100
  )
  (
    input wire   clk,
    input wire   n_rst,
    input wire   start,
    output logic out_flag,
    output logic finish
  );

  //============================= parameter =============================//
  localparam integer XY          = X * Y;
  localparam integer STATE_WIDTH = 2;
  localparam integer NEIGHBOR    = 10;
  localparam integer NODE_NUM    = X*Y*Z;
  localparam integer CNT_WIDTH   = $clog2(CNT_LIMIT);
  localparam integer PLACE_WIDTH = NEIGHBOR;
  localparam integer ADDR_WIDTH  = $clog2(NODE_NUM);
  localparam integer PENALTY     = 7;
  localparam integer PENALTY2    = PENALTY << 1;
  localparam integer INHIBIT     = {CF_WIDTH{1'b1}};

  localparam integer STAGE_MAX   = 11;
  localparam integer STAGE_WIDTH = 4;
  localparam integer FRAME_STAGE = (Y<12) ? 12 : Y;
  localparam integer PLANE       = X * Z;
  localparam integer PLANE_WIDTH = $clog2(PLANE);
  localparam integer FIFO_WIDTH  = XY;
  localparam integer DATA_WIDTH  = EX_WIDTH+WV_WIDTH+(CF_WIDTH*4)+(4*2);

  localparam integer EDGE_MAX_S1_2  = Z;
  localparam integer EDGE_MAX_S3  = Z/2;
  localparam integer EDGE_MAX_S4  = (Z-1)/2;
  localparam integer EDGE_MAX_S5_6  = Z-1;
  localparam integer EDGE_MAX_S7_8  = Z-1;
  localparam integer EDGE_MAX_S0_9  = Z;

  wire [9:0] f_w = FIFO_WIDTH;
  wire [9:0] cnt_lim = CNT_LIMIT;

  //=============================== STATE ===============================//
  localparam [STATE_WIDTH-1:0] INIT_STATE   = 0;
  localparam [STATE_WIDTH-1:0] LOAD_STATE   = 1;
  localparam [STATE_WIDTH-1:0] PROCESS_STATE = 2;
  localparam [STATE_WIDTH-1:0] FINISH_STATE = 3;

  //============================ Register ===============================//
  reg [STATE_WIDTH-1:0] state = INIT_STATE;
  reg [CNT_WIDTH-1:0]   done_cnt;
  reg [ADDR_WIDTH-1:0]  cnt;
  reg [23:0] clk_cnt = 0;

  reg [$clog2(XY-1):0] node_cnt_shift_reg [X-10:0];
  reg [$clog2(XY-1):0] node_cnt_0,node_cnt_1,node_cnt_2,node_cnt_3,
                      node_cnt_4,node_cnt_5,node_cnt_6,node_cnt_7,
                      node_cnt_8,node_cnt_9,node_cnt_manage;

  reg process_en_s12, process_en_s3, process_en_s4, process_en_s56, 
      process_en_s78, process_en_s09, process_en_manage;

  reg [EX_WIDTH-1:0] e_r_sink [XY-1:0];

  integer i,j;
  genvar g,h;
  
  // Data read and initialization BRAM
  reg clk_div, r_en=1, w_en;
  wire [DATA_WIDTH-1:0] w_data[Z], r_data[Z];

  always @(posedge clk) begin
    if (n_rst) begin
      clk_div <= 1'b0;
    end
    else begin
      clk_div <= (!state[0]);
    end
  end

  generate
  for (g=0; g<Z; g=g+1) begin : BRAM_FIFO_blk
    BRAM_FIFO
    #
    (
      .FIFO_WIDTH(FIFO_WIDTH),
      .W_WIDTH(FIFO_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
    )
    FIFO
    (
      .clk(clk),
      .n_rst(!n_rst),
      .w_en(w_en),
      .r_en(r_en),
      .w_data(w_data[g]),
      .r_data(r_data[g])
    );
  end : BRAM_FIFO_blk
  endgenerate

  `ifdef SIM // Simulation default
  initial begin
    $readmemb("../../data_set/hw_wff_graph/default/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/default/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
  end
  `elsif SIM0 // Simulation 0
  initial begin
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_0/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
  end
  `elsif SIM1 // Simulation 1
  initial begin
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_1/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
  end
  `elsif SIM2 // Simulation 2
  initial begin
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/12x12x7_2/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
  end
  `elsif SIM3 // Simulation 3
  initial begin
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/64x64x8_0/bram7.dat",
    BRAM_FIFO_blk[7].FIFO.RAM.RAM);
  end
  `elsif SIM4 // Simulation 4 (129x129x16_0) 
  initial begin
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram7.dat",
    BRAM_FIFO_blk[7].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram8.dat",
    BRAM_FIFO_blk[8].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram9.dat",
    BRAM_FIFO_blk[9].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram10.dat",
    BRAM_FIFO_blk[10].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram11.dat",
    BRAM_FIFO_blk[11].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram12.dat",
    BRAM_FIFO_blk[12].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram13.dat",
    BRAM_FIFO_blk[13].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram14.dat",
    BRAM_FIFO_blk[14].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_0/bram15.dat",
    BRAM_FIFO_blk[15].FIFO.RAM.RAM);
  end
  `elsif SIM5 // Simulation 5 (129x129x16_1)
  initial begin
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram7.dat",
    BRAM_FIFO_blk[7].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram8.dat",
    BRAM_FIFO_blk[8].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram9.dat",
    BRAM_FIFO_blk[9].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram10.dat",
    BRAM_FIFO_blk[10].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram11.dat",
    BRAM_FIFO_blk[11].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram12.dat",
    BRAM_FIFO_blk[12].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram13.dat",
    BRAM_FIFO_blk[13].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram14.dat",
    BRAM_FIFO_blk[14].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_1/bram15.dat",
    BRAM_FIFO_blk[15].FIFO.RAM.RAM);
  end
  `elsif SIM6 // Simulation 6 (129x129x16_2)
  initial begin
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram0.dat",
    BRAM_FIFO_blk[0].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram1.dat",
    BRAM_FIFO_blk[1].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram2.dat",
    BRAM_FIFO_blk[2].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram3.dat",
    BRAM_FIFO_blk[3].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram4.dat",
    BRAM_FIFO_blk[4].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram5.dat",
    BRAM_FIFO_blk[5].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram6.dat",
    BRAM_FIFO_blk[6].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram7.dat",
    BRAM_FIFO_blk[7].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram8.dat",
    BRAM_FIFO_blk[8].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram9.dat",
    BRAM_FIFO_blk[9].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram10.dat",
    BRAM_FIFO_blk[10].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram11.dat",
    BRAM_FIFO_blk[11].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram12.dat",
    BRAM_FIFO_blk[12].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram13.dat",
    BRAM_FIFO_blk[13].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram14.dat",
    BRAM_FIFO_blk[14].FIFO.RAM.RAM);
    $readmemb("../../data_set/hw_wff_graph/129x129x16_2/bram15.dat",
    BRAM_FIFO_blk[15].FIFO.RAM.RAM);
  end
  `else // nonSimulation
  initial begin
    //`include "./init.dat"
  end
  `endif
  
  // State Machine
  always @(posedge clk) begin
    if (n_rst) begin
      // reset
      state <= INIT_STATE;
    end
    else begin
      case(state)
        INIT_STATE : begin
          if (start) begin
            state <= LOAD_STATE;
          end
        end
        LOAD_STATE : begin
          state <= PROCESS_STATE;
        end
        PROCESS_STATE : begin
          if (done_cnt >= (CNT_LIMIT-1) && (node_cnt_manage == XY-1)) begin
            state <= FINISH_STATE;
          end
        end
        FINISH_STATE : begin
        end
        default : ;
      endcase
    end
  end

  always @(posedge clk) begin
    clk_cnt <= clk_cnt + 1;
    case (state)
      INIT_STATE : begin
        finish <= 0;
        done_cnt <= 0;
        cnt <= 0;
        for (j=0; j<X-9; j=j+1) begin
          node_cnt_shift_reg[j] <= 0;
        end
        node_cnt_0 <= 0;
        node_cnt_1 <= 0;
        node_cnt_2 <= 0;
        node_cnt_3 <= 0;
        node_cnt_4 <= 0;
        node_cnt_5 <= 0;
        node_cnt_6 <= 0;
        node_cnt_7 <= 0;
        node_cnt_8 <= 0;
        node_cnt_9 <= 0;
        node_cnt_manage <= 0;
        w_en <= 1'b0;

        process_en_s12    <= 0;
        process_en_s3     <= 0;
        process_en_s4     <= 0;
        process_en_s56    <= 0;
        process_en_s78    <= 0;
        process_en_s09    <= 0;
        process_en_manage <= 0;
      end
      PROCESS_STATE : begin
        //out_flag <= (w_smanage);
        if (node_cnt_manage == XY-1) begin
          done_cnt <= done_cnt + 1;
        end
        node_cnt_0 <= (node_cnt_0==XY-1) ? 0 : node_cnt_0 + 1;

        node_cnt_shift_reg[0] <= node_cnt_0;
        for (j=1; j<(X-9); j=j+1) begin
          node_cnt_shift_reg[j] <= node_cnt_shift_reg[j-1];
        end
        node_cnt_1 <= node_cnt_shift_reg[X-10];
        node_cnt_2 <= node_cnt_1;
        node_cnt_3 <= node_cnt_2;
        node_cnt_4 <= node_cnt_3;
        node_cnt_5 <= node_cnt_4;
        node_cnt_6 <= node_cnt_5;
        node_cnt_7 <= node_cnt_6;
        node_cnt_8 <= node_cnt_7;
        node_cnt_9 <= node_cnt_8;
        node_cnt_manage <= node_cnt_9;

        if ((node_cnt_manage==XY-1) && (done_cnt == CNT_LIMIT-1)) begin
          w_en <= 1'b0;
        end
        else if (node_cnt_8 == 1) begin
          w_en <= 1'b1;
        end
        if (node_cnt_0==(X-8)) begin
          process_en_s12 <= 1;
        end
        if (node_cnt_1%X==0 && node_cnt_2!=0) begin
          process_en_s12 <= 1;
        end
        if (node_cnt_1%X == X-1) begin
          process_en_s12 <= 0;
        end
        if (node_cnt_1 == 1) begin
          process_en_s3 <= 1;
        end
        if (node_cnt_2 == 1) begin
          process_en_s4 <= 1;
        end
        if (node_cnt_4%X == 0) begin
          process_en_s56 <= 0;
        end
        if (node_cnt_4%X == 1) begin
          process_en_s56 <= 1;
        end
        if (node_cnt_6%X==0) begin
          process_en_s78 <= 0;
        end
        if (node_cnt_6%X == 1) begin
          process_en_s78 <= 1;
        end
        if (node_cnt_7 == 1) begin
          process_en_s09 <= 1;
        end
        else if (node_cnt_0 == XY-1) begin
          process_en_s09 <= 0;
        end
        if (node_cnt_8 == 1) begin
          process_en_manage <= 1;
        end
      end
      FINISH_STATE : begin
        finish <= 1;
        cnt <= (cnt == NODE_NUM) ? cnt : cnt + 1;
      end
    endcase
  end

  reg [EX_WIDTH-1:0]  e_s0[Z-1:0],e_s1[Z-1:0],e_s2[Z-1:0],e_s3[Z-1:0],
                      e_s4[Z-1:0],e_s5[Z-1:0],e_s6[Z-1:0],e_s7[Z-1:0],
                      e_s8[Z-1:0],e_s9[Z-1:0],e_manage[Z-1:0];
                      

  reg [WV_WIDTH-1:0]  w_s0[Z-1:0],w_s1[Z-1:0],w_s2[Z-1:0],w_s3[Z-1:0],
                      w_s4[Z-1:0],w_s5[Z-1:0],w_s6[Z-1:0],w_s7[Z-1:0],
                      w_s8[Z-1:0],w_s9[Z-1:0],w_manage[Z-1:0];

  reg [WV_WIDTH-1:0]  w_next_s0[Z-1:0], w_next_s1[Z-1:0],
                      w_next_s2[Z-1:0], w_next_s3[Z-1:0],
                      w_next_s4[Z-1:0], w_next_s5[Z-1:0],
                      w_next_s6[Z-1:0], w_next_s7[Z-1:0],
                      w_next_s8[Z-1:0], w_next_s9[Z-1:0],
                      w_next_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_1_s0[Z-1:0],cf_1_s1[Z-1:0],
                      cf_1_s2[Z-1:0],cf_1_s3[Z-1:0],
                      cf_1_s4[Z-1:0],cf_1_s5[Z-1:0],
                      cf_1_s6[Z-1:0],cf_1_s7[Z-1:0],
                      cf_1_s8[Z-1:0],cf_1_s9[Z-1:0],
                      cf_1_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_2_s0[Z-1:0],cf_2_s1[Z-1:0],
                      cf_2_s2[Z-1:0],cf_2_s3[Z-1:0],
                      cf_2_s4[Z-1:0],cf_2_s5[Z-1:0],
                      cf_2_s6[Z-1:0],cf_2_s7[Z-1:0],
                      cf_2_s8[Z-1:0],cf_2_s9[Z-1:0],
                      cf_2_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_3_s0[Z-1:0],cf_3_s1[Z-1:0],
                      cf_3_s2[Z-1:0],cf_3_s3[Z-1:0],
                      cf_3_s4[Z-1:0],cf_3_s5[Z-1:0],
                      cf_3_s6[Z-1:0],cf_3_s7[Z-1:0],
                      cf_3_s8[Z-1:0],cf_3_s9[Z-1:0],
                      cf_3_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_5_s0[Z-1:0],cf_5_s1[Z-1:0],
                      cf_5_s2[Z-1:0],cf_5_s3[Z-1:0],
                      cf_5_s4[Z-1:0],cf_5_s5[Z-1:0],
                      cf_5_s6[Z-1:0],cf_5_s7[Z-1:0],
                      cf_5_s8[Z-1:0],cf_5_s9[Z-1:0],
                      cf_5_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_7_s0[Z-1:0],cf_7_s1[Z-1:0],
                      cf_7_s2[Z-1:0],cf_7_s3[Z-1:0],
                      cf_7_s4[Z-1:0],cf_7_s5[Z-1:0],
                      cf_7_s6[Z-1:0],cf_7_s7[Z-1:0],
                      cf_7_s8[Z-1:0],cf_7_s9[Z-1:0],
                      cf_7_manage[Z-1:0];

  reg [CF_WIDTH-1:0]  cf_9_s0[Z-1:0],cf_9_s1[Z-1:0],
                      cf_9_s2[Z-1:0],cf_9_s3[Z-1:0],
                      cf_9_s4[Z-1:0],cf_9_s5[Z-1:0],
                      cf_9_s6[Z-1:0],cf_9_s7[Z-1:0],
                      cf_9_s8[Z-1:0],cf_9_s9[Z-1:0],
                      cf_9_manage[Z-1:0];

  reg [ADDR_WIDTH-1:0] cnt_XY;
  assign out_flag = check_exf(e_manage[Z-1]);

  always @(posedge clk) begin
    if (n_rst) begin
      cnt_XY <= 0;
    end
    else begin
      cnt_XY <= (cnt_XY<XY-1) ? cnt_XY+1 : 0;
    end
  end

  // simulation sink node
  always @(posedge clk) begin
    if (state == PROCESS_STATE) begin
      //e_r_sink[node_cnt_0]     <= e_s0[Z-1];
      //e_r_sink[node_cnt_1]     <= e_s1[Z-1];
      //e_r_sink[node_cnt_2]     <= e_s2[Z-1];
      //e_r_sink[node_cnt_3]     <= e_s3[Z-1];
      //e_r_sink[node_cnt_4]     <= e_s4[Z-1];
      //e_r_sink[node_cnt_5]     <= e_s5[Z-1];
      //e_r_sink[node_cnt_6]     <= e_s6[Z-1];
      //e_r_sink[node_cnt_7]     <= e_s7[Z-1];
      //e_r_sink[node_cnt_8]     <= e_s8[Z-1];
      //e_r_sink[node_cnt_9]     <= e_s9[Z-1];
      e_r_sink[node_cnt_manage] <= e_manage[Z-1];
    end
  end


  // Wave_Front_Fetch Algorithm
  // Stage Shift register
  generate

  for (g=0; g<Z; g=g+1) begin : shift_reg_blk
    wire [ADDR_WIDTH-1:0] load_addr = 
    (node_cnt_0!=XY-1) ? node_cnt_0+1 : 0;

    reg [EX_WIDTH-1:0] e_shift_reg [(X-9)-1:0];
    reg [WV_WIDTH-1:0] w_shift_reg [(X-9)-1:0];
    reg [3:0]          cf_9_shift_reg [(X-9)-1:0];
    reg [CF_WIDTH-1:0] cf_7_shift_reg [(X-9)-1:0];
    reg [CF_WIDTH-1:0] cf_5_shift_reg [(X-9)-1:0];
    reg [CF_WIDTH-1:0] cf_3_shift_reg [(X-9)-1:0];
    reg [CF_WIDTH-1:0] cf_2_shift_reg [(X-9)-1:0];
    reg [3:0]          cf_1_shift_reg [(X-9)-1:0];
    reg [WV_WIDTH-1:0] w_next_shift_reg [(X-9)-1:0];

    localparam integer sim_addr = g*XY;

    always @(posedge clk) begin
      if (state == INIT_STATE) begin
        //w_next_s1[g] <= 0;
      end
      else if (state == LOAD_STATE) begin
        e_s0[g]      <= r_data[g][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH];
        w_s0[g]      <= r_data[g][DATA_WIDTH-EX_WIDTH-1:DATA_WIDTH-EX_WIDTH-WV_WIDTH];
        w_next_s0[g] <= r_data[g][DATA_WIDTH-EX_WIDTH-1:DATA_WIDTH-EX_WIDTH-WV_WIDTH];
        cf_9_s0[g]   <= {6'd0,r_data[g][(CF_WIDTH*4)+7:(CF_WIDTH*4)+4]};
        cf_7_s0[g]   <= r_data[g][(CF_WIDTH*4)+3:(CF_WIDTH*3)+4];
        cf_5_s0[g]   <= r_data[g][(CF_WIDTH*3)+3:(CF_WIDTH*2)+4];
        cf_3_s0[g]   <= r_data[g][(CF_WIDTH*2)+3:CF_WIDTH+4];
        cf_2_s0[g]   <= r_data[g][CF_WIDTH+3:4];
        cf_1_s0[g]   <= {6'd0,r_data[g][3:0]};
      end
      else if (state == PROCESS_STATE) begin

        //e_r_sim[node_cnt_0 + sim_addr]     <= e_s0[g];
        //e_r_sim[node_cnt_1 + sim_addr]     <= e_s1[g];
        //e_r_sim[node_cnt_2 + sim_addr]     <= e_s2[g];
        //e_r_sim[node_cnt_3 + sim_addr]     <= e_s3[g];
        //e_r_sim[node_cnt_4 + sim_addr]     <= e_s4[g];
        //e_r_sim[node_cnt_5 + sim_addr]     <= e_s5[g];
        //e_r_sim[node_cnt_6 + sim_addr]     <= e_s6[g];
        //e_r_sim[node_cnt_7 + sim_addr]     <= e_s7[g];
        //e_r_sim[node_cnt_8 + sim_addr]     <= e_s8[g];
        //e_r_sim[node_cnt_9 + sim_addr]     <= e_s9[g];
        //e_r_sim[node_cnt_manage + sim_addr] <= e_manage[g];

        //w_next_sim[node_cnt_0 + sim_addr]     <= w_next_s0[g];
        //w_next_sim[node_cnt_1 + sim_addr]     <= w_next_s1[g];
        //w_next_sim[node_cnt_2 + sim_addr]     <= w_next_s2[g];
        //w_next_sim[node_cnt_3 + sim_addr]     <= w_next_s3[g];
        //w_next_sim[node_cnt_4 + sim_addr]     <= w_next_s4[g];
        //w_next_sim[node_cnt_5 + sim_addr]     <= w_next_s5[g];
        //w_next_sim[node_cnt_6 + sim_addr]     <= w_next_s6[g];
        //w_next_sim[node_cnt_7 + sim_addr]     <= w_next_s7[g];
        //w_next_sim[node_cnt_8 + sim_addr]     <= w_next_s8[g];
        //w_next_sim[node_cnt_9 + sim_addr]     <= w_next_s9[g];
        //w_next_sim[node_cnt_manage + sim_addr] <= w_next_manage[g];

        //cf_1_sim[node_cnt_0 + sim_addr]     <= cf_1_s0[g];
        //cf_1_sim[node_cnt_1 + sim_addr]     <= cf_1_s1[g];
        //cf_1_sim[node_cnt_2 + sim_addr]     <= cf_1_s2[g];
        //cf_1_sim[node_cnt_3 + sim_addr]     <= cf_1_s3[g];
        //cf_1_sim[node_cnt_4 + sim_addr]     <= cf_1_s4[g];
        //cf_1_sim[node_cnt_5 + sim_addr]     <= cf_1_s5[g];
        //cf_1_sim[node_cnt_6 + sim_addr]     <= cf_1_s6[g];
        //cf_1_sim[node_cnt_7 + sim_addr]     <= cf_1_s7[g];
        //cf_1_sim[node_cnt_8 + sim_addr]     <= cf_1_s8[g];
        //cf_1_sim[node_cnt_9 + sim_addr]     <= cf_1_s9[g];
        //cf_1_sim[node_cnt_manage + sim_addr] <= cf_1_manage[g];

        //cf_2_sim[node_cnt_0 + sim_addr]     <= cf_2_s0[g];
        //cf_2_sim[node_cnt_1 + sim_addr]     <= cf_2_s1[g];
        //cf_2_sim[node_cnt_2 + sim_addr]     <= cf_2_s2[g];
        //cf_2_sim[node_cnt_3 + sim_addr]     <= cf_2_s3[g];
        //cf_2_sim[node_cnt_4 + sim_addr]     <= cf_2_s4[g];
        //cf_2_sim[node_cnt_5 + sim_addr]     <= cf_2_s5[g];
        //cf_2_sim[node_cnt_6 + sim_addr]     <= cf_2_s6[g];
        //cf_2_sim[node_cnt_7 + sim_addr]     <= cf_2_s7[g];
        //cf_2_sim[node_cnt_8 + sim_addr]     <= cf_2_s8[g];
        //cf_2_sim[node_cnt_9 + sim_addr]     <= cf_2_s9[g];
        //cf_2_sim[node_cnt_manage + sim_addr] <= cf_2_manage[g];

        //cf_3_sim[node_cnt_0 + sim_addr]     <= cf_3_s0[g];
        //cf_3_sim[node_cnt_1 + sim_addr]     <= cf_3_s1[g];
        //cf_3_sim[node_cnt_2 + sim_addr]     <= cf_3_s2[g];
        //cf_3_sim[node_cnt_3 + sim_addr]     <= cf_3_s3[g];
        //cf_3_sim[node_cnt_4 + sim_addr]     <= cf_3_s4[g];
        //cf_3_sim[node_cnt_5 + sim_addr]     <= cf_3_s5[g];
        //cf_3_sim[node_cnt_6 + sim_addr]     <= cf_3_s6[g];
        //cf_3_sim[node_cnt_7 + sim_addr]     <= cf_3_s7[g];
        //cf_3_sim[node_cnt_8 + sim_addr]     <= cf_3_s8[g];
        //cf_3_sim[node_cnt_9 + sim_addr]     <= cf_3_s9[g];
        //cf_3_sim[node_cnt_manage + sim_addr] <= cf_3_manage[g];

        //cf_5_sim[node_cnt_0 + sim_addr]     <= cf_5_s0[g];
        //cf_5_sim[node_cnt_1 + sim_addr]     <= cf_5_s1[g];
        //cf_5_sim[node_cnt_2 + sim_addr]     <= cf_5_s2[g];
        //cf_5_sim[node_cnt_3 + sim_addr]     <= cf_5_s3[g];
        //cf_5_sim[node_cnt_4 + sim_addr]     <= cf_5_s4[g];
        //cf_5_sim[node_cnt_5 + sim_addr]     <= cf_5_s5[g];
        //cf_5_sim[node_cnt_6 + sim_addr]     <= cf_5_s6[g];
        //cf_5_sim[node_cnt_7 + sim_addr]     <= cf_5_s7[g];
        //cf_5_sim[node_cnt_8 + sim_addr]     <= cf_5_s8[g];
        //cf_5_sim[node_cnt_9 + sim_addr]     <= cf_5_s9[g];
        //cf_5_sim[node_cnt_manage + sim_addr] <= cf_5_manage[g];

        //cf_7_sim[node_cnt_0 + sim_addr]     <= cf_7_s0[g];
        //cf_7_sim[node_cnt_1 + sim_addr]     <= cf_7_s1[g];
        //cf_7_sim[node_cnt_2 + sim_addr]     <= cf_7_s2[g];
        //cf_7_sim[node_cnt_3 + sim_addr]     <= cf_7_s3[g];
        //cf_7_sim[node_cnt_4 + sim_addr]     <= cf_7_s4[g];
        //cf_7_sim[node_cnt_5 + sim_addr]     <= cf_7_s5[g];
        //cf_7_sim[node_cnt_6 + sim_addr]     <= cf_7_s6[g];
        //cf_7_sim[node_cnt_7 + sim_addr]     <= cf_7_s7[g];
        //cf_7_sim[node_cnt_8 + sim_addr]     <= cf_7_s8[g];
        //cf_7_sim[node_cnt_9 + sim_addr]     <= cf_7_s9[g];
        //cf_7_sim[node_cnt_manage + sim_addr] <= cf_7_manage[g];

        //cf_9_sim[node_cnt_0 + sim_addr]     <= cf_9_s0[g];
        //cf_9_sim[node_cnt_1 + sim_addr]     <= cf_9_s1[g];
        //cf_9_sim[node_cnt_2 + sim_addr]     <= cf_9_s2[g];
        //cf_9_sim[node_cnt_3 + sim_addr]     <= cf_9_s3[g];
        //cf_9_sim[node_cnt_4 + sim_addr]     <= cf_9_s4[g];
        //cf_9_sim[node_cnt_5 + sim_addr]     <= cf_9_s5[g];
        //cf_9_sim[node_cnt_6 + sim_addr]     <= cf_9_s6[g];
        //cf_9_sim[node_cnt_7 + sim_addr]     <= cf_9_s7[g];
        //cf_9_sim[node_cnt_8 + sim_addr]     <= cf_9_s8[g];
        //cf_9_sim[node_cnt_9 + sim_addr]     <= cf_9_s9[g];
        //cf_9_sim[node_cnt_manage + sim_addr] <= cf_9_manage[g];

        e_s0[g]      <= r_data[g][DATA_WIDTH-1:DATA_WIDTH-EX_WIDTH];
        w_s0[g]      <= r_data[g][DATA_WIDTH-EX_WIDTH-1:DATA_WIDTH-EX_WIDTH-WV_WIDTH];
        w_next_s0[g] <= r_data[g][DATA_WIDTH-EX_WIDTH-1:DATA_WIDTH-EX_WIDTH-WV_WIDTH];
        cf_9_s0[g]   <= {6'd0,r_data[g][(CF_WIDTH*4)+7:(CF_WIDTH*4)+4]};
        cf_7_s0[g]   <= r_data[g][(CF_WIDTH*4)+3:(CF_WIDTH*3)+4];
        cf_5_s0[g]   <= r_data[g][(CF_WIDTH*3)+3:(CF_WIDTH*2)+4];
        cf_3_s0[g]   <= r_data[g][(CF_WIDTH*2)+3:CF_WIDTH+4];
        cf_2_s0[g]   <= r_data[g][CF_WIDTH+3:4];
        cf_1_s0[g]   <= {6'd0,r_data[g][3:0]};
        for (j=1; j<X-9; j=j+1) begin
          e_shift_reg[j]      <= e_shift_reg[j-1];
          w_shift_reg[j]      <= w_shift_reg[j-1];
          w_next_shift_reg[j] <= w_next_shift_reg[j-1];
          cf_9_shift_reg[j]   <= cf_9_shift_reg[j-1];
          cf_7_shift_reg[j]   <= cf_7_shift_reg[j-1];
          cf_5_shift_reg[j]   <= cf_5_shift_reg[j-1];
          cf_3_shift_reg[j]   <= cf_3_shift_reg[j-1];
          cf_2_shift_reg[j]   <= cf_2_shift_reg[j-1];
          cf_1_shift_reg[j]   <= cf_1_shift_reg[j-1];
        end
        e_s1[g]      <= e_shift_reg[X-10];
        w_s1[g]      <= w_shift_reg[X-10];
        w_next_s1[g] <= w_next_shift_reg[X-10];
        cf_9_s1[g]   <= cf_9_shift_reg[X-10];
        cf_7_s1[g]   <= cf_7_shift_reg[X-10];
        cf_5_s1[g]   <= cf_5_shift_reg[X-10];
        cf_3_s1[g]   <= cf_3_shift_reg[X-10];
        cf_2_s1[g]   <= cf_2_shift_reg[X-10];
        cf_1_s1[g]   <= cf_1_shift_reg[X-10];
      end
    end
  end : shift_reg_blk

  always @(posedge clk) begin
    if (state == PROCESS_STATE) begin
      //e_s2 <= e_s1;
      //e_s3 <= e_s2;
      //e_s4 <= e_s3;
      //e_s5 <= e_s4;
      //e_s6 <= e_s5;
      //e_s7 <= e_s6;
      //e_s8 <= e_s7;
      //e_s9 <= e_s8;
      //e_manage <= e_s9;
    end
  end

  // Stage 1 and 2
  for (g=0; g<EDGE_MAX_S1_2; g=g+1) begin : S1_2_Processing_blk
    //localparam integer a = search_addr(g,1,(X%2==0)); 
    //localparam integer b = a + 1;
    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s1[g]>w_s2[g])&(!check_exf(e_s2[g]))) begin
        // 1 <- 2 (1 pull 2)
        pull_flow <= min2(PENALTY2-cf_1_s2[g],e_s2[g]);
        wave_flag <= (e_s2[g] < PENALTY2-cf_1_s2[g]);
        ab_flag <= 1;
      end
      else if ((w_s2[g]>w_s1[g])&(!check_exf(e_s1[g]))) begin
        // 2 <- 1 (2 pull 1)
        pull_flow <= min2(cf_1_s2[g],e_s1[g]);
        wave_flag <= (e_s1[g] < cf_1_s2[g]);
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s1[g] > w_s2[g]) begin
          wave_flag <= ((PENALTY2-cf_1_s2[g])!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= (cf_1_s2[g]!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s12) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // 1 <- 2
              e_s2[g] <= e_s1[g] + pull_flow;
              e_s3[g] <= e_s2[g] - pull_flow;
              cf_1_s3[g] <= cf_1_s2[g] + pull_flow;
              w_next_s2[g] <= w_next_s1[g];
              if (wave_flag & (w_next_s2[g] < w_s1[g])) begin
                w_next_s3[g] <= w_s1[g];
              end
              else begin
                w_next_s3[g] <= w_next_s2[g];
              end
            end
            else if (ab_flag==2) begin // 2 <- 1
              e_s2[g] <= e_s1[g] - pull_flow;
              e_s3[g] <= e_s2[g] + pull_flow;
              cf_1_s3[g] <= cf_1_s2[g] - pull_flow;
              w_next_s3[g] <= w_next_s2[g];
              if (wave_flag & (w_next_s1[g] < w_s2[g])) begin
                w_next_s2[g] <= w_s2[g];
              end
              else begin
                w_next_s2[g] <= w_next_s1[g];
              end
            end
            else begin // cannot flow
              e_s2[g] <= e_s1[g];
              e_s3[g] <= e_s2[g];
              cf_1_s3[g] <= cf_1_s2[g];
              if (wave_flag) begin
                if ((!wave_ab_flag)) begin
                  w_next_s2[g] <= w_next_s1[g];
                  if (w_next_s2[g] < w_s1[g]) begin
                    w_next_s3[g] <= w_s1[g];
                  end
                  else begin
                    w_next_s3[g] <= w_next_s2[g];
                  end
                end
                else begin
                  w_next_s3[g] <= w_next_s2[g];
                  if (w_next_s1[g] < w_s2[g]) begin
                    w_next_s2[g] <= w_s2[g];
                  end
                  else begin
                    w_next_s2[g] <= w_next_s1[g];
                  end
                end
              end
              else begin
                w_next_s2[g] <= w_next_s1[g];
                w_next_s3[g] <= w_next_s2[g];
              end
            end
            w_s2[g] <= w_s1[g];
            cf_1_s2[g] <= cf_1_s1[g];
            cf_2_s2[g] <= cf_2_s1[g];
            cf_3_s2[g] <= cf_3_s1[g];
            cf_5_s2[g] <= cf_5_s1[g];
            cf_7_s2[g] <= cf_7_s1[g];
            cf_9_s2[g] <= cf_9_s1[g];

            w_s3[g] <= w_s2[g];
            cf_2_s3[g] <= cf_2_s2[g];
            cf_3_s3[g] <= cf_3_s2[g];
            cf_5_s3[g] <= cf_5_s2[g];
            cf_7_s3[g] <= cf_7_s2[g];
            cf_9_s3[g] <= cf_9_s2[g];
          end
        endcase
      end
      else if (state==PROCESS_STATE) begin
        e_s2[g] <= e_s1[g];
        w_s2[g] <= w_s1[g];
        w_next_s2[g] <= w_next_s1[g];
        cf_1_s2[g] <= cf_1_s1[g];
        cf_2_s2[g] <= cf_2_s1[g];
        cf_3_s2[g] <= cf_3_s1[g];
        cf_5_s2[g] <= cf_5_s1[g];
        cf_7_s2[g] <= cf_7_s1[g];
        cf_9_s2[g] <= cf_9_s1[g];

        e_s3[g] <= e_s2[g];
        w_s3[g] <= w_s2[g];
        w_next_s3[g] <= w_next_s2[g];
        cf_1_s3[g] <= cf_1_s2[g];
        cf_2_s3[g] <= cf_2_s2[g];
        cf_3_s3[g] <= cf_3_s2[g];
        cf_5_s3[g] <= cf_5_s2[g];
        cf_7_s3[g] <= cf_7_s2[g];
        cf_9_s3[g] <= cf_9_s2[g];
      end
    end
  end : S1_2_Processing_blk

  // Stage 3
  for (g=0; g<EDGE_MAX_S3; g=g+1) begin : S3_Processing_blk
    localparam integer a = 2*g; 
    localparam integer b = a + 1;

    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s3[a]>w_s3[b])&(!check_exf(e_s3[b]))) begin
        // g <- g+1 (g pull g+1)
        pull_flow <= min2(cf_3_s3[a],e_s3[b]);
        wave_flag <= (e_s3[b] < cf_3_s3[a]);
        ab_flag <= 1;
      end
      else if ((w_s3[b]>w_s3[a])&(!check_exf(e_s3[a]))) begin
        // g+1 <- g (g+1 pull g)
        pull_flow <= min2(cf_2_s3[b],e_s3[a]);
        wave_flag <= (e_s3[a] < cf_2_s3[b]);
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s3[a] < w_s3[b]) begin
          wave_flag <= (cf_2_s3[b]!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= (cf_3_s3[a]!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s3) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // a <- b
              e_s4[a] <= e_s3[a] + pull_flow;
              e_s4[b] <= e_s3[b] - pull_flow;
              cf_2_s4[b] <= cf_2_s3[b] + pull_flow;
              cf_3_s4[a] <= cf_3_s3[a] - pull_flow;
              w_next_s4[a] <= w_next_s3[a];
              if (wave_flag & (w_next_s3[b] < w_s3[a])) begin
                w_next_s4[b] <= w_s3[a];
              end
              else begin
                w_next_s4[b] <= w_next_s3[b];
              end
            end
            else if (ab_flag==2) begin // b <- a
              e_s4[a] <= e_s3[a] - pull_flow;
              e_s4[b] <= e_s3[b] + pull_flow;
              cf_2_s4[b] <= cf_2_s3[b] - pull_flow;
              cf_3_s4[a] <= cf_3_s3[a] + pull_flow;
              w_next_s4[b] <= w_next_s3[b];
              if (wave_flag & (w_next_s3[a] < w_s3[b])) begin
                w_next_s4[a] <= w_s3[b];
              end
              else begin
                w_next_s4[a] <= w_next_s3[a];
              end
            end
            else begin // cannot flow
              e_s4[a] <= e_s3[a];
              e_s4[b] <= e_s3[b];
              cf_2_s4[b] <= cf_2_s3[b];
              cf_3_s4[a] <= cf_3_s3[a];
              if (wave_flag) begin
                if (!wave_ab_flag) begin
                  w_next_s4[b] <= w_next_s3[b];
                  if (w_next_s3[a] < w_s3[b]) begin
                    w_next_s4[a] <= w_s3[b];
                  end
                  else begin
                    w_next_s4[a] <= w_next_s3[a];
                  end
                end
                else begin
                  w_next_s4[a] <= w_next_s3[a];
                  if (w_next_s3[b] < w_s3[a]) begin
                    w_next_s4[b] <= w_s3[a];
                  end
                  else begin
                    w_next_s4[b] <= w_next_s3[b];
                  end
                end
              end
              else begin
                w_next_s4[a] <= w_next_s3[a];
                w_next_s4[b] <= w_next_s3[b];
              end
            end
            cf_2_s4[a] <= cf_2_s3[a];
            cf_3_s4[b] <= cf_3_s3[b];

            w_s4[a] <= w_s3[a];
            cf_1_s4[a] <= cf_1_s3[a];
            cf_5_s4[a] <= cf_5_s3[a];
            cf_7_s4[a] <= cf_7_s3[a];
            cf_9_s4[a] <= cf_9_s3[a];

            w_s4[b] <= w_s3[b];
            cf_1_s4[b] <= cf_1_s3[b];
            cf_5_s4[b] <= cf_5_s3[b];
            cf_7_s4[b] <= cf_7_s3[b];
            cf_9_s4[b] <= cf_9_s3[b];
          end
        endcase
      end
    end
  end : S3_Processing_blk

  // no process node
  if (Z%2 == 1) begin 
    always @(posedge clk) begin // s3[Z-1]
      if (state == PROCESS_STATE) begin
        e_s4[Z-1] <= e_s3[Z-1];
        w_s4[Z-1] <= w_s3[Z-1];
        w_next_s4[Z-1] <= w_next_s3[Z-1];
        cf_1_s4[Z-1] <= cf_1_s3[Z-1];
        cf_2_s4[Z-1] <= cf_2_s3[Z-1];
        cf_3_s4[Z-1] <= cf_3_s3[Z-1];
        cf_5_s4[Z-1] <= cf_5_s3[Z-1];
        cf_7_s4[Z-1] <= cf_7_s3[Z-1];
        cf_9_s4[Z-1] <= cf_9_s3[Z-1];
      end
    end
  end

  // Stage 4
  for (g=0; g<EDGE_MAX_S4; g=g+1) begin : S4_Processing_blk
    localparam integer a = 2*g + 1; 
    localparam integer b = a + 1;

    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s4[a]>w_s4[b])&(!check_exf(e_s4[b]))) begin
        // g <- g+1 (g pull g+1)
        pull_flow <= min2(cf_3_s4[a],e_s4[b]);
        wave_flag <= (e_s4[b] < cf_3_s4[a]);
        ab_flag <= 1;
      end
      else if ((w_s4[b]>w_s4[a])&(!check_exf(e_s4[a]))) begin
        // g+1 <- g (g+1 pull g)
        pull_flow <= min2(cf_2_s4[b],e_s4[a]);
        wave_flag <= (e_s4[a] < cf_2_s4[b]);
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s4[a] < w_s4[b]) begin
          wave_flag <= (cf_2_s4[b]!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= (cf_3_s4[a]!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s4) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // g <- g+1
              e_s5[a] <= e_s4[a] + pull_flow;
              e_s5[b] <= e_s4[b] - pull_flow;
              cf_2_s5[b] <= cf_2_s4[b] + pull_flow;
              cf_3_s5[a] <= cf_3_s4[a] - pull_flow;
              w_next_s5[a]   <= w_next_s4[a];
              if (wave_flag & (w_next_s4[b] < w_s4[a])) begin
                w_next_s5[b] <= w_s4[a];
              end
              else begin
                w_next_s5[b] <= w_next_s4[b];
              end
            end
            else if (ab_flag==2) begin // g+1 <- g
              e_s5[a] <= e_s4[a] - pull_flow;
              e_s5[b] <= e_s4[b] + pull_flow;
              cf_2_s5[b] <= cf_2_s4[b] - pull_flow;
              cf_3_s5[a] <= cf_3_s4[a] + pull_flow;
              w_next_s5[b]   <= w_next_s4[b];
              if (wave_flag & (w_next_s4[a] < w_s4[b])) begin
                w_next_s5[a] <= w_s4[b];
              end
              else begin
                w_next_s5[a] <= w_next_s4[a];
              end
            end
            else begin // cannot flow
              e_s5[a] <= e_s4[a];
              e_s5[b] <= e_s4[b];
              cf_2_s5[b] <= cf_2_s4[b];
              cf_3_s5[a] <= cf_3_s4[a];
              if (wave_flag) begin 
                if (!wave_ab_flag) begin
                  w_next_s5[b] <= w_next_s4[b];
                  if (w_next_s4[a] < w_s4[b]) begin
                    w_next_s5[a] <= w_s4[b];
                  end
                  else begin
                    w_next_s5[a] <= w_next_s4[a];
                  end 
                end
                else begin
                  w_next_s5[a] <= w_next_s4[a];
                  if (w_next_s4[b] < w_s4[a]) begin
                    w_next_s5[b] <= w_s4[a];
                  end
                  else begin
                    w_next_s5[b] <= w_next_s4[b];
                  end
                end
              end
              else begin
                w_next_s5[a] <= w_next_s4[a];
                w_next_s5[b] <= w_next_s4[b];
              end
            end
            cf_2_s5[a] <= cf_2_s4[a];
            cf_3_s5[b] <= cf_3_s4[b];

            w_s5[a] <= w_s4[a];
            cf_1_s5[a] <= cf_1_s4[a];
            cf_5_s5[a] <= cf_5_s4[a];
            cf_7_s5[a] <= cf_7_s4[a];
            cf_9_s5[a] <= cf_9_s4[a];

            w_s5[b] <= w_s4[b];
            cf_1_s5[b] <= cf_1_s4[b];
            cf_5_s5[b] <= cf_5_s4[b];
            cf_7_s5[b] <= cf_7_s4[b];
            cf_9_s5[b] <= cf_9_s4[b];
          end
        endcase
      end
    end
  end : S4_Processing_blk

  // no process node
  always @(posedge clk) begin // s4[0]
    if (state == PROCESS_STATE) begin
      e_s5[0] <= e_s4[0];
      w_s5[0] <= w_s4[0];
      w_next_s5[0] <= w_next_s4[0];
      cf_1_s5[0] <= cf_1_s4[0];
      cf_2_s5[0] <= cf_2_s4[0];
      cf_3_s5[0] <= cf_3_s4[0];
      cf_5_s5[0] <= cf_5_s4[0];
      cf_7_s5[0] <= cf_7_s4[0];
      cf_9_s5[0] <= cf_9_s4[0];
    end
  end

  if (Z%2 == 0) begin
    always @(posedge clk) begin // s4[Z-1]
      if (state == PROCESS_STATE) begin
        e_s5[Z-1] <= e_s4[Z-1];
        w_s5[Z-1] <= w_s4[Z-1];
        w_next_s5[Z-1] <= w_next_s4[Z-1];
        cf_1_s5[Z-1] <= cf_1_s4[Z-1];
        cf_2_s5[Z-1] <= cf_2_s4[Z-1];
        cf_3_s5[Z-1] <= cf_3_s4[Z-1];
        cf_5_s5[Z-1] <= cf_5_s4[Z-1];
        cf_7_s5[Z-1] <= cf_7_s4[Z-1];
        cf_9_s5[Z-1] <= cf_9_s4[Z-1];
      end
    end
  end

  // Stage 5 and 6
  for (g=0; g<EDGE_MAX_S5_6; g=g+1) begin : S5_6_Processing_blk
    localparam integer a = g; 
    localparam integer b = a + 1;

    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s5[a]>w_s6[b])&(!check_exf(e_s6[b]))) begin
        // 5[a] <- 6[b]
        pull_flow <= min2(INHIBIT,e_s6[b]);
        wave_flag <= (e_s6[b] < INHIBIT);
        ab_flag <= 1;
      end
      else if ((w_s6[b]>w_s5[a])&(!check_exf(e_s5[a]))) begin
        // 6[b] <- 5[a]
        pull_flow <= min2(cf_5_s6[b],e_s5[a]);
        wave_flag <= (e_s5[a] < cf_5_s6[b]);
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s5[a] < w_s6[b]) begin
          wave_flag <= (cf_5_s6[b]!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= (INHIBIT!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s56) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // 5[a] <- 6[b]
              e_s6[a] <= e_s5[a] + pull_flow;
              e_s7[b] <= e_s6[b] - pull_flow;
              cf_5_s7[b] <= cf_5_s6[b] + pull_flow;
              w_next_s6[a] <= w_next_s5[a];
              if (wave_flag & (w_next_s6[b] < w_s5[a])) begin
                w_next_s7[b] <= w_s5[a];
              end
              else begin
                w_next_s7[b] <= w_next_s6[b];
              end
            end
            else if (ab_flag==2) begin // 6[b] <- 5[a]
              e_s6[a] <= e_s5[a] - pull_flow;
              e_s7[b] <= e_s6[b] + pull_flow;
              cf_5_s7[b] <= cf_5_s6[b] - pull_flow;
              w_next_s7[b] <= w_next_s6[b];
              if (wave_flag & (w_next_s5[a] < w_s6[b])) begin
                w_next_s6[a] <= w_s6[b];
              end
              else begin
                w_next_s6[a] <= w_next_s5[a];
              end
            end
            else begin // cannot flow
              e_s6[a] <= e_s5[a];
              e_s7[b] <= e_s6[b];
              cf_5_s7[b] <= cf_5_s6[b];
              if (wave_flag) begin
                if (!wave_ab_flag) begin
                  w_next_s7[b] <= w_next_s6[b];
                  if (w_next_s5[a] < w_s6[b]) begin
                    w_next_s6[a] <= w_s6[b];
                  end
                  else begin
                    w_next_s6[a] <= w_next_s5[a];
                  end
                end
                else begin
                  w_next_s6[a] <= w_next_s5[a];
                  if (w_next_s6[b] < w_s5[a]) begin
                    w_next_s7[b] <= w_s5[a];
                  end
                  else begin
                    w_next_s7[b] <= w_next_s6[b];
                  end
                end
              end
              else begin
                w_next_s6[a] <= w_next_s5[a];
                w_next_s7[b] <= w_next_s6[b];
              end
            end
            w_s6[a] <= w_s5[a];
            cf_9_s6[a] <= cf_9_s5[a];
            cf_7_s6[a] <= cf_7_s5[a];
            cf_5_s6[a] <= cf_5_s5[a];
            cf_3_s6[a] <= cf_3_s5[a];
            cf_2_s6[a] <= cf_2_s5[a];
            cf_1_s6[a] <= cf_1_s5[a];

            w_s7[b] <= w_s6[b];
            cf_9_s7[b] <= cf_9_s6[b];
            cf_7_s7[b] <= cf_7_s6[b];
            cf_3_s7[b] <= cf_3_s6[b];
            cf_2_s7[b] <= cf_2_s6[b];
            cf_1_s7[b] <= cf_1_s6[b];
          end
        endcase
      end
      else if (state==PROCESS_STATE) begin
        e_s6[a] <= e_s5[a];
        w_s6[a] <= w_s5[a];
        w_next_s6[a] <= w_next_s5[a];
        cf_1_s6[a] <= cf_1_s5[a];
        cf_2_s6[a] <= cf_2_s5[a];
        cf_3_s6[a] <= cf_3_s5[a];
        cf_5_s6[a] <= cf_5_s5[a];
        cf_7_s6[a] <= cf_7_s5[a];
        cf_9_s6[a] <= cf_9_s5[a];

        e_s7[b] <= e_s6[b];
        w_s7[b] <= w_s6[b];
        w_next_s7[b] <= w_next_s6[b];
        cf_1_s7[b] <= cf_1_s6[b];
        cf_2_s7[b] <= cf_2_s6[b];
        cf_3_s7[b] <= cf_3_s6[b];
        cf_5_s7[b] <= cf_5_s6[b];
        cf_7_s7[b] <= cf_7_s6[b];
        cf_9_s7[b] <= cf_9_s6[b];
      end
    end
  end : S5_6_Processing_blk

  // no process node
  always @(posedge clk) begin // s5[Z-1], s6[0]
    if (state == PROCESS_STATE) begin
      e_s6[Z-1] <= e_s5[Z-1];
      w_s6[Z-1] <= w_s5[Z-1];
      w_next_s6[Z-1] <= w_next_s5[Z-1];
      cf_1_s6[Z-1] <= cf_1_s5[Z-1];
      cf_2_s6[Z-1] <= cf_2_s5[Z-1];
      cf_3_s6[Z-1] <= cf_3_s5[Z-1];
      cf_5_s6[Z-1] <= cf_5_s5[Z-1];
      cf_7_s6[Z-1] <= cf_7_s5[Z-1];
      cf_9_s6[Z-1] <= cf_9_s5[Z-1];

      e_s7[0] <= e_s6[0];
      w_s7[0] <= w_s6[0];
      w_next_s7[0] <= w_next_s6[0];
      cf_1_s7[0] <= cf_1_s6[0];
      cf_2_s7[0] <= cf_2_s6[0];
      cf_3_s7[0] <= cf_3_s6[0];
      cf_5_s7[0] <= cf_5_s6[0];
      cf_7_s7[0] <= cf_7_s6[0];
      cf_9_s7[0] <= cf_9_s6[0];
    end
  end

  // Stage 7 and 8
  for (g=0; g<EDGE_MAX_S7_8; g=g+1) begin : S7_8_Processing_blk
    localparam integer a = g + 1; 
    localparam integer b = a - 1;

    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s7[a]>w_s8[b])&(!check_exf(e_s8[b]))) begin
        // 7[a] <- 8[b]
        pull_flow <= min2(cf_7_s7[a],e_s8[b]);
        wave_flag <= (e_s8[b] < cf_7_s7[a]);
        ab_flag <= 1;
      end
      else if ((w_s8[b]>w_s7[a])&(!check_exf(e_s7[a]))) begin
        // 8[b] <- 7[a]
        pull_flow <= min2(INHIBIT,e_s7[a]);
        wave_flag <= (e_s7[a] < INHIBIT);
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s7[a] < w_s8[b]) begin
          wave_flag <= (INHIBIT!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= (cf_7_s7[a]!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s78) begin
        case (state)
          INIT_STATE: begin
            //w_next_s8[a] <= 0;
            //w_next_s9[a] <= 0;
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // 7[a] <- 8[b]
              e_s8[a] <= e_s7[a] + pull_flow;
              e_s9[b] <= e_s8[b] - pull_flow;
              cf_7_s8[a] <= cf_7_s7[a] - pull_flow;
              w_next_s8[a] <= w_next_s7[a];
              if (wave_flag & (w_next_s8[b] < w_s7[a])) begin
                w_next_s9[b] <= w_s7[a];
              end
              else begin
                w_next_s9[b] <= w_next_s8[b];
              end
            end
            else if (ab_flag==2) begin // 8[b] <- 7[a]
              e_s8[a] <= e_s7[a] - pull_flow;
              e_s9[b] <= e_s8[b] + pull_flow;
              cf_7_s8[a] <= cf_7_s7[a] + pull_flow;
              w_next_s9[b] <= w_next_s8[b];
              if (wave_flag & (w_next_s7[a] < w_s8[b])) begin
                w_next_s8[a] <= w_s8[b];
              end
              else begin
                w_next_s8[a] <= w_next_s7[a];
              end
            end
            else begin // cannot flow
              e_s8[a] <= e_s7[a];
              e_s9[b] <= e_s8[b];
              cf_7_s8[a] <= cf_7_s7[a];
              if (wave_flag) begin
                if (!wave_ab_flag) begin
                  w_next_s9[b] <= w_next_s8[b];
                  if (w_next_s7[a] < w_s8[b]) begin
                    w_next_s8[a] <= w_s8[b];
                  end
                  else begin
                    w_next_s8[a] <= w_next_s7[a];
                  end
                end
                else begin
                  w_next_s8[a] <= w_next_s7[a];
                  if (w_next_s8[b] < w_s7[a]) begin
                    w_next_s9[b] <= w_s7[a];
                  end
                  else begin
                    w_next_s9[b] <= w_next_s8[b];
                  end
                end
              end
              else begin
                w_next_s8[a] <= w_next_s7[a];
                w_next_s9[b] <= w_next_s8[b];
              end
            end
            w_s8[a] <= w_s7[a];
            cf_1_s8[a] <= cf_1_s7[a];
            cf_2_s8[a] <= cf_2_s7[a];
            cf_3_s8[a] <= cf_3_s7[a];
            cf_5_s8[a] <= cf_5_s7[a];
            cf_9_s8[a] <= cf_9_s7[a];

            w_s9[b] <= w_s8[b];
            cf_1_s9[b] <= cf_1_s8[b];
            cf_2_s9[b] <= cf_2_s8[b];
            cf_3_s9[b] <= cf_3_s8[b];
            cf_5_s9[b] <= cf_5_s8[b];
            cf_7_s9[b] <= cf_7_s8[b];
            cf_9_s9[b] <= cf_9_s8[b];
          end
        endcase
      end
      else if (state==PROCESS_STATE) begin
        e_s8[a] <= e_s7[a];
        w_s8[a] <= w_s7[a];
        w_next_s8[a] <= w_next_s7[a];
        cf_1_s8[a] <= cf_1_s7[a];
        cf_2_s8[a] <= cf_2_s7[a];
        cf_3_s8[a] <= cf_3_s7[a];
        cf_5_s8[a] <= cf_5_s7[a];
        cf_7_s8[a] <= cf_7_s7[a];
        cf_9_s8[a] <= cf_9_s7[a];

        e_s9[b] <= e_s8[b];
        w_s9[b] <= w_s8[b];
        w_next_s9[b] <= w_next_s8[b];
        cf_1_s9[b] <= cf_1_s8[b];
        cf_2_s9[b] <= cf_2_s8[b];
        cf_3_s9[b] <= cf_3_s8[b];
        cf_5_s9[b] <= cf_5_s8[b];
        cf_7_s9[b] <= cf_7_s8[b];
        cf_9_s9[b] <= cf_9_s8[b];
      end
    end
  end : S7_8_Processing_blk

  // no process node
  always @(posedge clk) begin // s7[0], s8[Z-1]
    if (state == PROCESS_STATE) begin
      e_s8[0] <= e_s7[0];
      w_s8[0] <= w_s7[0];
      w_next_s8[0] <= w_next_s7[0];
      cf_1_s8[0] <= cf_1_s7[0];
      cf_2_s8[0] <= cf_2_s7[0];
      cf_3_s8[0] <= cf_3_s7[0];
      cf_5_s8[0] <= cf_5_s7[0];
      cf_7_s8[0] <= cf_7_s7[0];
      cf_9_s8[0] <= cf_9_s7[0];

      e_s9[Z-1] <= e_s8[Z-1];
      w_s9[Z-1] <= w_s8[Z-1];
      w_next_s9[Z-1] <= w_next_s8[Z-1];
      cf_1_s9[Z-1] <= cf_1_s8[Z-1];
      cf_2_s9[Z-1] <= cf_2_s8[Z-1];
      cf_3_s9[Z-1] <= cf_3_s8[Z-1];
      cf_5_s9[Z-1] <= cf_5_s8[Z-1];
      cf_7_s9[Z-1] <= cf_7_s8[Z-1];
      cf_9_s9[Z-1] <= cf_9_s8[Z-1];
    end
  end

  // Stage 0 and Stage 9 
  for (g=0; g<EDGE_MAX_S0_9; g=g+1) begin : S0_S9_Processing_blk
    logic [1:0] ab_flag; // 0:cannot flow, 1:a, 2:b
    logic [CF_WIDTH-1:0] pull_flow;
    logic wave_flag;
    logic wave_ab_flag;

    // FLOW calculation
    always_comb begin
      if ((w_s0[g]>w_s9[g])&(!check_exf(e_s9[g]))) begin
        // 0 <- 9
        pull_flow <= min2(PENALTY2-cf_9_s9[g],e_s9[g]);
        wave_flag <= (e_s9[g] < PENALTY2-cf_9_s9[g]);
        ab_flag <= 1;
      end
      else if ((w_s9[g]>w_s0[g])&(!check_exf(e_s0[g]))) begin
        // 9 <- a (9 pull a)
        pull_flow <= min2(cf_9_s9[g],e_s0[g]);
        wave_flag <= (e_s0[g] < (cf_9_s9[g]));
        ab_flag <= 2;
      end
      else begin
        // cannot flow
        pull_flow <= 0;
        if (w_s0[g] < w_s9[g]) begin
          wave_flag <= (cf_9_s9[g]!=0);
          wave_ab_flag <= 0;
        end
        else begin
          wave_flag <= ((PENALTY2-cf_9_s9[g])!=0);
          wave_ab_flag <= 1;
        end
        ab_flag <= 0;
      end
    end

    always @(posedge clk) begin
      if (process_en_s09) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            if (ab_flag==1) begin // 0 <- 9
              shift_reg_blk[g].e_shift_reg[0] <= e_s0[g] + pull_flow;
              e_manage[g] <= e_s9[g] - pull_flow;
              cf_9_manage[g] <= cf_9_s9[g] + pull_flow;
              shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
              if (wave_flag & (w_next_s9[g] < w_s0[g])) begin
                w_next_manage[g] <= w_s0[g];
              end
              else begin
                w_next_manage[g] <= w_next_s9[g];
              end
            end
            else if (ab_flag==2) begin // 9 <- 0
              shift_reg_blk[g].e_shift_reg[0] <= e_s0[g] - pull_flow;
              e_manage[g] <= e_s9[g] + pull_flow;
              cf_9_manage[g] <= cf_9_s9[g] - pull_flow;
              w_next_manage[g] <= w_next_s9[g];
              if (wave_flag & (w_next_s0[g] < w_s9[g])) begin
                shift_reg_blk[g].w_next_shift_reg[0] <= w_s9[g];
              end
              else begin
                shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
              end
            end
            else begin // cannot flow
              shift_reg_blk[g].e_shift_reg[0] <= e_s0[g];
              e_manage[g] <= e_s9[g];
              cf_9_manage[g] <= cf_9_s9[g];
              if (wave_flag) begin
                if ((!wave_ab_flag) & (w_next_s0[g]<w_s9[g])) begin
                  shift_reg_blk[g].w_next_shift_reg[0] <= w_s9[g];
                  w_next_manage[g] <= w_next_s9[g];
                end
                else if ((wave_ab_flag) & (w_s0[g]>w_next_s9[g])) begin
                  shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
                  w_next_manage[g] <= w_s0[g];
                end
                else begin
                  shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
                  w_next_manage[g] <= w_next_s9[g];
                end
              end
              else begin
                shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
                w_next_manage[g] <= w_next_s9[g];
              end
            end
            shift_reg_blk[g].w_shift_reg[0] <= w_s0[g];
            shift_reg_blk[g].cf_1_shift_reg[0] <= cf_1_s0[g];
            shift_reg_blk[g].cf_2_shift_reg[0] <= cf_2_s0[g];
            shift_reg_blk[g].cf_3_shift_reg[0] <= cf_3_s0[g];
            shift_reg_blk[g].cf_5_shift_reg[0] <= cf_5_s0[g];
            shift_reg_blk[g].cf_7_shift_reg[0] <= cf_7_s0[g];
            shift_reg_blk[g].cf_9_shift_reg[0] <= cf_9_s0[g];

            w_manage[g]    <= w_s9[g];
            cf_1_manage[g] <= cf_1_s9[g];
            cf_2_manage[g] <= cf_2_s9[g];
            cf_3_manage[g] <= cf_3_s9[g];
            cf_5_manage[g] <= cf_5_s9[g];
            cf_7_manage[g] <= cf_7_s9[g];
          end
        endcase
      end
      else if (state==PROCESS_STATE) begin
        shift_reg_blk[g].e_shift_reg[0] <= e_s0[g];
        shift_reg_blk[g].w_shift_reg[0] <= w_s0[g];
        shift_reg_blk[g].w_next_shift_reg[0] <= w_next_s0[g];
        shift_reg_blk[g].cf_1_shift_reg[0] <= cf_1_s0[g];
        shift_reg_blk[g].cf_2_shift_reg[0] <= cf_2_s0[g];
        shift_reg_blk[g].cf_3_shift_reg[0] <= cf_3_s0[g];
        shift_reg_blk[g].cf_5_shift_reg[0] <= cf_5_s0[g];
        shift_reg_blk[g].cf_7_shift_reg[0] <= cf_7_s0[g];
        shift_reg_blk[g].cf_9_shift_reg[0] <= cf_9_s0[g];

        e_manage[g] <= e_s9[g];
        w_manage[g] <= w_s9[g];
        w_next_manage[g] <= w_next_s9[g];
        cf_1_manage[g] <= cf_1_s9[g];
        cf_2_manage[g] <= cf_2_s9[g];
        cf_3_manage[g] <= cf_3_s9[g];
        cf_5_manage[g] <= cf_5_s9[g];
        cf_7_manage[g] <= cf_7_s9[g];
        cf_9_manage[g] <= cf_9_s9[g];
      end
    end
  end : S0_S9_Processing_blk

  // Stage Manage
  for (g=0; g<Z; g=g+1) begin : Manage_Processing_blk
    wire [ADDR_WIDTH-1:0] addr_g = node_cnt_manage;
    wire [WV_WIDTH-1:0] wave;
    wire [3:0] cf_9, cf_1;

    assign wave = check_exf(e_manage[g]) ? (w_manage[g]+1) : w_next_manage[g];
    assign cf_9 = cf_9_manage[g][3:0];
    assign cf_1 = cf_1_manage[g][3:0];
    assign w_data[g] = {
      e_manage[g],wave,
      cf_9,cf_7_manage[g],cf_5_manage[g],cf_3_manage[g],
      cf_2_manage[g],cf_1};

    always @(posedge clk) begin
      if (process_en_manage) begin
        case (state)
          INIT_STATE: begin
          end
          PROCESS_STATE: begin
            // update
            //if (!check_exf(e_manage[g])) begin // e_manage[g] >= 0
            //  wave_r[addr_g] <= w_next_manage[g];
            //end
            //else begin
            //  wave_r[addr_g] <= w_manage[g] + 1'b1; // e_manage[g] < 0
            //end
            //e_r[addr_g] <= e_manage[g];
            //cf_r1[addr_g] <= cf_1_manage[g];
            //cf_r2[addr_g] <= cf_2_manage[g];
            //cf_r3[addr_g] <= cf_3_manage[g];
            //cf_r5[addr_g] <= cf_5_manage[g];
            //cf_r7[addr_g] <= cf_7_manage[g];
            //cf_r9[addr_g] <= cf_9_manage[g];
          end
        endcase
      end
    end
  end : Manage_Processing_blk
  endgenerate

  // function
  
  // calculation e_r[g]
  function [EX_WIDTH-1:0] calc_e;
    input [EX_WIDTH-1:0] a;
    input [CF_WIDTH-1:0] b;
    input [CF_WIDTH-1:0] c;
    begin
      calc_e = a + b - c;
    end
  endfunction

  // check function
  function check_exf; // e < 0
    input signed [EX_WIDTH-1:0] a;
    begin
      check_exf = a[EX_WIDTH-1];
    end
  endfunction

  // search addr
  function [PLANE_WIDTH-1:0] search_addr;
    input [PLANE_WIDTH-1:0] a;
    input [PLACE_WIDTH-1:0] b;
    input even; // if Even or not
    begin
      case (b)
        1 : begin
          if (even) begin
            search_addr = a*2;
          end
          else begin
            search_addr = a*2 + (a/((X-1)/2));
          end
        end
        2 : begin
          if (even) begin
            search_addr = (a*2+1) + 2*(a/((X-2)/2));
          end
          else begin
            search_addr = (a*2+1) + (a/((X-1)/2));
          end
        end
        3 : begin
          search_addr = a + X*(a/X);
        end
        4 : begin
          search_addr = a + X + X*(a/X);
        end
        5 : begin
          search_addr = (a+1) + (X+1)*((a+1+(a/X))/X);
        end
        6 : begin
          search_addr = (a+1) + X + (X+1)*((a+1+(a/X))/X);
        end
        7 : begin
          search_addr = a + (X+1)*((a+1+(a/X))/X);
        end
        8 : begin
          search_addr = a + X + (X+1)*((a+1+(a/X))/X);
        end
        default : ;
      endcase
    end
  endfunction

  // search function
  function [ADDR_WIDTH-1:0] search_node;
    input [ADDR_WIDTH-1:0]  a;
    input [PLACE_WIDTH-1:0] b;
    begin
      case(b)
        0 : search_node = as_node({2'd0,a} - 1);
        1 : search_node = as_node({2'd0,a} + 1);
        2 : search_node = as_node({2'd0,a} - X);
        3 : search_node = as_node({2'd0,a} + X);
        4 : search_node = as_node({2'd0,a} + XY);
        5 : search_node = as_node({2'd0,a} - XY);
        6 : search_node = as_node({2'd0,a} + XY - 1);
        7 : search_node = as_node({2'd0,a} - XY + 1);
        8 : search_node = as_node({2'd0,a} + XY + 1);
        9 : search_node = as_node({2'd0,a} - XY - 1);
        default : search_node = -1;
      endcase
    end
  endfunction

  //search node_cal
  function [ADDR_WIDTH-1:0] as_node;
    input signed[ADDR_WIDTH+1:0] a;
    begin
      if (a < 0) begin
        as_node = NODE_NUM + a;
      end
      else begin
        if (a >= NODE_NUM) begin
          as_node = a - NODE_NUM;
        end
        else begin
          as_node = a;
        end
      end end
  endfunction

  //===================== log2 ========================//
  function integer log2;
    input integer value;
  begin
    value = value - 1;
    for (log2=0;value>0;log2=log2+1)
      value = value >> 1;
    end
  endfunction

  // --------------------- min2 --------------------- //
  function [CF_WIDTH-1:0] min2;
    input [CF_WIDTH-1:0] a;
    input [EX_WIDTH-1:0] b;
    begin
      min2 = (a < b) ? a : b;
    end
  endfunction
endmodule

`default_nettype wire
