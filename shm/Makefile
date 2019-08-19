#Vivado 2017.2
Xilinx = /home/cad/xilinx-vivado-2017.2/Vivado/2017.2/data
SHM_FILE = ./pipelined_wff_sim.v
V_FILE = \
../pipelined_wff.sv\
../BRAM_FIFO.sv\
../BRAM.sv\

#SIM_NUM = SIM

OPT = ${Xilinx}/verilog/src/glbl.v \
-y ${Xilinx}/verilog/src/unisims +libext+.v \
-y ${Xilinx}/verilog/src/unimacro +libext+.v \
+access+r+w \
#+define+${SIM_NUM}

.PHONY: all clren sim

all: sim

sim:
	xmverilog ${OPT} ${SHM_FILE} ${V_FILE}

clean:
	rm -rf  INCA_libs verilog.dump dump.txt *~ */*~ */\#*

distclean: clean
	rm -rf wff.shm

