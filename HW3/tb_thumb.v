/////////////////////////////////////////////////////////////////
// MODULE: Test bench for pipelined Thumb microproc: tb_thumb.v
// Author: Sunggu Lee
// Created: August 15, 2003
// Last Modified: September 13, 2003
// Description: Tests the module "thumb.v".

// DEFINITIONS
`timescale 1ns/1ns
`define PERIOD1 100      // assume system clock cycle of 10MHz
`define READ_DELAY 80    // delay before memory data is ready
`define WRITE_DELAY 80   // delay in writing to memory
`define STABLE_TIME 10   // time data is stable after end-of-read
`define MEMORY_SIZE 256  // size of reduced memory is 2^8 words
                         // - use only 8 lowest bits of address
`include "thumb_defs.vh" // include common defs

// MODULE DEFINITION
module tb_thumb();

  // SIGNAL DECLARATIONS for chip inputs and outputs
  wire read_instruction_n;  // control read from instruction mem
  wire [`WORD_SIZE-1:0] instruction_address; // address of instr
  wire [`HWORD_SIZE-1:0] instruction; // current instruction
  wire read_data_n;            // control read from data memory
  wire write_data_n;           // control write to data memory
  wire [`WORD_SIZE-1:0] data_address; // address of data
  wire [`WORD_SIZE-1:0] data;         // current data
  reg reset_n;    // active-low RESET signal
  reg clk;        // clock signal

  // SIGNAL DECLARATIONS for signals being used internally
  reg [`WORD_SIZE-1:0] output_instruction; // instr. memory outp.
  reg [`WORD_SIZE-1:0] output_data;        // data memory output
  reg [`WORD_SIZE-1:0] write_data;         // data to be written

  // instantiate the unit under test
  thumb UUT (read_instruction_n, instruction_address, instruction,
            read_data_n, write_data_n, data_address, data,
            reset_n, clk);
  
  // initialize inputs
  initial begin
    clk = 0;           // set initial clock value

    reset_n = 1;       // generate a LOW pulse for reset_n
    #(`PERIOD1/4) reset_n = 0;
    #(`PERIOD1 * 2) reset_n = 1;
  end

  // generate the clock
  always #(`PERIOD1/2)clk = ~clk; // period = `PERIOD1

  // model the instruction and data memory devices
  reg [`HWORD_SIZE-1:0] instruction_memory [0:`MEMORY_SIZE-1];
  reg [`WORD_SIZE-1:0] data_memory [0:`MEMORY_SIZE-1];

  // model the read process for the instruction memory device
  assign instruction = read_instruction_n ? 'bz
                                          : output_instruction;
  always begin
    if (read_instruction_n == 0) begin
      #`READ_DELAY;  // assume no spurious address changes
      output_instruction =
        instruction_memory[instruction_address[7:0]];
      wait ((read_instruction_n == 1) ||
            (output_instruction !=
             instruction_memory[instruction_address[7:0]]));
    end
    else begin  // end of read
      #`STABLE_TIME;
      output_instruction = `WORD_SIZE'bz;
      wait (read_instruction_n == 0);
    end
  end  // of always block for instruction memory read

  // model the read process for the data memory device
  assign data = read_data_n ? `WORD_SIZE'bz : output_data;
  always begin
    if (read_data_n == 0) begin
      #`READ_DELAY;  // assume no spurious address changes
      output_data = data_memory[data_address[7:0]];
      wait ((read_data_n == 1) ||
            (output_data != data_memory[data_address[7:0]]));
    end
    else begin  // end of read
      #`STABLE_TIME;
      output_data = `WORD_SIZE'bz;
      wait (read_data_n == 0);
    end
  end  // of always block for data memory read

  // model the write process for the data memory device
  always begin
    wait (write_data_n == 0);
    write_data = data;
    wait ((write_data_n == 1) || (data != write_data));
    if (write_data_n == 1) begin  // wait for write enable = '1'
      #`WRITE_DELAY;
      data_memory[data_address[7:0]] = write_data;
    end
    else  // data != write_data (data has changed)
      write_data = data;
  end  // of always block for data memory write
  
  // store programs and data in the instruction and memories
  initial begin
    instruction_memory[0] = 16'h2100; // MOV r1, #0 (R[1] <- 0)
    instruction_memory[2] = 16'h2200; // MOV r2, #0 (R[2] <- 0)
    instruction_memory[4] = 16'h20fc; // MOV r0, #fc (R[0] <- fc)
    instruction_memory[6] = 16'h2909; // CMP r1, #9 (R[1] == 9)?
    instruction_memory[8] = 16'hda04; // BGE 0x14(if >=, goto 20)
    instruction_memory[10] = 16'he001;// B   0x10(goto 0x10 = 16)
    instruction_memory[12] = 16'h3101;// ADD r1, #1 (R[1] += 1)
    instruction_memory[14] = 16'he7fa;// B   0x6  (goto 0x4 = 6)
    instruction_memory[16] = 16'h1852;// ADD r2, r2, r1
    instruction_memory[18] = 16'he7fb;// B   0xc  (goto 0xc = 12)
    instruction_memory[20] = 16'h6042;// STR R2,R0+1*4(data:0x24)
    instruction_memory[22] = 16'hdf00;// SWI(to halt the program)
    // instruction_memory[22] = 16'h4770;// BX  r14  (goto R[14])
    // last instruction in original assembly is a return to main
  end
  
  // test output of program to verify proper operation of circuit
  initial begin
    #15700; // this is the time when the program output is avail.
    if ( (data_address === 32'h00000100) &&
         (data === 32'h00000024) )
      $display("Data address (0x100) and data (0x24) correct.");
    else
      $display("ERROR: data address = %0x and data = %0x.",
               data_address, data);
    #500;  $display("Simulation completed at time %0t.", $time);
    $finish;  // terminate simulation after 16.2 microseconds
  end
  
endmodule
/////////////////////////////////////////////////////////////////
