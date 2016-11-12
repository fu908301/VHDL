/////////////////////////////////////////////////////////////////
// GLOBAL PARAMETER DEFINITIONS
`define WORD_SIZE 32    // bits in data registers and address
`define HWORD_SIZE 16   // instruction word size
`define REG_FILE_SIZE 8 // number of general-purpose registers

// OPCODE DEFINITIONS
// Undefined Instruction Thumb Opcode
`define UNDEFINED_INSTRUCTION 16'hDE00 // 16 = `HWORD_SIZE
// Internal Encoding for Thumb Instructions Supported in "thumb.v"
`define ADC        6'b00_0000
`define ADD_1      6'b00_0001
`define ADD_2      6'b00_0010
`define ADD_3      6'b00_0011
`define ADD_5      6'b00_0100
`define ADD_6      6'b00_0101
`define ADD_7      6'b00_0110
`define AND        6'b00_0111
`define ASR_1      6'b00_1000
`define ASR_2      6'b00_1001
`define B_1        6'b00_1010
`define B_2        6'b00_1011
`define BIC        6'b00_1100
`define BL_BLX_H10 6'b00_1101
`define BL_BLX_H11 6'b00_1110
`define CMN        6'b00_1111
`define CMP_1      6'b01_0000
`define CMP_2      6'b01_0001
`define EOR        6'b01_0010
`define LDMIA      6'b01_0011
`define LDR_1      6'b01_0100
`define LDR_2      6'b01_0101
`define LDR_3      6'b01_0110
`define LDR_4      6'b01_0111
`define LDRB_1     6'b01_1000
`define LDRB_2     6'b01_1001
`define LDRH_1     6'b01_1010
`define LDRH_2     6'b01_1011
`define LDRSB      6'b01_1100
`define LDRSH      6'b01_1101
`define LSL_1      6'b01_1110
`define LSL_2      6'b01_1111
`define LSR_1      6'b10_0000
`define LSR_2      6'b10_0001
`define MOV_1      6'b10_0010
`define MUL        6'b10_0011
`define MVN        6'b10_0100
`define NEG        6'b10_0101
`define ORR        6'b10_0110
`define POP_R0     6'b10_0111
`define POP_R1     6'b10_1000
`define PUSH_R0    6'b10_1001
`define PUSH_R1    6'b10_1010
`define ROR        6'b10_1011
`define SBC        6'b10_1100
`define STMIA      6'b10_1101
`define STR_1      6'b10_1110
`define STR_2      6'b10_1111
`define STR_3      6'b11_0000
`define STRB_1     6'b11_0001
`define STRB_2     6'b11_0010
`define STRH_1     6'b11_0011
`define STRH_2     6'b11_0100
`define SUB_1      6'b11_0101
`define SUB_2      6'b11_0110
`define SUB_3      6'b11_0111
`define SUB_4      6'b11_1000
`define SWI        6'b11_1001
`define TST        6'b11_1010
`define UNDEF      6'b11_1111
/////////////////////////////////////////////////////////////////
