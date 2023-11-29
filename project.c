/* MySPIM Simulator

Project Members:
- Michael Akinyemi

Setup:
1. gcc -o spimcore spimcore.c project.c
2. ./spimcore [input file].asc

Important Notes:
- 32-bit instructions
- Can use bitwise AND + shifting for easy partitioning of instructions
- Don't convert between HEX <-> DEC
- No structural hazards in MIPS
- Binary data is being stored as integers rather than char arrays
*/

#include "spimcore.h"

/* ALU (10 pts) */
void ALU(unsigned A, unsigned B, char ALUControl, unsigned *ALUResult, char *Zero) {

  switch(ALUControl) {
    case 0x0: // Addition (Or "Don't Care") - 000
      *ALUResult = A + B;
      break;

    case 0x1: // Subtraction - 001
      *ALUResult = A - B;
      break;

    case 0x2: // Set Less Than (Signed) - 010
      if (A < B) *ALUResult = 1;
      else *ALUResult = 0;
      break;

    case 0x3: // Set Less Than (Unsigned) - 011
      if (A < B) *ALUResult = 1;
      else *ALUResult = 0;
      break;

    case 0x4: // AND - 100
      if (A && B) *ALUResult = 1;
      else *ALUResult = 0;
      break;

    case 0x5: // OR - 101
      if (A || B) *ALUResult = 1;
      else *ALUResult = 0;
      break;

    case 0x6: // Shift B 16 bits - 110
      *ALUResult = B << 16;
      break;

    case 0x7: // NOT A - 111
      *ALUResult = ~A;
  }
}

/* Instruction Fetch (10 pts) */
int instruction_fetch(unsigned PC, unsigned *Mem, unsigned *instruction) {


}

/* Instruction Partition (10 pts) */
void instruction_partition(unsigned instruction, unsigned *op, unsigned *r1,unsigned *r2, unsigned *r3, unsigned *funct, unsigned *offset, unsigned *jsec) {
  // Isolate (partition) bit regions using bit shifting + bitwise AND operator with 32-bit HEX values
  // Shift is necessary for first four to move regions to LSB

	// *op     = (instruction & 0xFC000000) >> 26; // XXXXXX__ ________ ________ ________ [31-26]
	// *r1     = (instruction & 0x03E00000) >> 21; // ______XX XXX_____ ________ ________ [25-21] (=rs)
	// *r2     = (instruction & 0x001F0000) >> 16; // ________ ___XXXXX ________ ________ [20-16] (=rt)
	// *r3     = (instruction & 0x0000F800) >> 11; // ________ ________ XXXXX___ ________ [15-11] (=rd)
	// *funct  =  instruction & 0x0000003F; 		   // ________ ________ ________ __XXXXXX [5-0]
	// *offset =  instruction & 0x0000FFFF; 		   // ________ ________ XXXXXXXX XXXXXXXX [15-0]
	// *jsec   =  instruction & 0x03FFFFFF; 		   // ______XX XXXXXXXX XXXXXXXX XXXXXXXX [25-0]

  // HEX values represent binary values to retain for partition
  *op = (instruction >> 26) & 0x0000003f;
  *r1 = (instruction >> 21) & 0x1f;
  *r2 = (instruction >> 16) & 0x1f;
  *r3 = (instruction >> 11) & 0x1f;
  *funct = instruction & 0x0000003f;
  *offset = instruction & 0x0000ffff;
  *jsec = instruction & 0x03ffffff;

}

/* Instruction Decode (15 pts) */
int instruction_decode(unsigned op, struct_controls *controls) {
  // Configure the control signals based on the provided opcode ()
  switch(op) {
    /* 
       Control Signal Structure:
        (0) RegDst (1) Jump (2) Branch (3) MemRead (4) MemtoReg
        (5) ALUOp (6) MemWrite (7) ALUSrc (8) RegWrite

        TODO
    */

    case 0x0: // Arithmetic Logic Unit (add, sub, slt, sltu, AND, OR)
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x1: // Jump
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

		case 0x2:  // Branch on Equal
			*controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
			break;

    case 0x3: // Add Immediate
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x4: // Set Less Than Immediate (Signed)
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x5: // Set Less Than Immediate (Unsigned)
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x6: // lui
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x7: // Load Word
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    case 0x8: // Store Word
      *controls = (struct_controls) {0,0,0,0,0,0,0,0,0};
      break;

    default:
      return 1; // No op found (7, 111, or unknown)
  }

  return 0;


}

/* Read Register (5 pts) */
void read_register(unsigned r1, unsigned r2, unsigned *Reg, unsigned *data1, unsigned *data2) {
  *data1 = Reg[r1];
  *data2 = Reg[r2];
}


/* Sign Extend (10 pts) */
void sign_extend(unsigned offset, unsigned *extended_value) {


}

/* ALU operations (10 pts) */
int ALU_operations(unsigned data1, unsigned data2, unsigned extended_value, unsigned funct, char ALUOp, char ALUSrc, unsigned *ALUresult, char *Zero) {
  unsigned char ALUControl = ALUOp;

   // Handle function if using R-Type instruction (111)
   
   // TODO
  if (ALUControl == 0x7) {
    switch(funct) {
      case 0x1: // Addition
        ALUControl = 0x0;
        break;

      case 0x2: // Subtraction
        ALUControl = 0x0;
        break;

      case 0x3: // AND
        ALUControl = 0x0;
        break;
        
      case 0x4: // OR
        ALUControl = 0x0;
        break;
        
      case 0x5: // Set Less Than (Signed)
        ALUControl = 0x0;
        break;
        
      case 0x6: // Set Less Than (Unigned)
        ALUControl = 0x0;
        break;
      
      default: // Halt program if unknown command is provided 
        return 1; 
    }
  }

  // Call ALU function using parsed ALUControl value
  ALU(data1, data2, ALUControl, ALUresult, Zero);

}

/* Read/Write Memory (10 pts) */
int rw_memory(unsigned ALUResult, unsigned data2, char MemWrite, char MemRead, unsigned *memdata, unsigned *Mem) {

}


/* Write Register (10 pts) */
void write_register(unsigned r2, unsigned r3, unsigned memdata, unsigned ALUResult, char RegWrite, char RegDst, char MemtoReg, unsigned *Reg) {

}

/* PC update (10 pts) */
void PC_update(unsigned jsec, unsigned extended_value, char Branch, char Jump, char Zero, unsigned *PC) {

  PC += 4;
  

}
