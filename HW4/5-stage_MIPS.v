module CPU (clock); 
	parameter LW = 6'b100011, SW = 6'b101011, BEQ = 6'b000100, no-op = 32'b00000_100000, ALUop = 6'b0; 
	input clock;
	reg[31:0] PC, Regs[0:31], IMemory[0:1023], DMemory[0:1023], IFIDIR, IDEXA, IDEXB, IDEXIR, EXMEMIR, EXMEMB,  EXMEMALUOut, MEMWBValue, MEMWBIR; 
	wire [4:0] IDEXrs, IDEXrt, EXMEMrd, MEMWBrd, MEMWBrt; 
	wire [5:0] EXMEMop, MEMWBop, IDEXop; Hold opcodes; 
	wire [31:0] Ain, Bin; 
	wire bypassAfromMEM, bypassAfromALUinWB,bypassBfromMEM, bypassBfromALUinWB,        bypassAfromLWinWB, bypassBfromLWinWB;
	assign IDEXrs = IDEXIR[25:21]; 
	assign IDEXrt = IDEXIR[15:11]; 
	assign EXMEMrd = EXMEMIR[15:11]; 
	assign MEMWBrd = MEMWBIR[20:16]; 
	assign EXMEMop = EXMEMIR[31:26]; 
	assign MEMWBrt = MEMWBIR[25:20]; 
	assign MEMWBop = MEMWBIR[31:26]; 
	assign IDEXop = IDEXIR[31:26];
	assign bypassAfromMEM = (IDEXrs == EXMEMrd) & (IDEXrs!=0) & (EXMEMop==ALUop); 
	assign bypassBfromMEM = (IDEXrt == EXMEMrd)&(IDEXrt!=0) & (EXMEMop==ALUop);
	assign bypassAfromALUinWB =( IDEXrs == MEMWBrd) & (IDEXrs!=0) & (MEMWBop==ALUop); 
	 assign bypassBfromALUinWB = (IDEXrt == MEMWBrd) & (IDEXrt!=0) & (MEMWBop==ALUop);
	assign bypassAfromLWinWB =( IDEXrs == MEMWBIR[20:16]) & (IDEXrs!=0) & (MEMWBop==LW);
	assign bypassBfromLWinWB = (IDEXrt == MEMWBIR[20:16]) & (IDEXrt!=0) & (MEMWBop==LW); 
	 assign Ain = bypassAfromMEM? EXMEMALUOut : (bypassAfromALUinWB | bypassAfromLWinWB)? MEMWBValue : IDEXA;
	assign Bin = bypassBfromMEM? EXMEMALUOut : (bypassBfromALUinWB | bypassBfromLWinWB)? MEMWBValue: IDEXB; 
	reg [5:0] i; 
	initial begin
		PC = 0;
		IFIDIR = no-op; IDEXIR = no-op; EXMEMIR = no-op; MEMWBIR = no-op;
		for (i = 0;i<=31;i = i+1) 
			Regs[i] = i;
	end
	always @ (posedge clock) begin
		IFIDIR <= IMemory[PC>>2];
		PC <= PC + 4;
	 end 
	 IDEXA <= Regs[IFIDIR[25:21]]; IDEXB <= Regs[IFIDIR[20:16]];
	 IDEXIR <= IFIDIR;
	 if ((IDEXop==LW) | (IDEXop==SW))
	EXMEMALUOut <= IDEXA +{{16{IDEXIR[15]}}, IDEXIR[15:0]};
	else if (IDEXop==ALUop) 
		case (IDEXIR[5:0])
			32: EXMEMALUOut <= Ain + Bin; 
		 default:
			 32: EXMEMALUOut <= Ain - Bin;
	endcase
	EXMEMIR <= IDEXIR; EXMEMB <= IDEXB;
	 if (EXMEMop==ALUop) 
		MEMWBValue <= EXMEMALUOut;
	 else if (EXMEMop == LW)
		MEMWBValue <= DMemory[EXMEMALUOut>>2];
 	else if  (EXMEMop == SW)
		 DMemory[EXMEMALUOut>>2] <=EXMEMB;
	 MEMWBIR <= EXMEMIR;
	if ((MEMWBop==ALUop) & (MEMWBrd != 0)) 
		Regs[MEMWBrd] <= MEMWBValue;
	else if ((EXMEMop == LW)& (MEMWBrt != 0)) 
		Regs[MEMWBrt] <= MEMWBValue; 
	end
endmodule
		  