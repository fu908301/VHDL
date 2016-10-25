module pipeline(A,B,C,D,clk,rst);
	input [7:0] A;
	input [7:0] B;
	input [7:0] C;
	input clk;
	input rst;
	output [16:0] D;
	reg [16:0] Reg1;
	reg [16:0] Reg2;
	wire [8:0] Add;
	wire [16:0] Cross;
	assign Add = A + B;
	assign Cross = Reg1[8:0] * Reg1[16:9];
	assign D[16:0] = Reg2[16:0];
	always@(posedge clk or negedge rst)
	begin
	if(!rst)
		begin
			Reg1[0] <=  32'd0;
			Reg1[1] <=  32'd0;
			Reg1[2] <=  32'd0;
			Reg1[3] <=  32'd0;
			Reg1[4] <=  32'd0;
			Reg1[5] <=  32'd0;
			Reg1[6] <=  32'd0;
			Reg1[7] <=  32'd0;
			Reg1[8] <=  32'd0;
			Reg1[9] <=  32'd0;
			Reg1[10] <=  32'd0;
			Reg1[11] <=  32'd0;
			Reg1[12] <=  32'd0;
			Reg1[13] <=  32'd0;
			Reg1[14] <=  32'd0;
			Reg1[15] <=  32'd0;
			Reg1[16] <=  32'd0;

		end
	else
		begin
			Reg1[8:0] <= Add;
			 Reg1[16:9] <= C;		
		end
	end
	always@(posedge clk or negedge rst)
	begin
	if(!rst)
		begin
			Reg2[0] <=  32'd0;
			Reg2[1] <=  32'd0;
			Reg2[2] <=  32'd0;
			Reg2[3] <=  32'd0;
			Reg2[4] <=  32'd0;
			Reg2[5] <=  32'd0;
			Reg2[6] <=  32'd0;
			Reg2[7] <=  32'd0;
			Reg2[8] <=  32'd0;
			Reg2[9] <=  32'd0;
			Reg2[10] <=  32'd0;
			Reg2[11] <=  32'd0;
			Reg2[12] <=  32'd0;
			Reg2[13] <=  32'd0;
			Reg2[14] <=  32'd0;
			Reg2[15] <=  32'd0;
			Reg2[16] <=  32'd0;

		end
	else 
		begin
			Reg2[16:0] <= Cross;
		end
	end

endmodule

