`timescale 1ns/1ps
module testbench;
	

	reg [7:0] A;
	reg [7:0] B;
	reg [7:0] C;
	reg clk;
	reg rst;
	
	wire [15:0] D;

	pipeline test
	(
		.A(A),
		.B(B),
		.C(C),
		.D(D),
		.clk(clk),
		.rst(rst)
	);
	initial begin
  		$sdf_annotate("file_name.sdf",test);
	end	
	initial begin
		clk = 1'b0;
		forever #10 clk = ~clk;
	end

	initial begin
		rst = 0;
		#10
		rst = 1;
		A = 8'b00111111;
	  	B = 8'b00111111;
		C = 8'b00000011;
		#200
		rst = 1;
		A = $random;
	  	B = $random;
		C = $random;
		#200
		rst = 1;
		A = $random;
	  	B = $random;
		C = $random;
		#200
		rst = 1;
		A = $random;
	  	B = $random;
		C = $random;
		#200
		rst = 1;
		A = $random;
	  	B = $random;
		C = $random;
		#200
		rst = 1;
		A = $random;
	  	B = $random;
		C = $random;
		$stop;
	end
endmodule
