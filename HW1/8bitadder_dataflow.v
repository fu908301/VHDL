module adder_dataflow(sum,carry,a,b,cin);
	input [7:0] a,b;
	input cin;
	output [7:0] sum;
	output carry;
	assign {carry,sum}=a+b+cin;
endmodule
