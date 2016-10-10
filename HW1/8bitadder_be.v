module adder_beh(sum,carry,a,b,cin);
	input [7:0] a,b;
	input cin;
	output [7:0] sum; reg[7:0] sum;
	output carry; reg carry;
	always@(a,b,cin)
	begin
		{carry,sum}=a+b+cin;
	end
endmodule