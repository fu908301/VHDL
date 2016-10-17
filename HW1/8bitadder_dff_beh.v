module adder_beh_dff(sum,carry,a,b,cin,clock);
	output [7:0] sum;
	output carry;
	input [7:0] a;
	input [7:0] b;
	input cin;
	input clock;
	wire [7:0] sum_temp;
	wire carry_temp;
	adder_beh adder(sum_temp,carry_temp,a,b,cin);
	D_FF dff0(sum[0],sum_temp[0],clock),
		dff1(sum[1],sum_temp[1],clock),
		dff2(sum[2],sum_temp[2],clock),
		dff3(sum[3],sum_temp[3],clock),
		dff4(sum[4],sum_temp[4],clock),
		dff5(sum[5],sum_temp[5],clock),
		dff6(sum[6],sum_temp[6],clock),
		dff7(sum[7],sum_temp[7],clock),
		dff8(carry,carry_temp,clock);
endmodule