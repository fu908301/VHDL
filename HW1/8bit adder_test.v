module testbench;

	reg [7:0] a;
	reg [7:0] b;
	reg  cin;

	wire [7:0] behave_sum;
	wire [7:0] dataflow_sum;
	wire behave_carry;
	wire dataflow_carry;

	adder_beh test_behave (
		.sum(behave_sum), 
		.carry(behave_carry), 
		.a(a), 
		.b(b),
		.cin(cin)
	);
	adder_dataflow test_dataflow(
		.sum(dataflow_sum), 
		.carry(dataflow_carry), 
		.a(a), 
		.b(b),
		.cin(cin)
	);

	
	initial begin
		a = 8'b01111111;
	  	b = 8'b01111111;
	 	cin = 0;
		#5
		a = $random;
		b = $random;
		cin = 0;
		#5
		a = $random;
		b = $random;
		cin = 0;
		#5
		a = $random;
		b = $random;
		cin = 0;
		#5
		a = $random;
		b = $random;
		cin = 0;
	end
endmodule