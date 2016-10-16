module testbench_dff;

	reg [7:0] a;
	reg [7:0] b;
	reg cin;
	reg clock;

	wire [7:0] dataflow_sum;
	wire [7:0] beh_sum;
	wire [7:0] struct_sum;
	wire dataflow_carry;
	wire beh_carry;
	wire struct_carry;
always
	begin
	clock = 0; #5;
	clock = 1; #5;
	end
	
adder_dataflow_dff test_dataflow_dff(
	.sum(dataflow_sum),
	.carry(dataflow_carry),
	.a(a),
	.b(b),
	.cin(cin),
	.clock(clock)
);
adder_struct_dff test_struct_dff(
	.sum(struct_sum),
	.carry(struct_carry),
	.a(a),
	.b(b),
	.cin(cin),
	.clock(clock)
);
adder_beh_dff test_struct_dff(
	.sum(beh_sum),
	.carry(beh_carry),
	.a(a),
	.b(b),
	.cin(cin),
	.clock(clock)
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