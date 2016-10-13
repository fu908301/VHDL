module half_adder(S,C,x,y);
	output S,C;
	input x,y;
	xor(S,x,y);
	and(C,x,y);
endmodule

module full_adder(S,C,x,y,z);
	output S,C;
	input x,y,z;
	wire S1,C1,C2;
	half_adder HA1 (S1,C1,x,y);
	half_adder HA2 (S,C2,S1,z);
	or(C,C2,C1);
endmodule

module adder_struct(sum,cout,a,b,cin);
	output [7:0] sum;
	output cout;
	input [7:0] a,b;
	input cin;
	wire C1,C2,C3,C4,C5,C6,C7;
	full_adder FA0(sum[0],C1,a[0],b[0],cin),
	FA1(sum[1],C2,a[1],b[1],C1),
	FA2(sum[2],C3,a[2],b[2],C2),
	FA3(sum[3],C4,a[3],b[3],C3),
	FA4(sum[4],C5,a[4],b[4],C4),
	FA5(sum[5],C6,a[5],b[5],C5),
	FA6(sum[6],C7,a[6],b[6],C6),
	FA7(sum[7],cout,a[7],b[7],C7);
endmodule



	
