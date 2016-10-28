
module pipeline_DW01_add_0 ( A, B, CI, SUM, CO );
  input [7:0] A;
  input [7:0] B;
  output [7:0] SUM;
  input CI;
  output CO;
  wire   n1;
  wire   [7:1] carry;

  FADDX1 U1_7 ( .A(A[7]), .B(B[7]), .CI(carry[7]), .S(SUM[7]) );
  FADDX1 U1_6 ( .A(A[6]), .B(B[6]), .CI(carry[6]), .CO(carry[7]), .S(SUM[6])
         );
  FADDX1 U1_5 ( .A(A[5]), .B(B[5]), .CI(carry[5]), .CO(carry[6]), .S(SUM[5])
         );
  FADDX1 U1_4 ( .A(A[4]), .B(B[4]), .CI(carry[4]), .CO(carry[5]), .S(SUM[4])
         );
  FADDX1 U1_3 ( .A(A[3]), .B(B[3]), .CI(carry[3]), .CO(carry[4]), .S(SUM[3])
         );
  FADDX1 U1_2 ( .A(A[2]), .B(B[2]), .CI(carry[2]), .CO(carry[3]), .S(SUM[2])
         );
  FADDX1 U1_1 ( .A(A[1]), .B(B[1]), .CI(n1), .CO(carry[2]), .S(SUM[1]) );
  AND2X1 U1 ( .IN1(A[0]), .IN2(B[0]), .Q(n1) );
  XOR2X1 U2 ( .IN1(A[0]), .IN2(B[0]), .Q(SUM[0]) );
endmodule


module pipeline_DW_mult_uns_0 ( a, b, product );
  input [7:0] a;
  input [7:0] b;
  output [15:0] product;
  wire   n2, n3, n4, n5, n6, n7, n8, n9, n10, n11, n12, n13, n14, n15, n16,
         n17, n18, n19, n20, n21, n22, n23, n24, n25, n26, n27, n28, n29, n30,
         n31, n32, n33, n34, n35, n36, n37, n38, n39, n40, n41, n42, n43, n44,
         n45, n46, n47, n48, n49, n50, n51, n52, n53, n54, n55, n56, n57, n58,
         n59, n60, n61, n62, n63, n64, n65, n66, n67, n68, n69, n70, n71, n72,
         n73, n74, n75, n76, n77, n78, n79, n80, n81, n82, n83, n84, n85, n86,
         n87, n88, n89, n90, n91, n92, n93, n94, n95, n96, n97, n98, n99, n100,
         n101, n102, n103, n104, n105, n106, n107, n108, n109, n110, n111,
         n112, n113, n114, n115, n116, n117, n118, n119, n120, n121, n122,
         n123, n124, n125, n126, n127, n128, n129, n130, n131, n132, n133,
         n134, n135, n136, n137, n138, n139, n140, n141, n142, n143, n144,
         n145, n146, n147, n148, n149, n150, n151, n152, n153, n154, n155,
         n156, n157, n158, n159, n160, n161, n214, n215, n216, n217, n218,
         n219, n220, n221, n222, n223, n224, n225, n226, n227, n228, n229;

  FADDX1 U2 ( .A(n15), .B(n99), .CI(n2), .CO(product[15]), .S(product[14]) );
  FADDX1 U3 ( .A(n17), .B(n16), .CI(n3), .CO(n2), .S(product[13]) );
  FADDX1 U4 ( .A(n21), .B(n18), .CI(n4), .CO(n3), .S(product[12]) );
  FADDX1 U5 ( .A(n27), .B(n22), .CI(n5), .CO(n4), .S(product[11]) );
  FADDX1 U6 ( .A(n28), .B(n35), .CI(n6), .CO(n5), .S(product[10]) );
  FADDX1 U7 ( .A(n36), .B(n45), .CI(n7), .CO(n6), .S(product[9]) );
  FADDX1 U8 ( .A(n46), .B(n57), .CI(n8), .CO(n7), .S(product[8]) );
  FADDX1 U9 ( .A(n58), .B(n69), .CI(n9), .CO(n8), .S(product[7]) );
  FADDX1 U10 ( .A(n70), .B(n79), .CI(n10), .CO(n9), .S(product[6]) );
  FADDX1 U11 ( .A(n80), .B(n87), .CI(n11), .CO(n10), .S(product[5]) );
  FADDX1 U12 ( .A(n88), .B(n93), .CI(n12), .CO(n11), .S(product[4]) );
  FADDX1 U13 ( .A(n94), .B(n97), .CI(n13), .CO(n12), .S(product[3]) );
  FADDX1 U14 ( .A(n98), .B(n146), .CI(n14), .CO(n13), .S(product[2]) );
  HADDX1 U15 ( .A0(n154), .B0(n161), .C1(n14), .SO(product[1]) );
  FADDX1 U16 ( .A(n100), .B(n107), .CI(n19), .CO(n15), .S(n16) );
  FADDX1 U17 ( .A(n20), .B(n25), .CI(n23), .CO(n17), .S(n18) );
  FADDX1 U18 ( .A(n101), .B(n115), .CI(n108), .CO(n19), .S(n20) );
  FADDX1 U19 ( .A(n24), .B(n31), .CI(n29), .CO(n21), .S(n22) );
  FADDX1 U20 ( .A(n33), .B(n116), .CI(n26), .CO(n23), .S(n24) );
  FADDX1 U21 ( .A(n102), .B(n123), .CI(n109), .CO(n25), .S(n26) );
  FADDX1 U22 ( .A(n37), .B(n39), .CI(n30), .CO(n27), .S(n28) );
  FADDX1 U23 ( .A(n34), .B(n41), .CI(n32), .CO(n29), .S(n30) );
  FADDX1 U24 ( .A(n117), .B(n124), .CI(n43), .CO(n31), .S(n32) );
  FADDX1 U25 ( .A(n103), .B(n131), .CI(n110), .CO(n33), .S(n34) );
  FADDX1 U26 ( .A(n47), .B(n40), .CI(n38), .CO(n35), .S(n36) );
  FADDX1 U27 ( .A(n51), .B(n44), .CI(n49), .CO(n37), .S(n38) );
  FADDX1 U28 ( .A(n53), .B(n55), .CI(n42), .CO(n39), .S(n40) );
  FADDX1 U29 ( .A(n125), .B(n118), .CI(n132), .CO(n41), .S(n42) );
  FADDX1 U30 ( .A(n104), .B(n139), .CI(n111), .CO(n43), .S(n44) );
  FADDX1 U31 ( .A(n59), .B(n50), .CI(n48), .CO(n45), .S(n46) );
  FADDX1 U32 ( .A(n52), .B(n54), .CI(n61), .CO(n47), .S(n48) );
  FADDX1 U33 ( .A(n65), .B(n67), .CI(n63), .CO(n49), .S(n50) );
  FADDX1 U34 ( .A(n133), .B(n140), .CI(n56), .CO(n51), .S(n52) );
  FADDX1 U35 ( .A(n119), .B(n126), .CI(n147), .CO(n53), .S(n54) );
  HADDX1 U36 ( .A0(n112), .B0(n105), .C1(n55), .SO(n56) );
  FADDX1 U37 ( .A(n71), .B(n62), .CI(n60), .CO(n57), .S(n58) );
  FADDX1 U38 ( .A(n66), .B(n64), .CI(n73), .CO(n59), .S(n60) );
  FADDX1 U39 ( .A(n77), .B(n68), .CI(n75), .CO(n61), .S(n62) );
  FADDX1 U40 ( .A(n120), .B(n141), .CI(n127), .CO(n63), .S(n64) );
  FADDX1 U41 ( .A(n155), .B(n134), .CI(n148), .CO(n65), .S(n66) );
  HADDX1 U42 ( .A0(n113), .B0(n106), .C1(n67), .SO(n68) );
  FADDX1 U43 ( .A(n81), .B(n74), .CI(n72), .CO(n69), .S(n70) );
  FADDX1 U44 ( .A(n83), .B(n85), .CI(n76), .CO(n71), .S(n72) );
  FADDX1 U45 ( .A(n135), .B(n149), .CI(n78), .CO(n73), .S(n74) );
  FADDX1 U46 ( .A(n128), .B(n142), .CI(n156), .CO(n75), .S(n76) );
  HADDX1 U47 ( .A0(n121), .B0(n114), .C1(n77), .SO(n78) );
  FADDX1 U48 ( .A(n84), .B(n89), .CI(n82), .CO(n79), .S(n80) );
  FADDX1 U49 ( .A(n86), .B(n157), .CI(n91), .CO(n81), .S(n82) );
  FADDX1 U50 ( .A(n136), .B(n143), .CI(n150), .CO(n83), .S(n84) );
  HADDX1 U51 ( .A0(n129), .B0(n122), .C1(n85), .SO(n86) );
  FADDX1 U52 ( .A(n95), .B(n92), .CI(n90), .CO(n87), .S(n88) );
  FADDX1 U53 ( .A(n144), .B(n158), .CI(n151), .CO(n89), .S(n90) );
  HADDX1 U54 ( .A0(n137), .B0(n130), .C1(n91), .SO(n92) );
  FADDX1 U55 ( .A(n152), .B(n159), .CI(n96), .CO(n93), .S(n94) );
  HADDX1 U56 ( .A0(n145), .B0(n138), .C1(n95), .SO(n96) );
  HADDX1 U57 ( .A0(n160), .B0(n153), .C1(n97), .SO(n98) );
  INVX0 U140 ( .INP(a[0]), .ZN(n229) );
  INVX0 U141 ( .INP(a[7]), .ZN(n222) );
  INVX0 U142 ( .INP(b[0]), .ZN(n221) );
  INVX0 U143 ( .INP(b[4]), .ZN(n217) );
  INVX0 U144 ( .INP(b[5]), .ZN(n216) );
  INVX0 U145 ( .INP(b[6]), .ZN(n215) );
  INVX0 U146 ( .INP(b[3]), .ZN(n218) );
  INVX0 U147 ( .INP(b[2]), .ZN(n219) );
  INVX0 U148 ( .INP(b[7]), .ZN(n214) );
  INVX0 U149 ( .INP(b[1]), .ZN(n220) );
  INVX0 U150 ( .INP(a[4]), .ZN(n225) );
  INVX0 U151 ( .INP(a[3]), .ZN(n226) );
  INVX0 U152 ( .INP(a[2]), .ZN(n227) );
  INVX0 U153 ( .INP(a[5]), .ZN(n224) );
  INVX0 U154 ( .INP(a[6]), .ZN(n223) );
  INVX0 U155 ( .INP(a[1]), .ZN(n228) );
  NOR2X0 U156 ( .IN1(n229), .IN2(n221), .QN(product[0]) );
  NOR2X0 U157 ( .IN1(n222), .IN2(n214), .QN(n99) );
  NOR2X0 U158 ( .IN1(n229), .IN2(n220), .QN(n161) );
  NOR2X0 U159 ( .IN1(n229), .IN2(n219), .QN(n160) );
  NOR2X0 U160 ( .IN1(n229), .IN2(n218), .QN(n159) );
  NOR2X0 U161 ( .IN1(n229), .IN2(n217), .QN(n158) );
  NOR2X0 U162 ( .IN1(n229), .IN2(n216), .QN(n157) );
  NOR2X0 U163 ( .IN1(n229), .IN2(n215), .QN(n156) );
  NOR2X0 U164 ( .IN1(n229), .IN2(n214), .QN(n155) );
  NOR2X0 U165 ( .IN1(n221), .IN2(n228), .QN(n154) );
  NOR2X0 U166 ( .IN1(n220), .IN2(n228), .QN(n153) );
  NOR2X0 U167 ( .IN1(n219), .IN2(n228), .QN(n152) );
  NOR2X0 U168 ( .IN1(n218), .IN2(n228), .QN(n151) );
  NOR2X0 U169 ( .IN1(n217), .IN2(n228), .QN(n150) );
  NOR2X0 U170 ( .IN1(n216), .IN2(n228), .QN(n149) );
  NOR2X0 U171 ( .IN1(n215), .IN2(n228), .QN(n148) );
  NOR2X0 U172 ( .IN1(n214), .IN2(n228), .QN(n147) );
  NOR2X0 U173 ( .IN1(n221), .IN2(n227), .QN(n146) );
  NOR2X0 U174 ( .IN1(n220), .IN2(n227), .QN(n145) );
  NOR2X0 U175 ( .IN1(n219), .IN2(n227), .QN(n144) );
  NOR2X0 U176 ( .IN1(n218), .IN2(n227), .QN(n143) );
  NOR2X0 U177 ( .IN1(n217), .IN2(n227), .QN(n142) );
  NOR2X0 U178 ( .IN1(n216), .IN2(n227), .QN(n141) );
  NOR2X0 U179 ( .IN1(n215), .IN2(n227), .QN(n140) );
  NOR2X0 U180 ( .IN1(n214), .IN2(n227), .QN(n139) );
  NOR2X0 U181 ( .IN1(n221), .IN2(n226), .QN(n138) );
  NOR2X0 U182 ( .IN1(n220), .IN2(n226), .QN(n137) );
  NOR2X0 U183 ( .IN1(n219), .IN2(n226), .QN(n136) );
  NOR2X0 U184 ( .IN1(n218), .IN2(n226), .QN(n135) );
  NOR2X0 U185 ( .IN1(n217), .IN2(n226), .QN(n134) );
  NOR2X0 U186 ( .IN1(n216), .IN2(n226), .QN(n133) );
  NOR2X0 U187 ( .IN1(n215), .IN2(n226), .QN(n132) );
  NOR2X0 U188 ( .IN1(n214), .IN2(n226), .QN(n131) );
  NOR2X0 U189 ( .IN1(n221), .IN2(n225), .QN(n130) );
  NOR2X0 U190 ( .IN1(n220), .IN2(n225), .QN(n129) );
  NOR2X0 U191 ( .IN1(n219), .IN2(n225), .QN(n128) );
  NOR2X0 U192 ( .IN1(n218), .IN2(n225), .QN(n127) );
  NOR2X0 U193 ( .IN1(n217), .IN2(n225), .QN(n126) );
  NOR2X0 U194 ( .IN1(n216), .IN2(n225), .QN(n125) );
  NOR2X0 U195 ( .IN1(n215), .IN2(n225), .QN(n124) );
  NOR2X0 U196 ( .IN1(n214), .IN2(n225), .QN(n123) );
  NOR2X0 U197 ( .IN1(n221), .IN2(n224), .QN(n122) );
  NOR2X0 U198 ( .IN1(n220), .IN2(n224), .QN(n121) );
  NOR2X0 U199 ( .IN1(n219), .IN2(n224), .QN(n120) );
  NOR2X0 U200 ( .IN1(n218), .IN2(n224), .QN(n119) );
  NOR2X0 U201 ( .IN1(n217), .IN2(n224), .QN(n118) );
  NOR2X0 U202 ( .IN1(n216), .IN2(n224), .QN(n117) );
  NOR2X0 U203 ( .IN1(n215), .IN2(n224), .QN(n116) );
  NOR2X0 U204 ( .IN1(n214), .IN2(n224), .QN(n115) );
  NOR2X0 U205 ( .IN1(n221), .IN2(n223), .QN(n114) );
  NOR2X0 U206 ( .IN1(n220), .IN2(n223), .QN(n113) );
  NOR2X0 U207 ( .IN1(n219), .IN2(n223), .QN(n112) );
  NOR2X0 U208 ( .IN1(n218), .IN2(n223), .QN(n111) );
  NOR2X0 U209 ( .IN1(n217), .IN2(n223), .QN(n110) );
  NOR2X0 U210 ( .IN1(n216), .IN2(n223), .QN(n109) );
  NOR2X0 U211 ( .IN1(n215), .IN2(n223), .QN(n108) );
  NOR2X0 U212 ( .IN1(n214), .IN2(n223), .QN(n107) );
  NOR2X0 U213 ( .IN1(n221), .IN2(n222), .QN(n106) );
  NOR2X0 U214 ( .IN1(n222), .IN2(n220), .QN(n105) );
  NOR2X0 U215 ( .IN1(n222), .IN2(n219), .QN(n104) );
  NOR2X0 U216 ( .IN1(n222), .IN2(n218), .QN(n103) );
  NOR2X0 U217 ( .IN1(n222), .IN2(n217), .QN(n102) );
  NOR2X0 U218 ( .IN1(n222), .IN2(n216), .QN(n101) );
  NOR2X0 U219 ( .IN1(n222), .IN2(n215), .QN(n100) );
endmodule


module pipeline ( A, B, C, D, clk, rst );
  input [7:0] A;
  input [7:0] B;
  input [7:0] C;
  output [15:0] D;
  input clk, rst;
  wire   n2, n3, n4;
  wire   [7:0] Add;
  wire   [15:0] Reg1;
  wire   [15:0] Cross;

  DFFARX1 \Reg1_reg[15]  ( .D(C[7]), .CLK(clk), .RSTB(n4), .Q(Reg1[15]) );
  DFFARX1 \Reg1_reg[14]  ( .D(C[6]), .CLK(clk), .RSTB(n4), .Q(Reg1[14]) );
  DFFARX1 \Reg1_reg[13]  ( .D(C[5]), .CLK(clk), .RSTB(n4), .Q(Reg1[13]) );
  DFFARX1 \Reg1_reg[12]  ( .D(C[4]), .CLK(clk), .RSTB(n4), .Q(Reg1[12]) );
  DFFARX1 \Reg1_reg[11]  ( .D(C[3]), .CLK(clk), .RSTB(n4), .Q(Reg1[11]) );
  DFFARX1 \Reg1_reg[10]  ( .D(C[2]), .CLK(clk), .RSTB(n4), .Q(Reg1[10]) );
  DFFARX1 \Reg1_reg[9]  ( .D(C[1]), .CLK(clk), .RSTB(n4), .Q(Reg1[9]) );
  DFFARX1 \Reg1_reg[8]  ( .D(C[0]), .CLK(clk), .RSTB(n4), .Q(Reg1[8]) );
  DFFARX1 \Reg1_reg[7]  ( .D(Add[7]), .CLK(clk), .RSTB(n3), .Q(Reg1[7]) );
  DFFARX1 \Reg1_reg[6]  ( .D(Add[6]), .CLK(clk), .RSTB(n3), .Q(Reg1[6]) );
  DFFARX1 \Reg1_reg[5]  ( .D(Add[5]), .CLK(clk), .RSTB(n3), .Q(Reg1[5]) );
  DFFARX1 \Reg1_reg[4]  ( .D(Add[4]), .CLK(clk), .RSTB(n3), .Q(Reg1[4]) );
  DFFARX1 \Reg1_reg[3]  ( .D(Add[3]), .CLK(clk), .RSTB(n3), .Q(Reg1[3]) );
  DFFARX1 \Reg1_reg[2]  ( .D(Add[2]), .CLK(clk), .RSTB(n3), .Q(Reg1[2]) );
  DFFARX1 \Reg1_reg[1]  ( .D(Add[1]), .CLK(clk), .RSTB(n3), .Q(Reg1[1]) );
  DFFARX1 \Reg1_reg[0]  ( .D(Add[0]), .CLK(clk), .RSTB(n3), .Q(Reg1[0]) );
  DFFARX1 \Reg2_reg[15]  ( .D(Cross[15]), .CLK(clk), .RSTB(n3), .Q(D[15]) );
  DFFARX1 \Reg2_reg[14]  ( .D(Cross[14]), .CLK(clk), .RSTB(n3), .Q(D[14]) );
  DFFARX1 \Reg2_reg[13]  ( .D(Cross[13]), .CLK(clk), .RSTB(n3), .Q(D[13]) );
  DFFARX1 \Reg2_reg[12]  ( .D(Cross[12]), .CLK(clk), .RSTB(n3), .Q(D[12]) );
  DFFARX1 \Reg2_reg[11]  ( .D(Cross[11]), .CLK(clk), .RSTB(n2), .Q(D[11]) );
  DFFARX1 \Reg2_reg[10]  ( .D(Cross[10]), .CLK(clk), .RSTB(n2), .Q(D[10]) );
  DFFARX1 \Reg2_reg[9]  ( .D(Cross[9]), .CLK(clk), .RSTB(n2), .Q(D[9]) );
  DFFARX1 \Reg2_reg[8]  ( .D(Cross[8]), .CLK(clk), .RSTB(n2), .Q(D[8]) );
  DFFARX1 \Reg2_reg[7]  ( .D(Cross[7]), .CLK(clk), .RSTB(n2), .Q(D[7]) );
  DFFARX1 \Reg2_reg[6]  ( .D(Cross[6]), .CLK(clk), .RSTB(n2), .Q(D[6]) );
  DFFARX1 \Reg2_reg[5]  ( .D(Cross[5]), .CLK(clk), .RSTB(n2), .Q(D[5]) );
  DFFARX1 \Reg2_reg[4]  ( .D(Cross[4]), .CLK(clk), .RSTB(n2), .Q(D[4]) );
  DFFARX1 \Reg2_reg[3]  ( .D(Cross[3]), .CLK(clk), .RSTB(n2), .Q(D[3]) );
  DFFARX1 \Reg2_reg[2]  ( .D(Cross[2]), .CLK(clk), .RSTB(n2), .Q(D[2]) );
  DFFARX1 \Reg2_reg[1]  ( .D(Cross[1]), .CLK(clk), .RSTB(n2), .Q(D[1]) );
  DFFARX1 \Reg2_reg[0]  ( .D(Cross[0]), .CLK(clk), .RSTB(n2), .Q(D[0]) );
  pipeline_DW01_add_0 add_12 ( .A(A), .B(B), .CI(1'b0), .SUM(Add) );
  pipeline_DW_mult_uns_0 mult_13 ( .a(Reg1[7:0]), .b(Reg1[15:8]), .product(
        Cross) );
  NBUFFX2 U4 ( .INP(rst), .Z(n2) );
  NBUFFX2 U5 ( .INP(rst), .Z(n3) );
  NBUFFX2 U6 ( .INP(rst), .Z(n4) );
endmodule
