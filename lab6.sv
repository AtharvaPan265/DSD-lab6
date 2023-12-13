module data_memory(
input clk, rst,
input [31:0] A, //address
input [31:0] WD, //input data
input WE, // write enable input
output [31:0] RD, // output data
output [31:0] probe// to check the data in data memory
);

reg [31:0] mem[255:0] ;
assign probe = mem[1];

always_ff @(posedge clk or negedge rst) begin
	if (!rst) begin
		for (int i = 0 ; i < 256 ; i++) begin
			mem[i] <= i;
		end
	end else begin
		if (WE) begin
			mem[A] <= WD;
			RD = mem[A];
		end
	end
end
endmodule

module MUX_MemtoReg(
input MemtoReg,
input [31:0] ALUResult,
input [31:0] RD, //from data memory
output [31:0] MemtoReg_out
);

always_comb begin
	case(MemtoReg)
		1'b0: MemtoReg_out = ALUResult;
		1'b1: MemtoReg_out = RD;
		default: MemtoReg_out = 32'b0;
	endcase
end
endmodule

module MUX_ALUSrc(
input ALUSrc,
input [31:0] RD2,
input [31:0] SignImm, //from data memory
output [31:0] SrcB
);

always_comb begin
	case(ALUSrc)
		1'b0: SrcB = RD2;
		1'b1: SrcB = SignImm;
		default: SrcB = 32'b0;
	endcase
end
endmodule

module MUX_RegDst(
input RegDst,
input [4:0] rs,
input [4:0] rd, //from data memory
output [4:0] WriteReg
);

always_comb begin
	case(RegDst)
		1'b0: 	WriteReg = rs;
		1'b1: 	WriteReg = rd;
		default: WriteReg = 32'b0;
	endcase
end
endmodule

module sign_extend(
input [15:0] Imm,
output [31:0] SignImm
);
assign SignImm = 32'(signed'(Imm));
endmodule

module register_file(

	input  wire        clk, rst,
	input  wire [4:0]  A1, A2, A3, // A1, A2, A3 are the address
	input  wire [31:0] WD3, 		  //data from data memory
	input  wire 	     WE3, 		  //WE3 = 1, write register file
	output wire [31:0] RD1, 		  //output port one for register file
	output wire [31:0] RD2, 		  //output port two for register file
	output wire [31:0] probe 		  //probe to check the result in the register file
	
);

logic [31:0] registers[31:0];
assign probe = registers[1];
assign RD1 = registers[A1];
assign RD2 = registers[A2];

always_ff@(posedge clk or negedge rst) begin
	if (!rst) begin
		for (int i = 0 ;  i < 32 ; i++) begin
				registers[i] <= i;
			end
	end else begin
		if (!WE3) begin
			registers[1] <= WD3;
		end
	end
end
endmodule

module ALU(
	input  wire [31:0] SrcA,
	input  wire [31:0] SrcB,
	input  wire [2:0]  ALUControl,
	output wire [31:0] ALUResult
);

always_comb begin
	case(ALUControl)
		3'b010:ALUResult = SrcA + SrcB;
		3'b110:ALUResult = SrcA - SrcB;
		default: ALUResult = 31'b0;
	endcase
end
endmodule

module display(
input  wire [6:0] in,
output wire [6:0] seven_seg
);
always_comb begin
	case (in)
		7'b0000000: seven_seg = 7'b1000000; //0
		7'b0000001: seven_seg = 7'b1111001; //1
		7'b0000010: seven_seg = 7'b0100100; //2
		7'b0000011: seven_seg = 7'b0110000; //3
		7'b0000100: seven_seg = 7'b0011001; //4
		7'b0000101: seven_seg = 7'b0010010; //5
		7'b0000110: seven_seg = 7'b0000010; //6
		7'b0000111: seven_seg = 7'b1111000; //7
		7'b0001000: seven_seg = 7'b0000000; //8
		7'b0001001: seven_seg = 7'b0011000; //9
		7'b0001010: seven_seg = 7'b0001000; //a
		7'b0001011: seven_seg = 7'b0000011; //b
		7'b0001100: seven_seg = 7'b0100111; //c
		7'b0001101: seven_seg = 7'b0100001; //d
		7'b0001110: seven_seg = 7'b0000110; //e
		7'b0001111: seven_seg = 7'b0001110; //f
		default: seven_seg = 7'b1;
	endcase
end
endmodule

module lab6(
input  clk, rst,
input  [1:0] sw, //address for instruction memory
output [31:0] ALUResult, //output for pre-lab simulation
output [31:0] RD1, RD2, //output for pre-lab simulation
output [31:0] probe_register_file1,probe_register_fil2e, //output for pre-lab simulation
output [6:0] display_led1,display_led2, //output for in-lab
output [31:0] MemtoReg_out
);


wire[31:0] inst_0 = 32'b0;
wire[31:0] inst_1 =  32'b010101_00000_01000_0000_0000_0000_0101; 
//add rf_regs[5] and rf_regs[4] to rf_regs[1];
wire[31:0] inst_2 =  32'b010100_00000_00000_0000_0000_0000_0010; 
//sub rf_regs[10] - rf_regs[8] to rf_regs[1];
wire[31:0] inst_ex;
assign inst_ex = (sw==1)? inst_1:(sw==2)? inst_2: inst_0;
wire [31:0]t_result, t_a3,t_srcb,t_Sign_imm;

register_file r_f(.clk(clk),.rst(rst),
.A1(inst_ex[25:21]),.A2(inst_ex[20:16]),.A3(t_a3),
.WD3(ALUResult),
.WE3(1),
.RD1(RD1),
.RD2(RD2)
);


ALU t1(
.SrcA(RD1),
.SrcB(t_srcb),
.ALUControl(inst_ex[29:27]),
.ALUResult(ALUResult)
);

data_memory dm(
.A(ALUResult),
.WD(RD2),
.WE(1),
.RD(RD),
.probe(probe_register_file)
);

MUX_MemtoReg mr(
.MemtoReg(inst_ex[26]),
.ALUResult(ALUResult),
.RD(RD), 
.MemtoReg_out
);

MUX_ALUSrc ma(
.RD2(RD2),
.SignImm(t_Sign_imm),
.SrcB(t_srcb),
.ALUSrc(inst_ex[30])
);

MUX_RegDst rdst(
.rs(inst_ex[25:21]),
.rd(inst_ex[15:11]),
.RegDst(inst_ex[31]),
.WriteReg(t_a3)
);
sign_extend(
.Imm(inst_ex[15:0]),
.SignImm(t_Sign_imm)
);
display seg1(ALUResult, display_led1);
endmodule
