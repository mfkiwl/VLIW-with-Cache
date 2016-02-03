//CONTROL SIGNALS STILL LEFT
//NOOP, BUBBLE MAY REQUIRE REGWRITE, DECOUT SIGNALS
//HARDCODED 1s GIVEN RIGHT NOW
//(clk, reset, regWrite, decOut, writeData, outR)

module IF_ID(input clk, input reset,input if_id_write,  input [31:0] pc, input [31:0] instr,output [31:0] p0_pc, output [15:0] p0_instr1, output [15:0] p0_instr2);
	
	register32bit_single_signal pc_in(clk, reset, if_id_write, 1'b1, pc, p0_pc); //doubt for signals

	register16bit instr_1_in(clk, reset, if_id_write, 1'b1, instr[31:16], p0_instr1);

	register16bit instr_2_in(clk, reset, if_id_write, 1'b1, instr[15:0], p0_instr2);
endmodule

module ID_EX(input clk, input reset, input [31:0] imm, input [31:0] reg_rd, input [31:0] reg_rm, input [31:0] reg_rn, 
	input [31:0] reg_rd2, input [31:0] reg_rn2,input [2:0] rd, input [2:0] rm, input [2:0] rn, input [2:0] aluOp, input [1:0] aluSrc, input memRead,input memWrite,input regWrite1,input regWrite2,
	input [2:0] rd2, input [2:0] rn2, input [31:0] offset, input [3:0] flag, input causeWrite,input p0_op11,input p0_op12,
	output [31:0] p1_imm, output [31:0] p1_reg_rd, output [31:0] p1_reg_rm, output [31:0] p1_reg_rn, 
	output [31:0] p1_reg_rd2, output [31:0] p1_reg_rn2,output [2:0] p1_rd, output [2:0] p1_rm, output [2:0] p1_rn, output p1_causeWrite,
	output [2:0] p1_rd2, output [2:0] p1_rn2, output [31:0] p1_offset,output [3:0] p1_flag, output [2:0] p1_aluOp, output [1:0] p1_aluSrc, output p1_memRead,output p1_memWrite,output p1_regWrite1,output p1_regWrite2,output p1_op11,output p1_op12);
	
	register32bit_single_signal imm1( clk, reset, 1'b1, 1'b1, imm, p1_imm);

	register32bit_single_signal r_rd( clk, reset, 1'b1, 1'b1, reg_rd,  p1_reg_rd );
	register32bit_single_signal r_rn( clk, reset, 1'b1, 1'b1, reg_rn,  p1_reg_rn );
	register32bit_single_signal r_rm( clk, reset, 1'b1, 1'b1, reg_rm,  p1_reg_rm );


	register32bit_single_signal r_rn2( clk, reset, 1'b1, 1'b1, reg_rn2,  p1_reg_rn2 );
	register32bit_single_signal r_rd2( clk, reset, 1'b1, 1'b1, reg_rd2,  p1_reg_rd2 );

	register32bit_single_signal off( clk, reset, 1'b1, 1'b1, offset,  p1_offset );
	register3bit d( clk, reset, 1'b1, 1'b1, rd,  p1_rd );
	register3bit m( clk, reset, 1'b1, 1'b1, rm,  p1_rm );
	register3bit n( clk, reset, 1'b1, 1'b1, rn,  p1_rn );
	register3bit n2( clk, reset, 1'b1, 1'b1, rn2,  p1_rn2 );
	register3bit d2( clk, reset, 1'b1, 1'b1, rd2,  p1_rd2 );

	register1bit op11(clk, reset, 1'b1,1'b1, p0_op11, p1_op11);
	register1bit op12(clk, reset, 1'b1,1'b1, p0_op12, p1_op12);

	register3bit alOp( clk, reset, 1'b1, 1'b1, aluOp,  p1_aluOp);
	register2bit aluSc( clk, reset, 1'b1,1'b1, aluSrc, p1_aluSrc );
	register1bit memRd(clk, reset, 1'b1,1'b1, memRead, p1_memRead);
	register1bit memWt(clk, reset, 1'b1,1'b1, memWrite, p1_memWrite);
	register1bit regWt1(clk, reset, 1'b1,1'b1, regWrite1, p1_regWrite1);
	register1bit regWt2(clk, reset, 1'b1,1'b1, regWrite2, p1_regWrite2);
	register1bit cause(clk, reset, 1'b1,1'b1, causeWrite, p1_causeWrite);
	register4bit flg(clk, reset, 1'b1,1'b1,flag,p1_flag);
	endmodule

module EX_MEM(input clk, input reset, input [2:0] p1_rd1, input [31:0] aluOut, input [3:0] p1_flag, input [31:0] adderOut, input [2:0] p1_rd2, 
input [2:0] p1_aluOp, input [1:0] p1_aluSrc, input p1_memRead,input p1_memWrite,input p1_regWrite1,input p1_regWrite2,input [31:0] p1_reg_rd2,
output [2:0] p2_rd1, output [31:0] p2_aluOut, output [31:0] p2_adderOut, output [2:0] p2_rd2,output [3:0] p2_flag, output [2:0] p2_aluOp, 
output [1:0] p2_aluSrc, output p2_memRead,output p2_memWrite,output p2_regWrite1,output p2_regWrite2, output [31:0] p2_reg_rd2);

	register3bit d1( clk, reset, 1'b1, 1'b1, p1_rd1, p2_rd1);
	register32bit_single_signal alu1( clk, reset, 1'b1, 1'b1, aluOut, p2_aluOut );
	register4bit fl( clk,  reset,  1'b1,  1'b1, p1_flag, p2_flag );
	register32bit_single_signal alu2( clk, reset, 1'b1, 1'b1, adderOut, p2_adderOut );
	register3bit d2( clk, reset, 1'b1, 1'b1, p1_rd2, p2_rd2 );
	register32bit_single_signal reg_d2( clk, reset, 1'b1, 1'b1, p1_reg_rd2, p2_reg_rd2 );
	register3bit alOp( clk, reset, 1'b1, 1'b1, p1_aluOp,  p2_aluOp);
	register2bit aluSc( clk, reset, 1'b1,1'b1, p1_aluSrc,p2_aluSrc  );
	register1bit memRd(clk, reset, 1'b1,1'b1, p1_memRead, p2_memRead);
	register1bit memWt(clk, reset, 1'b1,1'b1, p1_memWrite, p2_memWrite);
	register1bit regWt1(clk, reset, 1'b1,1'b1, p1_regWrite1, p2_regWrite1);
	register1bit regWt2(clk, reset, 1'b1,1'b1, p1_regWrite2, p2_regWrite2);
endmodule

module MEM_WB(input clk, input reset, input [2:0] p2_rd1, input [31:0] p2_aluOut, input [3:0] p2_flag, input [31:0] memOut, input [2:0] p2_rd2,
input p2_regWrite1,input p2_regWrite2,
output [2:0] p3_rd1, output [31:0] p3_aluOut, output [3:0] p3_flag, output [31:0] p3_memOut, output [2:0] p3_rd2,output p3_regWrite1,output p3_regWrite2);

	register3bit rd( clk, reset, 1'b1, 1'b1, p2_rd1, p3_rd1 );
	register32bit_single_signal alu1( clk, reset, 1'b1, 1'b1, p2_aluOut, p3_aluOut );
	register4bit fl( clk,  reset,  1'b1,  1'b1, p2_flag, p3_flag );
	register32bit_single_signal alu2( clk, reset, 1'b1, 1'b1, memOut, p3_memOut );
	register3bit rd2( clk, reset, 1'b1, 1'b1, p2_rd2, p3_rd2 );
	register1bit regWt1(clk, reset, 1'b1,1'b1, p2_regWrite1, p3_regWrite1);
	register1bit regWt2(clk, reset, 1'b1,1'b1, p2_regWrite2, p3_regWrite2);
	
endmodule

module zext5_to_32(input [4:0] offset,output reg [31:0] zero_ext_offset);
	always@(offset)
		begin
			zero_ext_offset={{27{1'b0}},offset};
		end

endmodule

module zext16_to_32(input [15:0] offset,output reg [31:0] zero_ext_offset);
	always@(offset)
		begin
			zero_ext_offset={{16{1'b0}},offset};
		end

endmodule


module sext11_to_28(input [10:0] offset,output reg [27:0] zero_ext_offset);
	always@(offset)
		begin
			zero_ext_offset={{17{offset[10]}},offset};
		end

endmodule

module sext8_to_32(input [7:0] offset,output reg [31:0] zero_ext_offset);
	always@(offset)
		begin
			zero_ext_offset={{24{offset[7]}},offset};
		end
endmodule

 module mux_2_to_1(input [31:0] line1,input[31:0] line2,input select,output reg [31:0] muxOut); 
   always @ (line1,line2,select) begin
     case(select)
       1'b0 : muxOut = line1;
       1'b1 : muxOut = line2;
     endcase
   end
 endmodule

 module mux_3_to_1(input [31:0] line1,input[31:0] line2,input[31:0] line3,input [1:0] select,output reg [31:0] muxOut); 
   always @ (line1,line2,line3,select) begin
     case(select)
       2'b00 : muxOut = line1;
       2'b01 : muxOut = line2;
       2'b10 : muxOut = line3;
     endcase
   end
 endmodule

module mux_4_to_1(input [31:0] line1,input[31:0] line2,input[31:0] line3,input[31:0] line4,input [1:0] select,output reg [31:0] muxOut); 
   always @ (line1,line2,line3,line4,select) begin
     case(select)
       2'b00 : muxOut = line1;
       2'b01 : muxOut = line2;
       2'b10 : muxOut = line3;
       2'b11: muxOut=line4;
     endcase
   end
 endmodule



 module shiftleft_1bit(input [31:0] zero_ext_offset,output reg [31:0] left_shifted_zero_extended_offset);
 	always@(zero_ext_offset)
 	begin
 		left_shifted_zero_extended_offset=zero_ext_offset<<1;

 	end
 endmodule

 module shiftleft_2bit(input [27:0] sign_ext_offset,output reg [27:0] left_shifted_sign_extended_offset);
 	always@(sign_ext_offset)
 	begin
 		left_shifted_sign_extended_offset=sign_ext_offset<<2;

 	end
 endmodule
 
 module shiftleft_2bit_32(input [31:0] sign_ext_offset,output reg [31:0] left_shifted_sign_extended_offset);
 	always@(sign_ext_offset)
 	begin
 		left_shifted_sign_extended_offset=sign_ext_offset<<2;

 	end
 endmodule

//in top module modify neg flag with msb of memOut
module ctrlCkt	(	input [4:0] opcode1,input [4:0] opcode2, input negFlag, input overflowFlag,input [1:0] func, output reg [2:0] aluOp,
output reg [1:0] aluSrcSet1,output reg regWrite2, output reg memRead, output reg memWrite, output reg [1:0] pcSrc, output reg regWrite1, 
output reg exFlush, output reg ifFlush, output reg idFlush, output reg causeWrite);
	
	always@(opcode1 or opcode2 or negFlag or overflowFlag or func)
	    begin
	      idFlush = 0;
	      ifFlush = 0;
	      exFlush = 0;
	      pcSrc=2'b00;
	      causeWrite = 0;
		case(opcode1)
			5'b00011:
			begin
				//000 for add
				if(func == 2'b00) aluOp=3'b000;
				//001 for sub
				if(func == 2'b01) aluOp=3'b001;
				aluSrcSet1=2'b00; //for Rm
				
				regWrite1 = 1;
				if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	      exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
			end
			5'b00010:
			begin
				//010 for shift
				aluOp=3'b010;
				aluSrcSet1=2'b01; // for Immediate
				
				regWrite1 = 1;
				if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
			end
			5'b01000:
			begin
				
				if(func == 2'b00) 
				begin
					//011 for tst
					aluOp=3'b011;
					regWrite1 = 0;
				end
				
				if(func == 2'b01)
				begin
					//100 for bic
					aluOp=3'b100;
					regWrite1 = 1;
				end
				aluSrcSet1=2'b10; // for Rd
				if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
			end
			5'b00000:
				  begin
				    aluOp=3'b000;
				    aluSrcSet1=2'b00;
						regWrite1=0;
						if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
					end
			default:
				begin
						causeWrite=1;
						pcSrc = 2'b11;
						ifFlush = 1;
						  idFlush = 1;
				end
		endcase
		case(opcode2)
			//for second set
					//regDst is always Rd in set2
				5'b01111:		
					begin
						memRead=1;
						memWrite=0;
						regWrite2=1;
						if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
					end
				5'b01110:
					begin
						memRead=0;
						memWrite=1;
						regWrite2=0;
						if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
					end
				5'b11110:
					begin
						memRead=0;
						memWrite=0;
						regWrite2=0;
						ifFlush = 1;
						//01 for jump
						if(pcSrc == 2'b00)
						pcSrc=2'b01;
						if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
					end
				5'b11010:
					begin
						memRead=0;
						memWrite=0;
						regWrite2=0;
						//10 for branch
						
						if(negFlag==0 && pcSrc == 2'b00) pcSrc=2'b10;
						  if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
					end
				5'b00000:
				  begin
				    memRead=0;
						memWrite=0;
						regWrite2=0;
						
						if(overflowFlag == 1)
	    begin
	      idFlush = 1;
	      ifFlush = 1;
	        exFlush = 1;
	      causeWrite = 0;
	      pcSrc = 2'b11;
	    end
	    end
				default:
				begin
						causeWrite=1;
						pcSrc=2'b11;
						ifFlush=1;
						  idFlush=1;
						
				end
		endcase
	
		
	 end
endmodule

module alu(input [31:0] aluIn1, input [31:0] aluIn2, input [3:0] flagIn, input [2:0] aluOp,input p1_regWrite, output reg[31:0] aluOut, 
output reg[3:0] flagOut);
//order - N,Z,C,V
reg [32:0] temp;
reg [31:0] temp2;
always @ (aluIn1 or aluIn2 or aluOp or flagIn or p1_regWrite)
	begin
		flagOut = flagIn;
		if(p1_regWrite==0 && aluOp == 3'b0)
		  aluOut= 32'b0;
		else
		begin
		case(aluOp)
			3'b000: begin //ADD
				temp = aluIn1 + aluIn2;
				aluOut = aluIn1 + aluIn2;
				if(aluIn1[31]==0 && aluIn2[31]==0 && aluOut[31]==1) flagOut[0] = 1;//0 - V flag
				else if(aluIn1[31]==1 && aluIn2[31]==1 && aluOut[31]==0) flagOut[0] = 1;//0 - V flag
				else flagOut[0] = 0;
				if(temp[32]==1) flagOut[1] = 1;//1 - C flag
				else flagOut[1] = 0;
				if(aluOut == {32{1'b0}}) flagOut[2] = 1;//2 - Z flag
				else flagOut[2] = 0;
				if(aluOut[31] == 1) flagOut[3] = 1;//3 - N flag
				else flagOut[3] = 0;
				end
			3'b001: begin 
				//rd=rn-rm
				aluOut = aluIn1 - aluIn2;
				if(aluIn1[31]==1 && aluIn2[31]==0 && aluOut[31]==0) flagOut[0] = 1;//0 - V flag
				else if(aluIn1[31]==0 && aluIn2[31]==1 && aluOut[31]==1) flagOut[0] = 1;//0 - V flag
				else flagOut[0] = 0;
				if(aluIn1 < aluIn2) flagOut[1] = 1;//1 - C flag
				else flagOut[1] = 0;
				if(aluOut == {32{1'b0}}) flagOut[2] = 1;//2 - Z flag
				else flagOut[2] = 0;
				if(aluOut[31] == 1) flagOut[3] = 1;//3 - N flag
				else flagOut[3] = 0;
				end
			3'b010: begin
				//rn>>>imm
				aluOut = aluIn1 >>> aluIn2;
				temp2 = (aluIn2 - 1)%32;
				//CHECK!!
				flagOut[1] = aluIn2[temp2];//1 - C flag
				//
				if(aluOut == {32{1'b0}}) flagOut[2] = 1;//2 - Z flag
				else flagOut[2] = 0;
				if(aluOut[31] == 1) flagOut[3] = 1;//3 - N flag
			  else flagOut[3] = 0;
				end
			3'b011: begin
				
				aluOut = aluIn1 & aluIn2;
				//check this
				if(aluOut == {32{1'b0}}) flagOut[2] = 1;//2 - Z flag
				else flagOut[2] = 0;
				//if(aluOut[31] == 1) flagOut[3] = 1;//3 - N flag
				//else flagOut[3] = 0;
				end
			3'b100: begin
				aluOut = aluIn2 & (~aluIn1);
				//check this
				if(aluOut == {32{1'b0}}) flagOut[2] = 1;//2 - Z flag
				else flagOut[2] = 0;
				if(aluOut[31] == 1) flagOut[3] = 1;//3 - N flag
				else flagOut[3] = 0;
				end
		endcase
		end
		end
endmodule



module hazard_detection_unit(input [4:0] opcode2, input id_ex_memread, input id_ex_regWrite1,input[2:0] id_ex_Rd2,input [2:0] if_id_Rn1,
input [2:0] if_id_Rm1, input [2:0] if_id_Rd1, input [2:0] if_id_Rn2, input opcode_11,input opcode_12,output reg pcWrite,output reg if_id_write,
output reg ex_id_control);
	always@(id_ex_memread or id_ex_Rd2 or if_id_Rn1 or if_id_Rm1 or if_id_Rd1 or id_ex_Rd2 or if_id_Rn2 or opcode2 or id_ex_regWrite1)
		begin
		if_id_write=1;
		ex_id_control=0;//mux 	
		pcWrite=1;
		//included 2 stalls before each branch
		if((id_ex_memread && id_ex_Rd2==if_id_Rn1)||(id_ex_memread && id_ex_Rd2==if_id_Rm1&& opcode_11==1)||(id_ex_memread && id_ex_Rd2==if_id_Rd1 &&opcode_12==0)
		||(id_ex_memread && id_ex_Rd2==if_id_Rn2)||((id_ex_regWrite1) && opcode2 == 5'b11010))
			begin
				if_id_write=0;
				ex_id_control=1;//mux 	
				pcWrite=0;
			end	
		end 	
	
endmodule

//D flip flop
module D_ff (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1'b1)
		q=0;
	else
		if(regWrite == 1'b1 && decOut1b==1'b1) begin q=d; end
		
	end
endmodule

module D_ff_regset (input clk, input reset, input regWrite_set1,input regWrite_set2, input decOut1b_set1,input decOut1b_set2, input init,
input d_s1,input d_s2, output reg q);
	always @ (posedge clk)
	begin
	if(reset==1'b1)
		q=init;
	else
		if(regWrite_set1 == 1'b1 && decOut1b_set1==1'b1) begin q=d_s1; end

		if(regWrite_set2 == 1'b1 && decOut1b_set2==1'b1) begin q=d_s2; end	
	end
endmodule

module register1bit( input clk, input reset, input regWrite, input decOut1b, input  writeData, output  outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData, outR);
endmodule


module register2bit( input clk, input reset, input regWrite, input decOut1b, input[1:0] writeData, output[1:0]  outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	
endmodule


module register3bit( input clk, input reset, input regWrite, input decOut1b, input[2:0]  writeData, output[2:0]  outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	
endmodule

module register4bit( input clk, input reset, input regWrite, input decOut1b, input[3:0]  writeData, output[3:0]  outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	
endmodule

module register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
		
endmodule

module register32bit_single_signal( input clk, input reset, input regWrite, input decOut1b, input [31:0] writeData, output  [31:0] outR );
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
	D_ff d16(clk, reset, regWrite, decOut1b, writeData[16], outR[16]);
	D_ff d17(clk, reset, regWrite, decOut1b, writeData[17], outR[17]);
	D_ff d18(clk, reset, regWrite, decOut1b, writeData[18], outR[18]);
	D_ff d19(clk, reset, regWrite, decOut1b, writeData[19], outR[19]);
	D_ff d20(clk, reset, regWrite, decOut1b, writeData[20], outR[20]);
	D_ff d21(clk, reset, regWrite, decOut1b, writeData[21], outR[21]);
	D_ff d22(clk, reset, regWrite, decOut1b, writeData[22], outR[22]);
	D_ff d23(clk, reset, regWrite, decOut1b, writeData[23], outR[23]);
	D_ff d24(clk, reset, regWrite, decOut1b, writeData[24], outR[24]);
	D_ff d25(clk, reset, regWrite, decOut1b, writeData[25], outR[25]);
	D_ff d26(clk, reset, regWrite, decOut1b, writeData[26], outR[26]);
	D_ff d27(clk, reset, regWrite, decOut1b, writeData[27], outR[27]);
	D_ff d28(clk, reset, regWrite, decOut1b, writeData[28], outR[28]);
	D_ff d29(clk, reset, regWrite, decOut1b, writeData[29], outR[29]);
	D_ff d30(clk, reset, regWrite, decOut1b, writeData[30], outR[30]);
	D_ff d31(clk, reset, regWrite, decOut1b, writeData[31], outR[31]);
		
endmodule

 
//Register File
module register32bit( input clk, input reset, input regWrite_set1,input regWrite_set2, input decOut1b_set1,input decOut1b_set2, input [31:0] init, 
input [31:0] writeData_set1,input[31:0] writeData_set2, output  [31:0] outR );
  
	D_ff_regset d[31:0](clk, reset, regWrite_set1,regWrite_set2, decOut1b_set1,decOut1b_set2,init, writeData_set1,writeData_set2, outR);
		
endmodule

module registerSet( input clk, input reset, input regWrite_set1,input regWrite_set2, input [7:0] decOut_set1,input[7:0] decOut_set2, 
input [31:0] writeData_set1,input[31:0] writeData_set2 ,output [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7);
    //r0=0
		register32bit r0 (clk, reset, 1'b0,1'b0, decOut_set1[0] ,decOut_set2[0],32'b 000_0000_0000_00000_000_0000_0000_00000, 
		writeData_set1,writeData_set2 , outR0 );
		//r1=1
		register32bit r1 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[1] ,decOut_set2[1], 32'b 000_0000_0000_00000_000_0000_0000_00001,
		 writeData_set1,writeData_set2 , outR1 );
		 //r2=2
		register32bit r2 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[2] ,decOut_set2[2], 32'b 000_0000_0000_00000_000_0000_0000_00010, 
		writeData_set1,writeData_set2 , outR2 );
		//..and so on initially
		register32bit r3 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[3] ,decOut_set2[3], 32'b 011_1111_1111_11111_111_1111_1111_11111, 
		writeData_set1,writeData_set2 , outR3 );
		register32bit r4 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[4] ,decOut_set2[4], 32'b 000_0000_0000_00000_000_0000_0000_00100, 
		writeData_set1,writeData_set2 , outR4 );
		register32bit r5 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[5] ,decOut_set2[5], 32'b 000_0000_0000_00000_000_0000_0000_00101, 
		writeData_set1,writeData_set2 , outR5 );
		register32bit r6 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[6] ,decOut_set2[6], 32'b 000_0000_0000_00000_000_0000_0000_00110, 
		writeData_set1,writeData_set2 , outR6 );
		register32bit r7 (clk, reset, regWrite_set1,regWrite_set2, decOut_set1[7] ,decOut_set2[7], 32'b 000_0000_0000_00000_000_0000_0000_00111, 
		writeData_set1,writeData_set2 , outR7 );

endmodule

module adder_32bit(input[31:0] in1,input[31:0] in2,output reg[31:0] outp);
	always@(in1,in2)
		begin 
			outp=in1+in2;
		end
endmodule

module subtract_32bit(input[31:0] in1,input[31:0] in2,output reg[31:0] outp);
	always@(in1,in2)
		begin 
			outp=in1-in2;
		end
endmodule

module decoder3to8( input [2:0] destReg, output reg [7:0] decOut);
	always@(destReg)
	case(destReg)
			3'b000: decOut=8'b00000001; 
			3'b001: decOut=8'b00000010;
			3'b010: decOut=8'b00000100;
			3'b011: decOut=8'b00001000;
			3'b100: decOut=8'b00010000;
			3'b101: decOut=8'b00100000;
			3'b110: decOut=8'b01000000;
			3'b111: decOut=8'b10000000;
			
	endcase
endmodule

module decoder4to16( input [3:0] destReg, output reg [15:0] decOut);
	always@(destReg)
	case(destReg)
			4'b0000: decOut=16'b0000000000000001; 
			4'b0001: decOut=16'b0000000000000010;
			4'b0010: decOut=16'b0000000000000100;
			4'b0011: decOut=16'b0000000000001000;
			4'b0100: decOut=16'b0000000000010000;
			4'b0101: decOut=16'b0000000000100000;
			4'b0110: decOut=16'b0000000001000000;
			4'b0111: decOut=16'b0000000010000000;
			4'b1000: decOut=16'b0000000100000000; 
			4'b1001: decOut=16'b0000001000000000;
			4'b1010: decOut=16'b0000010000000000;
			4'b1011: decOut=16'b0000100000000000;
			4'b1100: decOut=16'b0001000000000000;
			4'b1101: decOut=16'b0010000000000000;
			4'b1110: decOut=16'b0100000000000000;
			4'b1111: decOut=16'b1000000000000000;
	endcase
endmodule

module mux_32bit_8to1( input [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,input [2:0] Sel, output reg [31:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or Sel)
	case (Sel)
				3'b000: outBus=outR0;
				3'b001: outBus=outR1;
				3'b010: outBus=outR2;
				3'b011: outBus=outR3;
				3'b100: outBus=outR4;
				3'b101: outBus=outR5;
				3'b110: outBus=outR6;
				3'b111: outBus=outR7;
				
	endcase
endmodule

module mux_10bit_2to1( input [9:0] outR0,outR1,input  Sel, output reg [9:0] outBus );
	always@(outR0 or outR1 or Sel)
	case (Sel)
				1'b0:outBus=outR0;
				
				1'b1:outBus=outR1;
				
	endcase
endmodule

module registerFile(input clk, input reset, input regWrite_set1,inout regWrite_set2, input [2:0] srcRegA_set1, input [2:0] srcRegB_set1, 
input [2:0] srcRegC_set1, input[2:0] srcRegA_set2,input[2:0] srcRegB_set2,input [2:0] destReg_set1,input[2:0] destReg_set2,  input [31:0] writeData_set1,
input[31:0] writeData_set2, output [31:0] outBusA_set1, output [31:0] outBusB_set1,output [31:0] outBusC_set1, output [31:0] outBusA_set2, 
output [31:0] outBusB_set2 );
	wire [7:0] decOut_set1;
	wire [7:0] decOut_set2;
	wire [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7;
	decoder3to8 d_set1 (destReg_set1,decOut_set1);
	decoder3to8 d_set2 (destReg_set2,decOut_set2);
	registerSet rSet0(clk, reset, regWrite_set1,regWrite_set2, decOut_set1,decOut_set2, writeData_set1,writeData_set2, outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7);
	mux_32bit_8to1 m1(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,srcRegA_set1,outBusA_set1);
	mux_32bit_8to1 m2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,srcRegB_set1,outBusB_set1);
	mux_32bit_8to1 m3(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,srcRegC_set1,outBusC_set1);
	mux_32bit_8to1 m1_set2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,srcRegA_set2,outBusA_set2);
	mux_32bit_8to1 m2_set2(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,srcRegB_set2,outBusB_set2);

endmodule

module flag( input clk, input reset, input regWrite, input decOut1b, input[3:0] writeData, output[3:0]  flagOut);
	//order - N,Z,C,V
	D_ff d0(clk, reset, regWrite, decOut1b, writeData[0], flagOut[0]);
	D_ff d1(clk, reset, regWrite, decOut1b, writeData[1], flagOut[1]);
	D_ff d2(clk, reset, regWrite, decOut1b, writeData[2], flagOut[2]);
	D_ff d3(clk, reset, regWrite, decOut1b, writeData[3], flagOut[3]);
endmodule

module D_ff_Mem (input clk, input reset, input regWrite, input decOut1b,input init, input d, output reg q);
	always @ (negedge clk)
	begin
	if(reset==1)
		q=init;
	else
		if(regWrite == 1 && decOut1b==1) begin q=d; end
	end
endmodule

module register_Mem(input clk,input reset,input regWrite,input decOut1b,input [31:0]init, input [31:0] d_in, output [31:0] q_out);
	D_ff_Mem dMem[31:0] (clk,reset,regWrite,decOut1b,init,d_in,q_out);
	endmodule
	
module Mem(input clk, input reset,input memWrite,input memRead, input [31:0] pc, input [31:0] dataIn,output [31:0] IR );
	wire [31:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15;
	wire [15:0] decOut;
	
	decoder4to16 dec0( pc[5:2], decOut);
	
	register_Mem r0(clk,reset,memWrite,decOut[0],32'b 000_1100_1111_11001_011_1000_0001_01101,dataIn,Qout0); //add $r1,$r7,$r7 - sw $r5 $r5(0)
	register_Mem r1(clk,reset,memWrite,decOut[1],32'b 000_1101_0000_01010_011_1100_0001_01100,dataIn,Qout1); //sub $r2,$r1,$r0 - lw $r4 $r5(0)
	//register_Mem r2(clk,reset,memWrite,decOut[2],32'b 000_1100_1100_11010_110_1010_1000_00011,dataIn,Qout2); //Overflow - bne 3
	//register_Mem r2(clk,reset,memWrite,decOut[2],32'b 111_1000_0010_11011_110_1010_1000_00011,dataIn,Qout2); //Invalid - bne 3
	register_Mem r2(clk,reset,memWrite,decOut[2],32'b 000_1000_0011_00110_011_1000_0000_10010,dataIn,Qout2);//sft r6 r4 1 - sw r2 r2
	register_Mem r3(clk,reset,memWrite,decOut[3],32'b 010_0000_0111_00100_110_1010_1000_00011,dataIn,Qout3); //tst r4 r4 - bne 3
	
	//register_Mem r4(clk,reset,memWrite,decOut[4],32'b 000_0000_0000_00000_111_1000_0000_00101,dataIn,Qout4); //nop - j 5
	register_Mem r4(clk,reset,memWrite,decOut[4],32'b 000_0000_0000_00000_000_0000_0000_00000,dataIn,Qout4);//nop-nop
	register_Mem r5(clk,reset,memWrite,decOut[5],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout5); //bic r7 r0 - nop
	register_Mem r6(clk,reset,memWrite,decOut[6],32'b 000_1100_1111_11001_000_0000_0000_00000,dataIn,Qout6); //add $r1,$r7,$r7-nop
	register_Mem r7(clk,reset,memWrite,decOut[7],32'b 000_1100_1110_11101_011_1100_0000_10100,dataIn,Qout7);  //add r5 r3 r7 - lw r4 r2
	
	register_Mem r8(clk,reset,memWrite,decOut[8],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout8); //bic r7 r0 - nop
	register_Mem r9(clk,reset,memWrite,decOut[9],32'b 000_1100_1110_11101_011_1100_0000_10100,dataIn,Qout9); //add $r1,$r7,$r7-nop
	register_Mem r10(clk,reset,memWrite,decOut[10],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout10);//bic r7 r0 - nop 
	register_Mem r11(clk,reset,memWrite,decOut[11],32'b 000_1100_1110_11101_011_1100_0000_10100,dataIn,Qout11); //add $r1,$r7,$r7-nop
	
	register_Mem r12(clk,reset,memWrite,decOut[12],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout12);//bic r7 r0 - nop	
	register_Mem r13(clk,reset,memWrite,decOut[13],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout13); //add $r1,$r7,$r7-nop
	register_Mem r14(clk,reset,memWrite,decOut[14],32'b 010_0001_1110_00111_000_0000_0000_00000,dataIn,Qout14);//bic r7 r0 - nop 
	register_Mem r15(clk,reset,memWrite,decOut[15],32'b 010_0001_1110_00110_000_0000_0000_00000,dataIn,Qout15); //bic r6 r0 - nop
	
	
	
	
	
	
	mux16to1_32bit m0(Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc[5:2],IR);
endmodule


module register_Mem_16(input clk,input reset,input regWrite,input decOut1b,input [15:0]init, input [15:0] d_in, output [15:0] q_out);
	D_ff_Mem dMem[15:0] (clk,reset,regWrite,decOut1b,init,d_in,q_out);
endmodule

module Mem_16bit(input clk, input reset,input memWrite,input memRead, input [31:0] pc, input [15:0] dataIn,output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15,decOut;
	
	decoder4to16 dec0( pc[3:0], decOut);
	
	register_Mem_16 r0(clk,reset,memWrite,decOut[0],16'b 000_0000_0000_00000,dataIn,Qout0); 
	register_Mem_16 r1(clk,reset,memWrite,decOut[1],16'b 000_0000_0000_00000,dataIn,Qout1); 
	register_Mem_16 r2(clk,reset,memWrite,decOut[2],16'b 000_0000_0000_00000,dataIn,Qout2); //All nops, change this
	register_Mem_16 r3(clk,reset,memWrite,decOut[3],16'b 000_0000_0000_00000,dataIn,Qout3); 
	
	register_Mem_16 r4(clk,reset,memWrite,decOut[4],16'b 000_0000_0000_00000,dataIn,Qout4); 
	register_Mem_16 r5(clk,reset,memWrite,decOut[5],16'b 000_0000_0000_00000,dataIn,Qout5); 
	register_Mem_16 r6(clk,reset,memWrite,decOut[6],16'b 000_0000_0000_00000,dataIn,Qout6); 
	register_Mem_16 r7(clk,reset,memWrite,decOut[7],16'b 000_0000_0000_00000,dataIn,Qout7);  
	
	register_Mem_16 r8(clk,reset,memWrite,decOut[8],16'b 000_0000_0000_00000,dataIn,Qout8); 
	register_Mem_16 r9(clk,reset,memWrite,decOut[9],16'b 000_0000_0000_00000,dataIn,Qout9); 
	register_Mem_16 r10(clk,reset,memWrite,decOut[10],16'b 000_0000_0000_00000,dataIn,Qout10); 
	register_Mem_16 r11(clk,reset,memWrite,decOut[11],16'b 000_0000_0000_00000,dataIn,Qout11); 
	
	register_Mem_16 r12(clk,reset,memWrite,decOut[12],16'b 000_0000_0000_00000,dataIn,Qout12);	
	register_Mem_16 r13(clk,reset,memWrite,decOut[13],16'b 000_0000_0000_00000,dataIn,Qout13); 
	register_Mem_16 r14(clk,reset,memWrite,decOut[14],16'b 000_0000_0000_00000,dataIn,Qout14); 
	register_Mem_16 r15(clk,reset,memWrite,decOut[15],16'b 000_0000_0000_00000,dataIn,Qout15); 
	
	mux16to1 mux_Mem(Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc[3:0],IR);
endmodule

//ensure that the changes are made in the pipeline
module fwd(input [2:0] id_ex_Rn1,input [2:0] id_ex_Rm1,input [2:0] id_ex_Rd1,input [2:0] id_ex_Rn2,input op11,input op12,input ex_mem_regWrite1,
input mem_wb_regWrite2,
input mem_wb_regWrite1,input [2:0]ex_mem_Rd1,input [2:0] mem_wb_Rd1,input [2:0]id_ex_Rd2,input [2:0]ex_mem_Rd2,input [2:0] mem_wb_Rd2,output reg[1:0]Rn1_sel,
output reg[1:0]Rm1_sel,output reg[1:0]Rd1_sel,output reg[1:0]Rn2_sel,output reg[1:0]Rd2_sel);
	always@(id_ex_Rn1,id_ex_Rm1,id_ex_Rd1,id_ex_Rn2,op11,op12,ex_mem_regWrite1,mem_wb_regWrite1,ex_mem_Rd1,mem_wb_Rd1,mem_wb_Rd2,mem_wb_regWrite2)
		begin
			Rm1_sel=2'b00;
			Rd1_sel=2'b00;
			Rn1_sel=2'b00;
			Rn2_sel=2'b00;
			Rd2_sel=2'b00;
			//00-normal, 01-ex_mem aluOut, 10-mem_wb memOut, 11-mem_wb aluOut
			//for Rm1
			if(ex_mem_regWrite1==1 && ex_mem_Rd1!=3'b000 && op11==1 && ex_mem_Rd1==id_ex_Rm1)
				Rm1_sel=2'b01;
			//Load write is given priority over set 1 write in D_FF
			else if(mem_wb_regWrite2==1 && mem_wb_Rd2!=3'b000 && op11==1 && mem_wb_Rd2==id_ex_Rm1)
				Rm1_sel=2'b10;
			else if(mem_wb_regWrite1==1 && mem_wb_Rd1!=3'b000 && op11==1 && mem_wb_Rd1==id_ex_Rm1)
				Rm1_sel=2'b11;
			//for Rn1
			if(ex_mem_regWrite1==1 && ex_mem_Rd1!=3'b000 && ex_mem_Rd1==id_ex_Rn1)
				Rn1_sel=2'b01;
			else if(mem_wb_regWrite2==1 && mem_wb_Rd2!=3'b000 && mem_wb_Rd2==id_ex_Rn1)
				Rn1_sel=2'b10;
			else if(mem_wb_regWrite1==1 && mem_wb_Rd1!=3'b000 && mem_wb_Rd1==id_ex_Rn1)
				Rn1_sel=2'b11;
			//for Rd1
			if(op12==0 && ex_mem_regWrite1==1 && ex_mem_Rd1!=3'b000 && ex_mem_Rd1==id_ex_Rd1)
				Rd1_sel=2'b01;
			else if(mem_wb_regWrite2==1 && mem_wb_Rd2!=3'b000 && op12==0 && mem_wb_Rd2==id_ex_Rd1)
				Rd1_sel=2'b10;
			else if(op12==0 && mem_wb_regWrite1==1 && mem_wb_Rd1!=3'b000 && mem_wb_Rd1==id_ex_Rd1)
				Rd1_sel=2'b11;
			//for Rn2
			if(ex_mem_regWrite1==1 && ex_mem_Rd1!=3'b000 && ex_mem_Rd1==id_ex_Rn2)
				Rn2_sel=2'b01;
			else if(mem_wb_regWrite2==1 && mem_wb_Rd2!=3'b000 && mem_wb_Rd2==id_ex_Rn2)
				Rn2_sel=2'b10;
			else if(mem_wb_regWrite1==1 && mem_wb_Rd1!=3'b000 && mem_wb_Rd1==id_ex_Rn2)
				Rn2_sel=2'b11;
			//ensure the regWrite signals are present in pipeline
			if(ex_mem_regWrite1==1 && ex_mem_Rd1!=3'b000 && ex_mem_Rd1==id_ex_Rd2)
				Rd2_sel=2'b01;
			else if(mem_wb_regWrite2==1 && mem_wb_Rd2!=3'b000 && mem_wb_Rd2==id_ex_Rd2)
				Rd2_sel=2'b10;
			else if(mem_wb_regWrite1==1 && mem_wb_Rd1!=3'b000 && mem_wb_Rd1==id_ex_Rd2)
				Rd2_sel=2'b11;
		end
endmodule
		
module vliwTestBench;
	reg clk;
	reg reset;
	wire [31:0] Result1, Result2;
	topmodule uut (.clk(clk), .reset(reset), .p3_Alu_set1(Result1), .p3_Mem_set2(Result2));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#10  reset=0;	
		
		#250 $finish; 
	end
endmodule
			
module flagsetmodule(input[3:0] p2_flag,input regwrite2, input[31:0] memOut, output reg [3:0] flag);
	always@(p2_flag,memOut,regwrite2)
		begin 
			flag=p2_flag;

			if(memOut==31'b0 && regwrite2)
				flag[2]=1;

		end

endmodule

module topmodule(input clk,input reset, output [31:0] p3_Alu_set1, output [31:0] p3_Mem_set2);
wire pcWrite;
wire[31:0] pcOut,pcMuxOut,pcPlus4,pcBranch,imOut,p0_pc,zero_ext_offset,branchAdder2,pcMinus4,immZext_set1,immZext_set2;
wire[31:0] epcOut, Set2_imm,p1_imm,p1_reg_rd,p1_reg_rm,p1_reg_rn,p1_reg_rd2,p1_reg_rn2,p1_offset,memAdderOut,Rn1_mux,Rm1_mux, Rd1_mux,aluOut,p2_aluOut,p2_adderOut,p2_reg_rd2;
//check out the pcException waala
//IMOut from IM
wire[1:0] pcSrc,aluSrc,p1_aluSrc,p2_aluSrc,Rn1_sel,Rm1_sel;
wire[2:0] p3_rd1,p3_rd2,p1_aluOp,aluOp,p2_rd1,p2_aluOp,p2_rd2,p1_rd2,p1_rd,p1_rn,p1_rm,p1_rn2 ;
wire if_id_write,p3_regWrite1,p3_regWrite2, cause, p1_causeWrite,memRead,memWrite, regWrite1,regWrite2,causeWrite,p1_memRead,p1_memWrite,p1_regWrite1,p1_regWrite2;
wire ifFlush,idFlush,exFlush,p2_memRead,p2_memWrite,p2_regWrite1,p2_regWrite2;
wire[3:0] flagIn,flagOut,p1_flag,p2_flag,p3_flag,flag1;
wire[27:0] jump_ext_offset,pcJump;
wire[31:0] memOut,alu_In2, regRd_set1, reg_rn_set1,reg_rm_set1,reg_rd_set2, reg_rn_set2, Rd2_mux, Rn2_mux;
wire ex_id_control,p1_op11,p1_op12;
wire[1:0] Rd1_sel,Rn2_sel,Rd2_sel;
wire[15:0] memZext,p0_set1,p0_set2;
wire [9:0] ctrlOut;
		register32bit_single_signal pc(clk, reset, pcWrite, 1'b1, pcMuxOut, pcOut);
		//instantiate IM
		adder_32bit pc_adder1(pcOut,32'b00000000000000000000000000000100,pcPlus4); 
		mux_4_to_1 pcMux(pcPlus4,{p0_pc[31:28],pcJump},pcBranch,32'b11111111_11111111_11111111_111111_00,pcSrc,pcMuxOut); //pending is pcException
		
    Mem IM(clk, reset,1'b0,1'b1, pcOut,32'b0,imOut);
		IF_ID p0(clk,reset|ifFlush,if_id_write,pcPlus4,imOut, p0_pc, p0_set1, p0_set2);
		
		
		sext11_to_28 sextJump(p0_set2[10:0],jump_ext_offset);
		shiftleft_2bit jummka(jump_ext_offset,pcJump);

		registerFile regFile(clk,reset,p3_regWrite1,p3_regWrite2,p0_set1[2:0], p0_set1[5:3],p0_set1[8:6], p0_set2[2:0],p0_set2[5:3],
		p3_rd1,p3_rd2,  p3_Alu_set1,p3_Mem_set2, regRd_set1, reg_rn_set1,reg_rm_set1,reg_rd_set2, reg_rn_set2 );
		
		flag nzcv(clk, reset, 1'b1, 1'b1, p3_flag,  flagOut);

		sext8_to_32 sextBranch(p0_set2[7:0],zero_ext_offset);
		shiftleft_2bit_32 shiftBranch(zero_ext_offset,branchAdder2);
		adder_32bit pc_adder2(branchAdder2,p0_pc,pcBranch); 	

	
		subtract_32bit pc_subtractor(p0_pc,32'b00000000000000000000000000000100,pcMinus4); 

		zext5_to_32 set1_imm(p0_set1[10:6],immZext_set1);
		zext5_to_32 set2_imm(p0_set2[10:6],immZext_set2);

		ctrlCkt control(p0_set1[15:11],p0_set2[15:11], flagIn[3], flagIn[0],p0_set1[10:9],aluOp,aluSrc,regWrite2,memRead,  memWrite,  pcSrc, regWrite1, exFlush, ifFlush,  idFlush,causeWrite);
		
		//check working
		mux_10bit_2to1 toIdex( {aluOp,aluSrc,regWrite2,memRead,  memWrite,   regWrite1,causeWrite},{10{1'b0}},ex_id_control,ctrlOut);

		shiftleft_1bit shift_set2_imm(immZext_set2,Set2_imm);
		//v- flag[0]
		register32bit_single_signal epc(clk, reset, p1_causeWrite | flagOut[0], 1'b1, pcMinus4, epcOut);
		register1bit cause_reg(clk,reset,p1_causeWrite | flagOut[0],1'b1,flagOut[0],cause);
	
		hazard_detection_unit haz(p0_set2[15:11], p1_memRead, p1_regWrite1,p1_rd2,p0_set1[5:3],p0_set1[8:6], p0_set1[2:0],p0_set2[5:3],  p0_set1[11],p0_set1[12], pcWrite, if_id_write, ex_id_control);

		//add ex_id_control here
		ID_EX p1(clk,reset|idFlush, immZext_set1, regRd_set1,reg_rm_set1, reg_rn_set1,reg_rd_set2,reg_rn_set2,p0_set1[2:0],p0_set1[8:6],p0_set1[5:3], ctrlOut[9:7],ctrlOut[6:5],ctrlOut[3],ctrlOut[2],ctrlOut[1],
	ctrlOut[4],p0_set2[2:0], p0_set2[5:3], Set2_imm, flagOut, ctrlOut[0],p0_set1[11],p0_set1[12],p1_imm, p1_reg_rd, p1_reg_rm, p1_reg_rn, p1_reg_rd2, p1_reg_rn2, p1_rd,  p1_rm, p1_rn, p1_causeWrite,	p1_rd2, p1_rn2, p1_offset, p1_flag, p1_aluOp, p1_aluSrc,  p1_memRead,p1_memWrite, p1_regWrite1, p1_regWrite2,p1_op11,p1_op12);
		
		adder_32bit mem_adder2(Rn2_mux,p1_offset,memAdderOut); 	
		
		//mux for aluIn1 i/p doubts
		//mux for aluIn2 i/p doubts 
		mux_4_to_1 forRn1(p1_reg_rn,p2_aluOut,p3_Mem_set2,p3_Alu_set1,Rn1_sel,Rn1_mux); //Rn1
		mux_4_to_1 forRm1(p1_reg_rm,p2_aluOut,p3_Mem_set2,p3_Alu_set1,Rm1_sel,Rm1_mux); //Rm1
		mux_4_to_1 forRd1(p1_reg_rd,p2_aluOut,p3_Mem_set2,p3_Alu_set1,Rd1_sel,Rd1_mux); //Rd1
		mux_4_to_1 forRd2(p1_reg_rd2,p2_aluOut,p3_Mem_set2,p3_Alu_set1,Rn2_sel,Rd2_mux); //Rd2
		mux_4_to_1 forRn2(p1_reg_rn2,p2_aluOut,p3_Mem_set2,p3_Alu_set1,Rd2_sel,Rn2_mux); //Rn2
		
		mux_4_to_1 forRmALu(Rm1_mux,p1_imm,Rd1_mux,{32{1'b0}},p1_aluSrc,alu_In2); //Rn2
		
		alu ALU(Rn1_mux, alu_In2, p1_flag, p1_aluOp,p1_regWrite1, aluOut, flagIn);	
	
		EX_MEM p2(clk, exFlush|reset, p1_rd,  aluOut, flagIn, memAdderOut, p1_rd2,p1_aluOp, p1_aluSrc, p1_memRead,p1_memWrite, p1_regWrite1, p1_regWrite2, Rd2_mux,
	p2_rd1, p2_aluOut, p2_adderOut, p2_rd2, p2_flag, p2_aluOp,  p2_aluSrc, p2_memRead, p2_memWrite, p2_regWrite1, p2_regWrite2, p2_reg_rd2);


		//DM
		Mem_16bit dm(clk, reset,p2_memWrite,p2_memRead, p2_adderOut,p2_reg_rd2[15:0],memZext);

		zext16_to_32 z0(memZext,memOut);


		flagsetmodule setFlag(p2_flag,p2_regWrite2,memOut, flag1);

		//modify p2_flag to take care of by adding a module
		MEM_WB p3(clk, reset, p2_rd1,  p2_aluOut,flag1, memOut, p2_rd2, p2_regWrite1, p2_regWrite2, p3_rd1, p3_Alu_set1, p3_flag, p3_Mem_set2, p3_rd2, p3_regWrite1, p3_regWrite2);
		
		

	//pass p0
		fwd f0(p1_rn,p1_rm,p1_rd,p1_rn2 ,p1_op11,p1_op12,p2_regWrite1,  p3_regWrite2, p3_regWrite1,p2_rd1,p3_rd1,p1_rd2, p2_rd2,p3_rd2, Rn1_sel, Rm1_sel,Rd1_sel,Rn2_sel,Rd2_sel);

		//make reset of if/id and id/ex as i and idFlush resp. 
		//pipelines numbered 0,1,2,3
		//if memOut_MSB is 1, negFlag 1
		//if memOut == 0, zeroFlag 1 - both in mem stage, before putting in pipeline 3
		//put neg flag from pipeline 3 to ctrl ckt
		//put overflow flag from pipeline 2 to ctrl ckt
		//00-normal, 01-ex_mem aluOut, 10-mem_wb memOut, 11-mem_wb aluOut FOR 5 Forwarding Muxes.
		
endmodule

module mux16to1( input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15, input [3:0] Sel, output reg [15:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15 or Sel)
	case (Sel)
				4'b0000: outBus=outR0;
				4'b0001: outBus=outR1;
				4'b0010: outBus=outR2;
				4'b0011: outBus=outR3;
				4'b0100: outBus=outR4;
				4'b0101: outBus=outR5;
				4'b0110: outBus=outR6;
				4'b0111: outBus=outR7;
				4'b1000: outBus=outR8;
				4'b1001: outBus=outR9;
				4'b1010: outBus=outR10;
				4'b1011: outBus=outR11;
				4'b1100: outBus=outR12;
				4'b1101: outBus=outR13;
				4'b1110: outBus=outR14;
				4'b1111: outBus=outR15;
	endcase
endmodule

module mux16to1_32bit( input [31:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15, input [3:0] Sel, output reg [31:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15 or Sel)
	case (Sel)
				4'b0000: outBus=outR0;
				4'b0001: outBus=outR1;
				4'b0010: outBus=outR2;
				4'b0011: outBus=outR3;
				4'b0100: outBus=outR4;
				4'b0101: outBus=outR5;
				4'b0110: outBus=outR6;
				4'b0111: outBus=outR7;
				4'b1000: outBus=outR8;
				4'b1001: outBus=outR9;
				4'b1010: outBus=outR10;
				4'b1011: outBus=outR11;
				4'b1100: outBus=outR12;
				4'b1101: outBus=outR13;
				4'b1110: outBus=outR14;
				4'b1111: outBus=outR15;
	endcase
endmodule
