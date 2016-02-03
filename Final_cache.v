//D flip flop
module D_FF(input clk, input reset, input write, input d, output reg q);
  always @(posedge clk) 
  if(reset) q=0;
  else
   if(write) q=d;
   endmodule

//3x8 decoder for index bits
module decoder3to8(input [2:0]select, output reg[7:0]out);
	always @(select)
	begin
		case(select)
			3'b000: out = 8'b00000001;
			3'b001: out = 8'b00000010;
			3'b010: out = 8'b00000100;
			3'b011: out = 8'b00001000;
			3'b100: out = 8'b00010000;
			3'b101: out = 8'b00100000;
			3'b110: out = 8'b01000000;
			3'b111: out = 8'b10000000;
		endcase
	end
endmodule

//array of dirty bits
module dirtyArray(input clk, input reset, input we,input miss,input write,input location1,input location2, input dirtyBit, output dirtyOut0, output dirtyOut1);
    D_FF d0(clk,reset, we & (miss|write) & ((miss & ~location1) | (write & ~location2)),dirtyBit,dirtyOut0);
    D_FF d1(clk,reset, we & (miss|write) & ((miss & location1) | (write & location2)),dirtyBit,dirtyOut1);
endmodule 

//array of valid bits
module validArray(input clk, input reset, input we, input sel, input validBit, output validOut0, output validOut1);
    D_FF d2(clk,reset,we && ~sel,validBit,validOut0);
    D_FF d3(clk,reset,we && sel,validBit,validOut1);
endmodule

//array of FIFO bits
module fifoArray(input clk, input reset, input we, input fifoBit, output fifoOut);
    D_FF d1(clk,reset,we,fifoBit,fifoOut);
endmodule

module mruArray(input clk, input reset, input we, input mruBit, output mruOut);
    D_FF d1(clk,reset,we,mruBit,mruOut);
endmodule

//24 bit tag
module tagBlock(input clk, input reset, input write, input [23:0] tag ,output [23:0] tagData);
    D_FF d[23:0](clk,reset, write, tag, tagData);
 endmodule
 
 //array of tag blocks
 module tagArray(input clk, input reset, input we,input miss,input write,input location1,input location2, input [23:0] tagIn, output [23:0] tagOut0, output [23:0] tagOut1);
    tagBlock tb0(clk,reset, we & (miss|write) & ((miss & ~location1) | (write & ~location2)), tagIn, tagOut0);
    tagBlock tb1(clk,reset, we & (miss|write) & ((miss & location1) | (write & location2)), tagIn, tagOut1);
endmodule

//Each byte has 8 data bits
module dataByte(input clk, input reset, input writeEnable, input [7:0] in,output [7:0] out);    
    D_FF d[7:0](clk,reset, writeEnable, in, out);
endmodule

//32 byte data line
module dataBlock(input clk, input reset, input write, input[7:0] in0, input[7:0] in1, input[7:0] in2, input[7:0] in3,
input[7:0] in4, input[7:0] in5, input[7:0] in6, input[7:0] in7,  input[7:0] in8, input[7:0] in9, input[7:0] in10, input[7:0] in11,
input[7:0] in12, input[7:0] in13, input[7:0] in14, input[7:0] in15, input[7:0] in16, input[7:0] in17, input[7:0] in18, input[7:0] in19,
input[7:0] in20, input[7:0] in21, input[7:0] in22, input[7:0] in23,  input[7:0] in24, input[7:0] in25, input[7:0] in26, input[7:0] in27,
input[7:0] in28, input[7:0] in29, input[7:0] in30, input[7:0] in31,
output[7:0] out0, output[7:0] out1, output[7:0] out2, output[7:0] out3, 
output[7:0] out4, output[7:0] out5, output[7:0] out6, output[7:0] out7, output[7:0] out8, output[7:0] out9, output[7:0] out10, output[7:0] out11, 
output[7:0] out12, output[7:0] out13, output[7:0] out14, output[7:0] out15, output[7:0] out16, output[7:0] out17, output[7:0] out18, output[7:0] out19, 
output[7:0] out20, output[7:0] out21, output[7:0] out22, output[7:0] out23, output[7:0] out24, output[7:0] out25, output[7:0] out26, output[7:0] out27, 
output[7:0] out28, output[7:0] out29, output[7:0] out30, output[7:0] out31);
dataByte db0(clk,reset,write, in0, out0);
dataByte db1(clk,reset,write, in1, out1);
dataByte db2(clk,reset,write, in2, out2);
dataByte db3(clk,reset,write, in3, out3);
dataByte db4(clk,reset,write, in4, out4);
dataByte db5(clk,reset,write, in5, out5);
dataByte db6(clk,reset,write, in6, out6);
dataByte db7(clk,reset,write, in7, out7);
dataByte db8(clk,reset,write, in8, out8);
dataByte db9(clk,reset,write, in9, out9);
dataByte db10(clk,reset,write, in10, out10);
dataByte db11(clk,reset,write, in11, out11);
dataByte db12(clk,reset,write, in12, out12);
dataByte db13(clk,reset,write, in13, out13);
dataByte db14(clk,reset,write, in14, out14);
dataByte db15(clk,reset,write, in15, out15);
dataByte db16(clk,reset,write, in16, out16);
dataByte db17(clk,reset,write, in17, out17);
dataByte db18(clk,reset,write, in18, out18);
dataByte db19(clk,reset,write, in19, out19);
dataByte db20(clk,reset,write, in20, out20);
dataByte db21(clk,reset,write, in21, out21);
dataByte db22(clk,reset,write, in22, out22);
dataByte db23(clk,reset,write, in23, out23);
dataByte db24(clk,reset,write, in24, out24);
dataByte db25(clk,reset,write, in25, out25);
dataByte db26(clk,reset,write, in26, out26);
dataByte db27(clk,reset,write, in27, out27);
dataByte db28(clk,reset,write, in28, out28);
dataByte db29(clk,reset,write, in29, out29);
dataByte db30(clk,reset,write, in30, out30);
dataByte db31(clk,reset,write, in31, out31);
endmodule

//array of data blocks
module dataArray(input clk, input reset, input writeEnable,input miss,input write,input location1,input location2, input[255:0] in_dataBlock, 
                 output[255:0] out_dataBlock0, output[255:0] out_dataBlock1);

 dataBlock db0( clk,  reset,  writeEnable & (miss|write) & ((miss & ~location1) | (write & ~location2)),  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 in_dataBlock[135:128],  in_dataBlock[143:136] ,  in_dataBlock[151:144],  in_dataBlock[159:152],
 in_dataBlock[167:160],  in_dataBlock[175:168],  in_dataBlock[183:176],  in_dataBlock[191:184], in_dataBlock[199:192], in_dataBlock[207:200], 
 in_dataBlock[215:208], in_dataBlock[223:216], in_dataBlock[231:224], in_dataBlock[239:232], in_dataBlock[247:240], in_dataBlock[255:248],
 out_dataBlock0[7:0],  out_dataBlock0[15:8] ,  out_dataBlock0[23:16],  out_dataBlock0[31:24], 
 out_dataBlock0[39:32], out_dataBlock0[47:40],  out_dataBlock0[55:48],  out_dataBlock0[63:56], out_dataBlock0[71:64], out_dataBlock0[79:72], 
 out_dataBlock0[87:80], out_dataBlock0[95:88], out_dataBlock0[103:96], out_dataBlock0[111:104], out_dataBlock0[119:112], out_dataBlock0[127:120],
 out_dataBlock0[135:128],  out_dataBlock0[143:136] ,  out_dataBlock0[151:144],  out_dataBlock0[159:152],
 out_dataBlock0[167:160],  out_dataBlock0[175:168],  out_dataBlock0[183:176],  out_dataBlock0[191:184], out_dataBlock0[199:192], out_dataBlock0[207:200], 
 out_dataBlock0[215:208], out_dataBlock0[223:216], out_dataBlock0[231:224], out_dataBlock0[239:232], out_dataBlock0[247:240], out_dataBlock0[255:248]); 
 
 dataBlock db1( clk,  reset,  writeEnable & (miss|write) & ((miss & location1) | (write & location2)) ,  
 in_dataBlock[7:0],  in_dataBlock[15:8] ,  in_dataBlock[23:16],  in_dataBlock[31:24],
 in_dataBlock[39:32],  in_dataBlock[47:40],  in_dataBlock[55:48],  in_dataBlock[63:56], in_dataBlock[71:64], in_dataBlock[79:72], 
 in_dataBlock[87:80], in_dataBlock[95:88], in_dataBlock[103:96], in_dataBlock[111:104], in_dataBlock[119:112], in_dataBlock[127:120],
 in_dataBlock[135:128],  in_dataBlock[143:136] ,  in_dataBlock[151:144],  in_dataBlock[159:152],
 in_dataBlock[167:160],  in_dataBlock[175:168],  in_dataBlock[183:176],  in_dataBlock[191:184], in_dataBlock[199:192], in_dataBlock[207:200], 
 in_dataBlock[215:208], in_dataBlock[223:216], in_dataBlock[231:224], in_dataBlock[239:232], in_dataBlock[247:240], in_dataBlock[255:248],
 out_dataBlock1[7:0],  out_dataBlock1[15:8] ,  out_dataBlock1[23:16],  out_dataBlock1[31:24], 
 out_dataBlock1[39:32], out_dataBlock1[47:40],  out_dataBlock1[55:48],  out_dataBlock1[63:56], out_dataBlock1[71:64], out_dataBlock1[79:72], 
 out_dataBlock1[87:80], out_dataBlock1[95:88], out_dataBlock1[103:96], out_dataBlock1[111:104], out_dataBlock1[119:112], out_dataBlock1[127:120],
 out_dataBlock1[135:128], out_dataBlock1[143:136], out_dataBlock1[151:144], out_dataBlock1[159:152],
 out_dataBlock1[167:160], out_dataBlock1[175:168], out_dataBlock1[183:176], out_dataBlock1[191:184], out_dataBlock1[199:192], out_dataBlock1[207:200], 
 out_dataBlock1[215:208], out_dataBlock1[223:216], out_dataBlock1[231:224], out_dataBlock1[239:232], out_dataBlock1[247:240], out_dataBlock1[255:248]);
endmodule

module cache(input clk,input reset,input cacheWrite,input [31:0]address,input [255:0] in_dataBlock,output [7:0]cacheOutput);
	
	wire [255:0]out_dataBlock0_0,out_dataBlock0_1,out_dataBlock0_2,out_dataBlock0_3,out_dataBlock0_4,out_dataBlock0_5,out_dataBlock0_6,out_dataBlock0_7,out_dataBlock1_0,out_dataBlock1_1,out_dataBlock1_2,out_dataBlock1_3,out_dataBlock1_4,out_dataBlock1_5,out_dataBlock1_6,out_dataBlock1_7,data_block_mux0,data_block_mux1,muxOutData;
	wire [23:0] tagOut0_0,tagOut0_1,tagOut0_2,tagOut0_3,tagOut0_4,tagOut0_5,tagOut0_6,tagOut0_7,tagOut1_0,tagOut1_1,tagOut1_2,tagOut1_3,tagOut1_4,tagOut1_5,tagOut1_6,tagOut1_7,tag_block_mux0,tag_block_mux1;
	wire comp_tag0out,comp_tag1out,fifoBitOut,mruBitOut,dirtyBit,hitOrMiss,valid,validBitOut;
	wire [7:0]fifoSelect,fifo,MRU,validOut0,validOut1,dirtyOut0,dirtyOut1;
	
	//cacheControl cacheCtrl(valid,cacheWrite1, fifo[address[7:5]], MRU[address[7:5]],comp_tag0out,comp_tag1out, hitOrMiss,cacheWrite, fifoBitOut, mruBitOut, validBitOut);
	decoder3to8 dec(address[7:5],fifoSelect);
	
	fifoArray f1(clk, reset,fifoSelect[0] && ~hitOrMiss, fifoBitOut,fifo[0]);
	fifoArray f2(clk, reset,fifoSelect[1] && ~hitOrMiss, fifoBitOut,fifo[1]);
	fifoArray f3(clk, reset,fifoSelect[2] && ~hitOrMiss, fifoBitOut,fifo[2]);
	fifoArray f4(clk, reset,fifoSelect[3] && ~hitOrMiss, fifoBitOut,fifo[3]);
	fifoArray f5(clk, reset,fifoSelect[4] && ~hitOrMiss, fifoBitOut,fifo[4]);
	fifoArray f6(clk, reset,fifoSelect[5] && ~hitOrMiss, fifoBitOut,fifo[5]);
	fifoArray f7(clk, reset,fifoSelect[6] && ~hitOrMiss, fifoBitOut,fifo[6]);
	fifoArray f8(clk, reset,fifoSelect[7] && ~hitOrMiss, fifoBitOut,fifo[7]);
	
	mruArray m1(clk, reset,fifoSelect[0], mruBitOut,MRU[0]);
	mruArray m2(clk, reset,fifoSelect[1], mruBitOut,MRU[1]);
	mruArray m3(clk, reset,fifoSelect[2], mruBitOut,MRU[2]);
	mruArray m4(clk, reset,fifoSelect[3], mruBitOut,MRU[3]);
	mruArray m5(clk, reset,fifoSelect[4], mruBitOut,MRU[4]);
	mruArray m6(clk, reset,fifoSelect[5], mruBitOut,MRU[5]);
	mruArray m7(clk, reset,fifoSelect[6], mruBitOut,MRU[6]);
	mruArray m8(clk, reset,fifoSelect[7], mruBitOut,MRU[7]);
	
	dirtyArray dArr1(clk,  reset, fifoSelect[0], ~hitOrMiss,(hitOrMiss && cacheWrite),  fifo[0],mruBitOut, dirtyBit,  dirtyOut0[0],  dirtyOut1[0]);
	dirtyArray dArr2(clk,  reset, fifoSelect[1], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[1], mruBitOut, dirtyBit,  dirtyOut0[1],  dirtyOut1[1]);
	dirtyArray dArr3( clk,  reset,fifoSelect[2], ~hitOrMiss,(hitOrMiss && cacheWrite),  fifo[2], mruBitOut, dirtyBit,  dirtyOut0[2],  dirtyOut1[2]);
	dirtyArray dArr4( clk,  reset,fifoSelect[3], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[3], mruBitOut, dirtyBit,  dirtyOut0[3],  dirtyOut1[3]);
	dirtyArray dArr5( clk,  reset,fifoSelect[4], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[4],mruBitOut,  dirtyBit,  dirtyOut0[4],  dirtyOut1[4]);
	dirtyArray dArr6( clk,  reset,fifoSelect[5], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[5],mruBitOut,  dirtyBit,  dirtyOut0[5],  dirtyOut1[5]);
	dirtyArray dArr7( clk,  reset,fifoSelect[6], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[6], mruBitOut, dirtyBit,  dirtyOut0[6],  dirtyOut1[6]);
	dirtyArray dArr8( clk,  reset,fifoSelect[7], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[7], mruBitOut, dirtyBit,  dirtyOut0[7],  dirtyOut1[7]);
	
	validArray v1(clk,  reset, ~hitOrMiss && fifoSelect[0], fifo[0], validBitOut,validOut0[0],  validOut1[0]);
	validArray v2(clk,  reset, ~hitOrMiss && fifoSelect[1], fifo[1], validBitOut,validOut0[1],  validOut1[1]);
	validArray v3(clk,  reset, ~hitOrMiss && fifoSelect[2], fifo[2], validBitOut,validOut0[2],  validOut1[2]);
	validArray v4(clk,  reset, ~hitOrMiss && fifoSelect[3], fifo[3], validBitOut,validOut0[3],  validOut1[3]);
	validArray v5(clk,  reset, ~hitOrMiss && fifoSelect[4], fifo[4], validBitOut,validOut0[4],  validOut1[4]);
	validArray v6(clk,  reset, ~hitOrMiss && fifoSelect[5], fifo[5], validBitOut,validOut0[5],  validOut1[5]);
	validArray v7(clk,  reset, ~hitOrMiss && fifoSelect[6], fifo[6], validBitOut,validOut0[6],  validOut1[6]);
	validArray v8(clk,  reset, ~hitOrMiss && fifoSelect[7], fifo[7], validBitOut,validOut0[7],  validOut1[7]);
	
	tagArray ta1(clk,  reset,  fifoSelect[0], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[0],mruBitOut,  address[31:8],  tagOut0_0,  tagOut1_0);
	tagArray ta2(clk,  reset,  fifoSelect[1], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[1],mruBitOut,  address[31:8],  tagOut0_1,   tagOut1_1);
	tagArray ta3(clk,  reset,  fifoSelect[2], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[2],mruBitOut,  address[31:8],  tagOut0_2,   tagOut1_2);
	tagArray ta4(clk,  reset,  fifoSelect[3], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[3],mruBitOut,  address[31:8],  tagOut0_3,   tagOut1_3);
	tagArray ta5(clk,  reset,  fifoSelect[4], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[4],mruBitOut,  address[31:8],  tagOut0_4,   tagOut1_4);
	tagArray ta6(clk,  reset,  fifoSelect[5], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[5],mruBitOut,  address[31:8],  tagOut0_5,   tagOut1_5);
	tagArray ta7(clk,  reset,  fifoSelect[6], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[6],mruBitOut,  address[31:8],  tagOut0_6,   tagOut1_6);
	tagArray ta8(clk,  reset,  fifoSelect[7], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[7],mruBitOut,  address[31:8],  tagOut0_7,   tagOut1_7);
	
	dataArray da1( clk,  reset,  fifoSelect[0], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[0],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_0,  out_dataBlock1_0);
	dataArray da2( clk,  reset,  fifoSelect[1], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[1],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_1,  out_dataBlock1_1);
	dataArray da3( clk,  reset,  fifoSelect[2], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[2],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_2,  out_dataBlock1_2);
	dataArray da4( clk,  reset,  fifoSelect[3], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[3],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_3,  out_dataBlock1_3);
	dataArray da5( clk,  reset,  fifoSelect[4], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[4],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_4,  out_dataBlock1_4);
	dataArray da6( clk,  reset,  fifoSelect[5], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[5],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_5,  out_dataBlock1_5);
	dataArray da7( clk,  reset,  fifoSelect[6], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[6],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_6,  out_dataBlock1_6);
	dataArray da8( clk,  reset,  fifoSelect[7], ~hitOrMiss,(hitOrMiss && cacheWrite), fifo[7],mruBitOut,  in_dataBlock, 
                  out_dataBlock0_7,  out_dataBlock1_7);
				  
	fifoReplacement fr(hitOrMiss,fifo[address[7:5]],fifoBitOut);
	
  //Logic to select data array
	mux_8_to_1_256bit mux0(out_dataBlock0_0,out_dataBlock0_1,out_dataBlock0_2,out_dataBlock0_3,out_dataBlock0_4,out_dataBlock0_5,out_dataBlock0_6,out_dataBlock0_7,address[7:5],data_block_mux0); 
	mux_8_to_1_256bit mux1(out_dataBlock1_0,out_dataBlock1_1,out_dataBlock1_2,out_dataBlock1_3,out_dataBlock1_4,out_dataBlock1_5,out_dataBlock1_6,out_dataBlock1_7,address[7:5],data_block_mux1); 
  //Logic to select tag array
	mux_8_to_1_24bit mux0_24bit(tagOut0_0,tagOut0_1,tagOut0_2,tagOut0_3,tagOut0_4,tagOut0_5,tagOut0_6,tagOut0_7,address[7:5],tag_block_mux0); 
	mux_8_to_1_24bit mux1_24bit(tagOut1_0,tagOut1_1,tagOut1_2,tagOut1_3,tagOut1_4,tagOut1_5,tagOut1_6,tagOut1_7,address[7:5],tag_block_mux1); 
	//Tag comparison
	comparator_24bit cmp1(MRU[address[7:5]],address[31:8],tag_block_mux0,tag_block_mux1,validOut0[address[7:5]],validOut1[address[7:5]],mruBitOut,hitOrMiss,validBitOut);
  mux_2_to_1 m2to1(data_block_mux0,data_block_mux1,in_dataBlock,cacheWrite,dirtyOut0[address[7:5]],dirtyOut1[address[7:5]],mruBitOut,fifoBitOut,hitOrMiss,muxOutData);
	mux_32_to_1 m32to1(muxOutData,address[4:0],cacheOutput);
	dirtyReplacement dR(hitOrMiss,cacheWrite,mruBitOut,dirtyOut0[address[7:5]],dirtyOut1[address[7:5]],dirtyBit);
	//comparator_24bit cmp2(MRU[address[7:5]],address[31:8],tag_block_mux1,comp_tag1out);
	//To select the valid bit from two blocks
	//mux_2_to_1_1bit m2to1_1bit(validOut0[address[7:5]],validOut1[address[7:5]],fifo[address[7:5]],valid);
	//mux_2_to_1 m2to1(input [31:0] line1,input[31:0] line2,input select,output reg [31:0] muxOut); 
  //if comp_tag0out==0 and comp_tag1out==0 then CACHE miss => do replacement
	//hit h(comp_tag0out,comp_tag1out,valid,hitOrMiss);
	//Encoder not needed in way prediction
	//encoder e(comp_tag0out,comp_tag1out,comp_tag1out || comp_tag0out,encoderOut);
	 
endmodule


module cacheTestBench;
	reg clk;
	reg reset,cacheWrite;
	reg [255:0] datblock;
	wire [7:0] cacheout;
	reg [31:0]add;
	//cache uut(.clk,input cacheRead,input cacheWrite,input [31:0]address,input [255:0] in_dataBlock,output [7:0]cacheOutput);
	cache uut (.clk(clk), .reset(reset), .cacheWrite(cacheWrite), .address(add),.in_dataBlock(datblock),.cacheOutput(cacheout));

	always
	begin
	#5 clk=~clk;
	end
	initial
	begin
		clk=0; reset=1; cacheWrite=1'b0;
		#10  reset=0;	
		#50 add=32'b000_0000_0000_0000_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b0001};
		#20 add=32'b000_0001_0000_0000_0000_0000_0001_00000; datblock={4'b0000,{248{1'b0}},4'b0010};
		#20 add=32'b000_1000_0000_0000_0000_0000_0001_00000; datblock={4'b0000,{248{1'b0}},4'b0011}; 
		#20 add=32'b010_0100_0000_0000_0000_0000_0010_00000; datblock={4'b0000,{248{1'b0}},4'b0100};//cacheWrite=1'b1; 
		#20 add=32'b011_0000_0000_0011_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b0101};//cacheWrite=1'b0;
		#20 add=32'b000_0000_0000_1110_0000_0000_0001_00000; datblock={4'b0000,{248{1'b0}},4'b0110};
		#20 add=32'b000_0000_1111_0000_0000_0000_0010_00000; datblock={4'b0000,{248{1'b0}},4'b0111}; 	
		#20 add=32'b000_0000_0000_0000_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b1000}; cacheWrite=1'b1;
		#20 add=32'b011_0000_0011_0000_0000_0000_0010_00000; datblock={4'b0000,{248{1'b0}},4'b1001}; cacheWrite=1'b0;
		#20 add=32'b000_0000_0000_0000_0000_0001_0001_00000; datblock={4'b0000,{248{1'b0}},4'b1010};
		#20 add=32'b000_0000_0111_0000_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b1011}; cacheWrite=1'b1;//cache miss
		#20	datblock={4'b0000,{248{1'b0}},4'b1111};
		#20 add=32'b010_0000_0000_0000_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b1100}; cacheWrite=1'b0;
		#20 add=32'b011_0000_0000_0000_0000_0000_0000_00000; datblock={4'b0000,{248{1'b0}},4'b1101}; cacheWrite=1'b0;
		#250 $finish; 
	end
endmodule

module fifoReplacement(input hitOrMiss,input fifo,output reg fifoBitOut);
always@(hitOrMiss or fifo)
	begin
		if(hitOrMiss==0)
			fifoBitOut=~fifo;
		else
			fifoBitOut=fifo;
	end
endmodule

module dirtyReplacement(input hitOrMiss,input cacheWrite,input mru,input dirty1,input dirty2,output reg dirtyBitOut);
always@(hitOrMiss or cacheWrite or mru or dirty1 or dirty2)
	begin
		if(cacheWrite)
			dirtyBitOut=1'b1;
		else if(~hitOrMiss)
			dirtyBitOut=1'b0;
		else
		begin
		case(mru)
		1'b0:	dirtyBitOut=dirty1;
		1'b1:	dirtyBitOut=dirty2;
		endcase
		end
	end
endmodule


module mux_32_to_1(input [255:0] line1,input [4:0] select,output reg [7:0] muxOut); 
   always @ (line1,select) begin
     case(select)
       5'b00000 : muxOut = line1[7:0];
       5'b00001 : muxOut = line1[15:8];
       5'b00010 : muxOut = line1[23:16];
       5'b00011 : muxOut = line1[31:24];
       5'b00100 : muxOut = line1[39:32];
       5'b00101 : muxOut = line1[47:40];
       5'b00110 : muxOut = line1[55:48];
       5'b00111 : muxOut = line1[63:56];
       5'b01000 : muxOut = line1[71:64];
       5'b01001 : muxOut = line1[79:72];
       5'b01010 : muxOut = line1[87:80];
       5'b01011 : muxOut = line1[95:88];
       5'b01100 : muxOut = line1[103:96];
       5'b01101 : muxOut = line1[111:104];
       5'b01110 : muxOut = line1[119:112];
       5'b01111 : muxOut = line1[127:120];
       5'b10000 : muxOut = line1[135:128];
       5'b10001 : muxOut = line1[143:136];
       5'b10010 : muxOut = line1[151:144];
       5'b10011 : muxOut = line1[159:152];
       5'b10100 : muxOut = line1[167:160];
       5'b10101 : muxOut = line1[175:168];
       5'b10110 : muxOut = line1[183:176];
       5'b10111 : muxOut = line1[191:184];
       5'b11000 : muxOut = line1[199:192];
       5'b11001 : muxOut = line1[207:200];
       5'b11010 : muxOut = line1[215:208];
       5'b11011 : muxOut = line1[223:216];
       5'b11100 : muxOut = line1[231:224];
       5'b11101 : muxOut = line1[239:232];
       5'b11110 : muxOut = line1[247:240];
       5'b11111 : muxOut = line1[255:248]; 
     endcase
   end
 endmodule

 module mux_2_to_1(input [255:0] line1,input[255:0] line2,input[255:0] data, input cacheWrite, input dirty1,input dirty2,input select,input fifo, 
 input hitOrMiss,output reg [255:0] muxOut); 
   always @ (line1,line2,select,hitOrMiss,dirty1,dirty2,fifo, data, cacheWrite) begin
   if(hitOrMiss)
	begin
	  if(cacheWrite)
	    muxOut = data;
	   else
	     begin
     case(select)
       1'b0 : muxOut = line1;
       1'b1 : muxOut = line2;
     endcase
   end
	 end
	 else
	 begin 
		if((dirty1==1 && fifo == 1)||(dirty2==1 && fifo == 0))
		begin
			case(~fifo)
					1'b0 : muxOut = line1;
					1'b1 : muxOut = line2;
				endcase
		end
		else
		muxOut=256'b0;
	 end
 end
 endmodule

module comparator_24bit(input mru,input [23:0]address,input[23:0] tag1,input [23:0] tag2,input valid1,input valid2,output reg mruBitOut,output reg hitOrMiss,output reg valid);

	always @(tag1 or tag2 or address or mru or valid1 or valid2)
	begin
	  if(mru==0)
	    begin
		if (tag1==address && valid1==1)
		begin
			 hitOrMiss=1'b1;
			 mruBitOut=1'b0;
			 end
		else if(tag2==address && valid2==1) begin
			 hitOrMiss=1'b1;
			 mruBitOut=1'b1;

			 end
		else begin
			hitOrMiss=1'b0;
		mruBitOut=mru; 
		valid=1'b1;
		end
		end
	  else
	    begin
		if (tag2==address && valid2==1)
		begin
			 hitOrMiss=1'b1;
			 mruBitOut=1'b1;
			end
		else if(tag1==address && valid2==1) begin
			 hitOrMiss=1'b1;
			 mruBitOut=1'b0;
			 end
		else begin
			hitOrMiss=1'b0;
		mruBitOut=mru;
		valid=1'b1;
		end
		end
	end	

endmodule

module mux_8_to_1_256bit(input [255:0] line1,input[255:0] line2,input[255:0] line3,input[255:0] line4,input[255:0] line5,input[255:0] line6,input[255:0] line7,input[255:0] line8,input [2:0] select,output reg [255:0] muxOut); 
   always @ (line1,line2,line3,line4,line5,line6,line7,line8,select) begin
      begin
     case(select)
       3'b000 : muxOut = line1;
       3'b001 : muxOut = line2;
       3'b010 : muxOut = line3;
       3'b011 : muxOut = line4;
	   3'b100 : muxOut = line5;
	   3'b101 : muxOut = line6;
	   3'b110 : muxOut = line7;
	   3'b111 : muxOut = line8;
     endcase
   end
   end
 endmodule
 
 module mux_8_to_1_24bit(input [23:0] line1,input[23:0] line2,input[23:0] line3,input[23:0] line4,input[23:0] line5,input[23:0] line6,input[23:0] line7,input[23:0] line8,input [2:0] select,output reg [23:0] muxOut); 
   always @ (line1,line2,line3,line4,line5,line6,line7,line8,select) begin
       begin
     case(select)
       3'b000 : muxOut = line1;
       3'b001 : muxOut = line2;
       3'b010 : muxOut = line3;
       3'b011 : muxOut = line4;
	   3'b100 : muxOut = line5;
	   3'b101 : muxOut = line6;
	   3'b110 : muxOut = line7;
	   3'b111 : muxOut = line8;
     endcase
   end
   end
 endmodule
