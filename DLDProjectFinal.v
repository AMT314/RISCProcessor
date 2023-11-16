module alu(
    input [7:0] A, B, // ALU 8-bit Inputs
    input [3:0] ALU_Sel,
    output [15:0] ALU_Out, // ALU 16-bit Output
    output CarryOut
);
    reg [7:0] ALU_Result;
    reg [8:0] tmpAdd;
	 reg [8:0] tmpSub;
	 reg CarryBorrow;
	 reg [15:0] multiplyOP;
	 reg [7:0] multiX;
    assign ALU_Out[7:0] = ALU_Result; // ALU out
	 assign ALU_Out[15:8] = multiX ; //ALU out for multiplication
	 assign CarryOut = CarryBorrow; // Carry or borrow out
	 
	 integer i; // shift
	 
    always @ (*) begin
        case (ALU_Sel)
            4'b0000: begin
							ALU_Result = A + B;
							tmpAdd = {1'b0, A} + {1'b0, B};
							CarryBorrow = tmpAdd[8]; // Carryout flag
							end
            4'b0001: begin
							ALU_Result = A - B;
							tmpSub = {1'b0, A} - {1'b0, B};
							CarryBorrow = tmpSub[8]; // Carryout flag
							end
            4'b0010: begin
							multiplyOP = A * B;
							ALU_Result = multiplyOP[7:0];
							multiX = multiplyOP[15:8];
							end
            4'b0011: ALU_Result = A / B;
            4'b0100: ALU_Result = A << B;
            4'b0101: ALU_Result = A >> B;				
            4'b0110: begin
					case (B[2:0])
							3'b001: ALU_Result = {A[6:0], A[7]}; //right rotation by B[2:0] steps
							3'b010: ALU_Result = {A[5:0], A[7:6]};
							3'b011: ALU_Result = {A[4:0], A[7:5]};
							3'b100: ALU_Result = {A[3:0], A[7:4]};
							3'b101: ALU_Result = {A[2:0], A[7:3]};
							3'b110: ALU_Result = {A[1:0], A[7:2]};
							3'b111: ALU_Result = {A[0], A[7:1]};
					endcase
					end
            4'b0111: ALU_Result = {A[0], A[7:1]}; //left rotation by one step
            4'b1000: ALU_Result = A & B;
            4'b1001: ALU_Result = A | B;
            4'b1010: ALU_Result = A ^ B; // Logical xor
            4'b1011: ALU_Result = ~(A | B); // Logical nor
            4'b1100: ALU_Result = ~(A & B);
            4'b1101: ALU_Result = ~(A ^ B);
            4'b1110: ALU_Result = (A > B) ? 8'd1 : 8'd0; // Greater comparison
            4'b1111: ALU_Result = (A == B) ? 8'd1 : 8'd0; // Equal comparison
            /*default: begin
							ALU_Result = A + B;
							multiX = 0;
							end*/
				// carry for all operation
				// shifting for
        endcase
    end
endmodule



module data_memory (
    input [7:0] address, // 8-bit address
    input [7:0] write_data, // Data to write
    input write_enable, // Write enable signal
    output reg [7:0] read_data // Data read from memory
);
    reg [7:0] memory [255:0]; // 256 bytes of memory

    always @ (*) begin
        if (write_enable)
            memory[address] = write_data;
        read_data = memory[address];
    end
endmodule

module ControlUnit (
    input [7:0] opcode,
    output reg [3:0] ALUOp,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite
);

always @(opcode) begin
    case (opcode)
        8'b00000001: begin // ADD
            ALUOp = 4'b0000;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
        end
        8'b00000010: begin // SUB
            ALUOp = 4'b0001;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
        end
        8'b00000011: begin // AND
            ALUOp = 4'b1000;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
        end
        8'b00000100: begin // OR
            ALUOp = 4'b1001;
            RegWrite = 1'b1;
            MemRead = 1'b0;
            MemWrite = 1'b0;
        end
        8'b00000101: begin // LOAD
            ALUOp = 4'b0100;
            RegWrite = 1'b1;
            MemRead = 1'b1;
            MemWrite = 1'b0;
        end
        8'b00000110: begin // STORE
            ALUOp = 4'b0101;
            RegWrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b1;
        end
        default: begin // NOP or unknown instruction
            ALUOp = 4'b0000;
            RegWrite = 1'b0;
            MemRead = 1'b0;
            MemWrite = 1'b0;
        end
    endcase
end

endmodule

module RegisterFile (
    input [7:0] ReadRegister1, // Read register 1 address
    input [7:0] ReadRegister2, // Read register 2 address
    input [7:0] WriteRegister, // Write register address
    input [7:0] WriteData, // Data to write
	 input clk,
    input RegWrite, // Write enable signal
    output reg [7:0] ReadData1, // Data read from register 1
    output reg [7:0] ReadData2 // Data read from register 2
);
    reg [7:0] registers [31:0]; // 32 registers of 8 bits each

    always @ (*) begin
        // Read data from the registers
        ReadData1 = registers[ReadRegister1];
        ReadData2 = registers[ReadRegister2];
    end

    always @(posedge clk) begin
        // Write data to the register if the write enable signal is high
        if (RegWrite)
            registers[WriteRegister] = WriteData;
    end
endmodule

module DLDProjectFinal (
    input [7:0] opcode, // Opcode for the instruction
    output [15:0] ALU_Out, // Output from the ALU
    output CarryOut, // Carry out from the ALU
    input [7:0] DM_address, // Address for data memory
    input [7:0] DM_write_data, // Data to write to data memory
    output [7:0] DM_read_data // Data read from data memory
);

	 wire [3:0] ALU_Sel; // ALU operation selector
    wire [7:0] RegData1, RegData2; // Data read from the register file
    reg [7:0] ReadReg1, ReadReg2, WriteReg; // Register addresses
	 wire DM_write_enable; // Write enable for data memory
    reg [7:0] WriteData; // Data to write to the register file

    // Instantiate the control unit
    ControlUnit u0 (
        .opcode(opcode), 
        .ALUOp(ALU_Sel), 
        .RegWrite(DM_write_enable)
    );

    // Instantiate the register file
    RegisterFile u1 (
        .ReadRegister1(ReadReg1), 
        .ReadRegister2(ReadReg2), 
        .WriteRegister(WriteReg), 
        .WriteData(WriteData), 
        .RegWrite(DM_write_enable), 
        .ReadData1(RegData1), 
        .ReadData2(RegData2)
    );

    // Instantiate the ALU
    alu u2 (
        .A(RegData1), 
        .B(RegData2), 
        .ALU_Sel(ALU_Sel), 
        .ALU_Out(ALU_Out), 
        .CarryOut(CarryOut)
    );

    // Instantiate the data memory
    data_memory u3 (
        .address(DM_address), 
        .write_data(DM_write_data), 
        .write_enable(DM_write_enable), 
        .read_data(DM_read_data)
    );
endmodule
