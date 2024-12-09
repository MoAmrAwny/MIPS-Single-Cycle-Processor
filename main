// Code your design here
// Code your design here
// MIPS Processor Verilog Code

module pc(lastpc,nextpc,jump,clk);
  
    input lastpc,jump,clk; // wire 1 bit
    output nextpc;
    wire [31:0] lastpc;
    reg [31:0] nextpc;
    wire [31:0] jump_inst;
    always@(posedge clk)
      begin
        if(jump) nextpc <= jump_inst;
    	else nextpc <= lastpc+4;
      end
endmodule
    
module instruction_mem(
  input wire [31:0] read_add,
  input wire clk,
  output reg [31:0] instruction ) // 4kb instruction memory
  reg [31:0] mem [0:1023]; // to store the instructions 1024 words of a 32-bit instruction

   always @(posedge clk)
     begin
       instruction <= mem[ read_add >> 2 ]; // some say shift right by 2 to make it word aligned 
     end

endmodule


module control_unit(
  input wire[5:0] op_code,
  input clk,
  output reg [6:0] control_signals) // control unit with 5 instructions add,Lw,Sw,sub,branch
  
  always @(posedge clk) begin
        case (op_code)
            6'b000000: // ADD Instruction
                control_signals <= 7'b1000000; // RegWrite=1, ALUOp=00 (ADD), others=0
            6'b000001: // SUB Instruction
                control_signals <= 7'b1000010; // RegWrite=1, ALUOp=10 (SUB), others=0
            6'b100010: // LW Instruction
                control_signals <= 7'b1101000; // RegWrite=1, MemRead=1, ALUSrc=1
            6'b101011: // SW Instruction
                control_signals <= 7'b0011000; // MemWrite=1, ALUSrc=1
            6'b000100: // BRANCH Instruction
                control_signals <= 7'b0000100; // Branch=1
            default: // Default case for unsupported opcodes
                control_signals <= 7'b0000000; // No operation
        endcase
    end

endmodule
  

module register_file(
    input wire clk,                    // Clock signal
    input wire reg_write,              // Write enable signal
    input wire [4:0] read_reg1,        // Address of the first register to read
    input wire [4:0] read_reg2,        // Address of the second register to read
    input wire [4:0] write_reg,        // Address of the register to write
    input wire [31:0] write_data,      // Data to write into the register
    output reg [31:0] read_data1,      // Data read from the first register
    output reg [31:0] read_data2       // Data read from the second register
);

    // Define a 32x32 register array (32 registers, each 32 bits wide)
    reg [31:0] registers [31:0];

   
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1) begin
            registers[i] = 0;
        end
    end

    // Read operations (combinational logic)
    always @(*) begin
        read_data1 = registers[read_reg1];
        read_data2 = registers[read_reg2];
    end

   
    always @(posedge clk) begin
        if (reg_write && write_reg != 0) begin
            registers[write_reg] <= write_data;
        end
    end

endmodule


module sign_extend(
  input wire[15:0] notextended,
  input wire clk,
  output reg[31:0] extended);
  
  always @(posedge clk)
    begin
      extended <= { {16{notextended[15]}}, notextended }; // sign extend
    end
endmodule



module ALU(input wire[31:0] in_alu1,
           input wire [31:0] in_alu2,
           input wire [3:0] ALU_control,
           input wire clk,
           output reg [31:0] res_alu,
           output reg zero_flag);
  always @(posedge clk)
    begin 
      zero_flag=0;
      case(ALU_control)
        4'b0010: res_alu=in_alu1+in_alu2;
        4'b0110: 
          begin
            res_alu=in_alu1-in_alu2;
            if(res_alu==0) zero_flag=1; 
          end
        4'b0000: res_alu=in_alu1 & in_alu2;
        default: 
          res_alu = 32'b0; 
      endcase
    end
endmodule
        
        

module ALU_Control(
  input wire [1:0] ALU_OP,
  input wire [5:0] funct,
  output reg [3:0] ALU_control);
  
   
  always @(*) 
    begin
    case (ALU_OP)
            2'b00: // ADD Instruction
              ALU_control = 4'b0010; // ADD for lw, sw, and branches
            2'b01: // SUB Instruction
              ALU_control = 4'b0110; // SUB for branches (beq)
            2'b10:
              begin
                case(funct)
               		6'b100000: ALU_control = 4'b0010;  // ADD
                    6'b100010: ALU_control = 4'b0110;  // SUB
                    6'b100100: ALU_control = 4'b0000;  // AND
                    6'b100101: ALU_control = 4'b0001;  // OR
                    6'b101010: ALU_control = 4'b0111;  // SLT
                  default: ALU_control = 4'b0000;    // Default to AND (should handle other cases)
                endcase
                  
              end
            
            default: 
                ALU_control = 4'b0000; // No operation
        endcase
    end
endmodule



module Data_memory(
  input wire [31:0] add,
  input wire [31:0] write_data,
  input wire [6:0] mem_read,
  input wire clk,
  input wire [6:0] mem_write,
  output reg [31:0] read_data);
  
  reg [31:0] location [0:4095];
  
  initial 
    begin
      integer i;
      for( i=0 ; i<4096 ; i=i+1 )
        location[i] = 32'b0;
    end
  
  always @(posedge clk)
    begin
      if(mem_write == 1'b1) location [add] <= write_data;
      if(mem_read == 1'b1)  read_data <= location [add];
    end
endmodule
        
  


module mux_2_1(
    input wire [31:0] in0,  // Input 0
    input wire [31:0] in1,  // Input 1
    input wire sel,         // Select signal
    output reg [31:0] out   // Output
);

    // Always block to define behavior
    always @(*) begin
        case (sel)
            1'b0: out = in0; // Select input 0
            1'b1: out = in1; // Select input 1
            default: out = 32'b0; // Default case (optional)
        endcase
    end
endmodule





module MIPs_Processor(
    input wire clk,          
    input wire reset,        
    output wire [31:0] out   
);

    
    reg [31:0] lastpc;          
    wire [31:0] nextpc;        
    reg jump;                  
    pc pc1(
        .lastpc(lastpc),   
        .nextpc(nextpc),   
        .jump(jump),       
        .clk(clk)
    );

    
    wire [31:0] instruction;    
    instruction_mem instruction_mem1(
        .read_add(nextpc),
        .instruction(instruction),
        .clk(clk)
    );

    
    wire [5:0] opcode = instruction[31:26];
    wire [6:0] control_signal;  
    control_unit control_unit1(
        .op_code(opcode),
        .clk(clk),
        .control_signals(control_signal)
    );
  
  wire [4:0] read_reg1 = instruction[25:21];
  wire [4:0] read_reg2 = instruction[20:16];
 
  wire [4:0] write_reg_to_mux = instruction[15:11];
  wire [31:0] write_reg;
  mux_2_1 mux1(
    .in1(write_reg_to_mux),
    .in0(read_reg2),
    .sel(control_signal),
    .out(write_reg)
  );
  
  wire [4:0] write_data = instruction[25:21];
  wire [31:0] read_data1;
  wire [31:0] read_data2;
  wire reg_write;
  register_file register_file1(.clk(clk),
                               .reg_write(reg_write),
                               .read_reg1(read_reg1),
                               .read_reg2(read_reg2),
                               .write_reg(write_reg),
                               .write_data(write_data),
                               .read_data1(read_data1),
                               .read_data2(read_data2));
  
  
  wire [15:0] notextended = instruction[15:0];
  wire [31:0] extended;
  sign_extend sign_extend1(.notextended(notextended),
                           .clk(clk),
                           .extended(extended)
                          );
  
  wire [5:0] alu_op = instruction[5:0];
  wire [3:0] alu_control;
  ALU_Control alu_control1(.ALU_OP(alu_op),
                           .funct(control_signal),
                           .ALU_control(alu_control));
  
  wire [31:0] in_alu2;
  mux_2_1 mux1(
    .in0(read_data2),
    .in1(extended),
    .sel(control_signal),
    .out(in_alu2)
  );
  
  wire [31:0] res_alu;
  wire zero_flag
  ALU alu1(.in_alu1(read_data1),
           .in_alu2(in_alu2),
           .ALU_control(alu_control),
           .clk(clk),
           .res_alu(res_alu),
           .zero_flag(zero_flag));

  wire [31:0] read_data;
  wire [] 
  Data_memory data_mem1(.add(res_alu),
                        .write_data(read_data2),
                        .mem_read(control_signal),
                        .clk(clk),
                        .mem_write(control_signal),
                        .read_data(read_data)
                       );
  
  mux_2_1 mux1(
    .in0(read_data),
    .in1(res_alu),
    .sel(control_signal),
    .out(out)
  );
  
  assign out = (control_signals[2]) ? mem_data : alu_res;
  
  
  
endmodule



