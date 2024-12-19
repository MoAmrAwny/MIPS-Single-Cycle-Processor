
module pc(
    input wire [31:0] lastpc,
    output reg [31:0] nextpc,
    input wire clk
);
  
    always @(posedge clk) 
            nextpc <= lastpc+1;
endmodule

    
module instruction_mem(
  input wire [31:0] read_add,
  output reg [31:0] instruction // 4KB instruction memory
);
  reg [31:0] mem [1024]; // To store instructions: 1024 words of a 32-bit instruction

    	// Initialize specific instructions
    initial begin
        mem[0] = 32'h8D0B0000; // lw 	$t3, 0($t0) //lw $t0, 32 ($s3) 8E 68 00 20
        mem[1] = 32'h02484020; //add 	$t0, $s2, $t0
        mem[2] = 32'h01134822; // sub 	$t1, $t0, $s3
        mem[3] = 32'hAD280000; // sw    $t0, 0 ($t1)
        mem[4] = 32'h11600001; //beq 	$t3, $zero, Label
    end
  

  // Combinational read operation
  always @(*) begin
    instruction = mem[read_add [11:2]]; // Shift right by 2 to make it word-aligned
  end

endmodule



module Control_Unit( instruction,alu_op,branch,reg_des,mem_write,mem_read,reg_write,MemtoReg,alu_src);
  input wire [31:26] instruction;
  output reg branch,reg_des,mem_write,mem_read,reg_write,MemtoReg,alu_src;
  output reg [1:0] alu_op;
  
    parameter lw       = 6'd35;     // Load Word
    parameter sw       = 6'd43;     // Store Word
    parameter beq      = 6'd4;      // Branch if Equal
    parameter r_format = 6'd0;      // R-type instructions
    parameter addi     = 6'd8;      // Add Immediate
  
  always@(instruction)begin 
    case(instruction)
    lw :begin                  
           alu_op     = 2'b00 ;
           branch     = 1'b0  ;
           reg_des    = 1'b0  ;
           mem_write  = 1'b0  ;
           mem_read   = 1'b1  ;
           reg_write  = 1'b1  ;
           MemtoReg   = 1'b1  ;
           alu_src    = 1'b1  ;

    end
     sw :begin                    
           alu_op     = 2'b00 ;
           branch     = 1'b0  ;
           reg_des    = 1'bx  ;
           mem_write  = 1'b1  ;
           mem_read   = 1'b0  ;
           reg_write  = 1'b0  ;
           MemtoReg   = 1'bx  ;
           alu_src    = 1'b1  ;

    end 
       beq :begin                
           alu_op     = 2'b01 ;
           branch     = 1'b1  ;
           reg_des    = 1'bx  ;
           mem_write  = 1'b0  ;
           mem_read   = 1'b0  ;
           reg_write  = 1'b0  ;
           MemtoReg   = 1'bx  ;
           alu_src    = 1'b0  ;

    end 
       r_format :begin                    
           alu_op     = 2'b10 ;
           branch     = 1'b0  ;
           reg_des    = 1'b1  ;
           mem_write  = 1'b0  ;
           mem_read   = 1'b0  ;
           reg_write  = 1'b1  ;
           MemtoReg   = 1'b0  ;
           alu_src    = 1'b0  ;

    end
      addi :begin                     
           alu_op     = 2'b00 ;
           branch     = 1'b0  ;
           reg_des    = 1'b0  ;
           mem_write  = 1'b0  ;
           mem_read   = 1'b0  ;
           reg_write  = 1'b1  ;
           MemtoReg   = 1'b0  ;
           alu_src    = 1'b1  ;

    end
      default :begin
           alu_op     = 2'b00 ;
           branch     = 1'b0  ;
           reg_des    = 1'b0  ;
           mem_write  = 1'b0  ;
           mem_read   = 1'b0  ;
           reg_write  = 1'b0  ;
           MemtoReg   = 1'b0  ;
           alu_src    = 1'b0  ;
          
      end
    endcase
 
  end
  
  
endmodule
  
// ALU Control
module Alu_Control(alu_op,instruction,control_lines);

  input wire [1:0] alu_op;
  input wire [5:0] instruction;
  output reg [3:0] control_lines;
 
  
  always@(*)begin
    if(alu_op == 2'b10)begin
      case(instruction)
      
        6'd32 : control_lines = 4'b0010 ;//add
        6'd34 : control_lines = 4'b0110 ;//sub
        6'd36 : control_lines = 4'b0000 ;//and
        6'd37 : control_lines = 4'b0001 ;//or
        6'd39 : control_lines = 4'b1100 ;//nor
        6'd42 : control_lines = 4'b0111 ;//slt
          
      endcase 
      end 
    else if(alu_op == 2'b00)
       control_lines = 4'b0010 ;//add
    
    else if(alu_op == 2'b01)
       control_lines = 4'b0110 ;//sub
      
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
  
  // Initialize secific registers
    initial begin
      registers[18] = 32'h0f42a800;  //$s2
      registers[0] = 32'h00000000;  //$0
      registers[19] = 32'h00723419;  //$s3
      registers[9]= 32'h00000821;    //$t1
      registers[8] = 32'h00f00364; //$t0
      registers[11] = 32'h00000000; //$t3
      
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


// Sign Extend
module sign_extend(
  input wire [15:0] notextended,
  output wire [31:0] extended
);
  assign extended = { {16{notextended[15]}}, notextended }; // sign extend
endmodule




module ALU(input wire[31:0] in_alu1,
           input wire [31:0] in_alu2,
           input wire [3:0] ALU_control,
           output reg [31:0] res_alu,
           output reg zero_flag);
  always @(*)
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
        



module Data_memory(
  input wire [31:0] add,
  input wire [31:0] write_data,
  input wire  mem_read,
  input wire  mem_write,
  input wire MemtoReg,
  output reg [31:0] read_data);
  
  reg [31:0] location [4096];
  
  initial 
    begin
      integer i;
      for( i=0 ; i<4096 ; i=i+1 )
        location[i] = 32'b0;
    end
  // Initialize specific locations
    initial begin
      location[0] = 32'h8D0B0f00; 
      location[1] = 32'h8C220004; 
      location[2] = 32'h8C23afe8;
      location[3] = 32'h8C230008;
      location[4] = 32'h8Cab4451;
      location[5] = 32'h8C230f08;
      location[6] = 32'h8C230f08;
      location[7] = 32'h00000f08;
      
    end
  
  always @(*)
    begin
      if(mem_write == 1'b1) location [add[11:0]] <= write_data;
      if(mem_read == 1'b1)  read_data <= location [add[11:0]];
    end
endmodule
        
  


module mux_2_1(
    input wire [31:0] in0,  
    input wire [31:0] in1,  
    input wire sel,         
    output reg [31:0] out   
);

    
    always @(*) begin
        case (sel)
            1'b0: out = in0;
            1'b1: out = in1; 
            default: out = 32'b0;
        endcase
    end
endmodule



// Adder module
module Adder(
    input wire [31:0] in1,  // First input
    input wire [31:0] in2,  // Second input
    output wire [31:0] out  // Output
);
    assign out = in1 + in2; // Perform addition
endmodule

// ShiftLeft2 module
module shiftleft2(
    input wire [31:0] in,  // Input to shift left
    output wire [31:0] out // Shifted output
);
    assign out = in << 2;  // Shift left by 2 bits
endmodule


// module MIPS_Processor(
//     input wire clk,
//     input wire reset,
//     output wire [31:0] pc_out
// );
//     // Wires for interconnections
//     wire [31:0] pc_current, pc_next, instruction;
//     wire [31:0] reg_data1, reg_data2, alu_result, mem_data;
//     wire [31:0] sign_ext_imm, branch_target, jump_target, alu_input2, write_data;
//     wire [4:0] write_reg;
//     wire [6:0] control_signals;
//     wire [3:0] alu_control;
//     wire zero_flag, jump, mem_read, mem_write, reg_write, alu_src,mem_to_reg, reg_dst;;

//     // Assign control signals to respective wires
//     assign jump = control_signals[6];
//     assign reg_write = control_signals[5];
//     assign mem_read = control_signals[4];
//     assign mem_write = control_signals[3];
//     assign alu_src = control_signals[2];
//     assign write_data = mem_read ? mem_data : alu_result;

//     // Program Counter
//     pc PC(
//         .lastpc(pc_current),
//         .reset(reset),
//         .clk(clk),
//         .nextpc(pc_next)
//     );

//     // Instruction Memory
//     instruction_mem IM(
//         .read_add(pc_next),
//         .instruction(instruction)
//     );

//     // Control Unit
//     Control_Unit CU(
//         .instruction(instruction[31:26]),
//         .alu_op(control_signals[1:0]),
//         .branch(branch),
//         .reg_des(reg_dst),
//         .mem_write(mem_write),
//         .mem_read(mem_read),
//         .reg_write(reg_write),
//         .MemtoReg(mem_to_reg),
//         .alu_src(alu_src)
//     );

//     // Register File
//     register_file RF(
//         .clk(clk),
//         .reg_write(reg_write),
//         .read_reg1(instruction[25:21]),
//         .read_reg2(instruction[20:16]),
//         .write_reg(write_reg),
//         .write_data(write_data),
//         .read_data1(reg_data1),
//         .read_data2(reg_data2)
//     );

//     // Sign Extension
//     sign_extend SE(
//         .notextended(instruction[15:0]),
//         .extended(sign_ext_imm)
//     );

//     // ALU Control
//     Alu_Control ALU_Ctrl(
//     .alu_op(control_signals[1:0]),     
//     .instruction(instruction[5:0]),   
//     .control_lines(alu_control)       
// 	);
    
  
//     // ALU
//     ALU ALU_Unit(
//         .in_alu1(reg_data1),
//         .in_alu2(alu_src ? sign_ext_imm : reg_data2),
//         .ALU_control(alu_control),
//         .res_alu(alu_result),
//       	.zero_flag(zero_flag)
//     );

//     // Data Memory
//     Data_memory DM(
//         .add(alu_result),
//         .write_data(reg_data2),
//         .mem_read(mem_read),
//         .mem_write(mem_write),
//         .read_data(mem_data)
//     );
  
  
//   mux_2_1 MUX_RegDst(
//         .in0(instruction[20:16]),
//         .in1(instruction[15:11]),
//         .sel(reg_dst),
//         .out(write_reg)
//     );

//     // MUX for ALU Input 2 (ALUSrc)
//     mux_2_1 MUX_ALUSrc(
//         .in0(reg_data2),
//         .in1(sign_ext_imm),
//         .sel(alu_src),
//         .out(alu_input2)
//     );

//     // MUX for Write Data (MemtoReg)
//     mux_2_1 MUX_MemtoReg(
//         .in0(alu_result),
//         .in1(mem_data),
//         .sel(mem_to_reg),
//         .out(write_data)
//     );

//    assign branch_target = pc_next + (sign_ext_imm << 2);

//     // Jump Target Calculation
//     assign jump_target = {pc_next[31:28], instruction[25:0] << 2};

//     // PC Source MUX
//     wire pc_src;
//     assign pc_src = branch & zero_flag;
//     mux_2_1 MUX_PC(
//         .in0(pc_next + 4),
//         .in1(branch_target),
//         .sel(pc_src),
//         .out(pc_current)
//     );
 	
//     assign pc_out = pc_next;

// endmodule



