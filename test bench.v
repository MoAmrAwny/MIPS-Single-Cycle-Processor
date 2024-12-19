`timescale 1ns / 1ps
module mips_processor_testbench;


    // Inputs for PC
    reg [31:0] old_address;
    reg clk;

    // Outputs for PC
    wire [31:0] new_address;

    // Clock signal generation
    initial clk = 0;
    always #10 clk = ~clk; // 20 ns clock period

    // Wires for interconnections
    wire [31:0] instruction;
    wire [31:0] read_data1, read_data2, alu_result, mem_data, mux2_out, mux3_out, immediate, shift_out, adder1_out, adder2_out;
    wire [4:0] reg_write_mux_out;
    wire [3:0] control_lines;
    wire [1:0] alu_op;
    wire branch, reg_des, mem_write, mem_read, reg_write, MemtoReg, alu_src, zero, pc_src;

    // Instantiate PC module
    pc pc (
        .lastpc(old_address),
        .clk(clk),
        .nextpc(new_address)
    );

    // Instantiate Instruction Memory
    instruction_mem instruc (
        .read_add(new_address),
        .instruction(instruction)
    );
  


    // Instantiate Control Unit
    Control_Unit control (
        .instruction(instruction[31:26]),
        .alu_op(alu_op),
        .branch(branch),
        .reg_des(reg_des),
        .mem_write(mem_write),
        .mem_read(mem_read),
        .reg_write(reg_write),
        .MemtoReg(MemtoReg),
        .alu_src(alu_src)
    );

    // MUX for Register File Write Register
    mux_2_1 mux1 (
        .in0(instruction[20:16]),
        .in1(instruction[15:11]),
        .sel(reg_des),
        .out(reg_write_mux_out)
    );

    // Instantiate Register File
    register_file RF (
        .clk(clk),
        .reg_write(reg_write),
        .read_reg1(instruction[25:21]),
        .read_reg2(instruction[20:16]),
        .write_reg(reg_write_mux_out),
        .write_data(mux3_out),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );
 

    // Instantiate ALU Control Unit
    Alu_Control alu_control (
        .alu_op(alu_op),
        .instruction(instruction[5:0]),
        .control_lines(control_lines)
    );

    // Instantiate ALU
    ALU alu_inst (
        .in_alu1(read_data1),
        .in_alu2(mux2_out),
        .ALU_control(control_lines),
        .res_alu(alu_result),
        .zero_flag(zero)
    );

    // MUX for ALU Input 2
    mux_2_1 mux2 (
        .in0(read_data2),
        .in1(immediate),
        .sel(alu_src),
        .out(mux2_out)
    );

    // Sign Extension Unit
    sign_extend sign (
        .notextended(instruction[15:0]),
        .extended(immediate)
    );

    // Data Memory
    Data_memory datamem (
        .add(alu_result),
        .write_data(read_data2),
        .mem_read(mem_read),
        .mem_write(mem_write),
        .read_data(mem_data)
    );
  
    // MUX for Write Data to Register File (MemtoReg)
    mux_2_1 mux3 (
        .in0(alu_result),
        .in1(mem_data),
        .sel(MemtoReg),
        .out(mux3_out)
    );

    // Instantiate Adder for PC + 4
    Adder adder1 (
        .in1(new_address),
        .in2(32'd4),
        .out(adder1_out)
    );

    // Instantiate ShiftLeft2 for Branch Offset
    shiftleft2 shift (
        .in(immediate),
        .out(shift_out)
    );

    // Instantiate Adder for Branch Target Address
    Adder adder2 (
        .in1(adder1_out),
        .in2(shift_out),
        .out(adder2_out)
    );

    // PC Source (Branch Decision)
    assign pc_src = branch & zero;

    // MUX for PC Source (Branch/Sequential)
    mux_2_1 mux4 (
        .in0(adder1_out),
        .in1(adder2_out),
        .sel(pc_src),
        .out(mux4_out)
    );

    // Testbench Initialization
    initial begin
        // Initialize inputs
        old_address = 32'd0;
		#20
        
        // Run the simulation
       
        $display("Instruction: %h", instruction);
        $display("ALU Result: %h", alu_result);
        $display("Data Memory Read: %h", mem_data);
        $display("ALU OP: %b",alu_op);
        $display("input control unit: %d", instruction[31:26]);
        $display("Control Signals: %b", control_lines);
        $display("Write Register: %d", reg_write_mux_out);
        $display("Write Data to Register: %h", mux3_out);

        // Simulate branch and observe changes
        old_address <= mux4_out;
       

        // Display after branch
        $display("New Address: %d", new_address);
        $display("Instruction: %h", instruction);
        $display("Data Memory Read: %h", mem_data);
        $display("ALU Result: %h", alu_result);

        $finish;
    end
endmodule
