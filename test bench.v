`timescale 1ns / 1ps

module MIPS_Processor_tb;

    // Clock and reset signals
    reg clk;
    reg reset;
    integer i;
    integer outfile; // File handle for output

    // Output wire for the program counter
    wire [31:0] pc_out;

    // Instantiate the MIPS Processor
    MIPS_Processor uut (
        .clk(clk),
        .reset(reset),
        .pc_out(pc_out)
    );

    // Clock generation
    always begin
        #5 clk = ~clk; // Toggle clock every 5 ns (100 MHz clock)
    end

    initial begin
        // Initialize signals
        clk = 0;
        reset = 1;

        // Open the output file for writing
        outfile = $fopen("outputfile.txt", "w");
        if (outfile == 0) begin
            $display("Error: Could not open output file.");
            $finish;
        end

        // Release reset after 10 ns
        #10 reset = 0;

        // Wait for a few clock cycles
        #100;

        // End simulation
        $fclose(outfile); // Close the output file
        $stop;
    end

    // Memory initialization and instruction loading
    initial begin
        // Dump file for waveform
        $dumpfile("dump.vcd");
        $dumpvars(0, MIPS_Processor_tb);

        // Load instructions into instruction memory
        uut.IM.mem[0] = 32'b000000_00001_00010_00011_00000_100000; // ADD $3, $1, $2
        uut.IM.mem[1] = 32'b100010_00001_00100_0000000000000100;  // LW $4, 4($1)
        uut.IM.mem[2] = 32'b101011_00001_00101_0000000000001000;  // SW $5, 8($1)
        uut.IM.mem[3] = 32'b000100_00001_00110_0000000000000010;  // BEQ $1, $6, 2
        uut.IM.mem[4] = 32'b000000_00001_00111_01000_00000_100010; // SUB $8, $1, $7

        // Initialize registers
        uut.RF.registers[1] = 32'd10;  // $1 = 10
        uut.RF.registers[2] = 32'd20;  // $2 = 20
        uut.RF.registers[6] = 32'd10;  // $6 = 10 (for BEQ)
        uut.RF.registers[7] = 32'd5;   // $7 = 5
        uut.RF.registers[5] = 32'd50;  // $5 = 50 (for SW)

        // Initialize data memory
        uut.DM.location[4] = 32'd100; // Address 4 = 100

        // Simulation output to file
        #20 $fwrite(outfile, "Initial State:\n");
        for (i = 0; i < 10; i = i + 1) begin
            $fwrite(outfile, "Instruction Memory [%0d] = %b\n", i, uut.IM.mem[i]);
        end
        $fwrite(outfile, "\n");

        // Allow some time for execution
        #200;

        // Output register file contents after execution
        $fwrite(outfile, "Final Register File Contents:\n");
        for (i = 0; i < 32; i = i + 1) begin
            $fwrite(outfile, "Register %0d: %d\n", i, uut.RF.registers[i]);
        end
        $fwrite(outfile, "\n");

        // Output data memory contents after execution
        $fwrite(outfile, "Final Data Memory Contents:\n");
        for (i = 0; i < 10; i = i + 1) begin
            $fwrite(outfile, "Address %0d: %d\n", i, uut.DM.location[i]);
        end

        // Wait for some more cycles
        #100;
    end

endmodule
