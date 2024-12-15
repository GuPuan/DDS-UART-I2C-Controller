module DDS_Module (
    input wire Clk,                 // System clock
    input wire Reset_n,             // Active-low reset
    input wire [1:0] Mode_Sel,      // Waveform mode selection
    input wire [31:0] Fword,        // Frequency word
    input wire [11:0] Pword,        // Phase word
    output reg [13:0] Data          // Output waveform data
);

    // Registers for storing parameters
    reg [1:0] Mode_Sel_r;           // Registered waveform mode
    reg [31:0] Fword_r;             // Registered frequency word
    reg [11:0] Pword_r;             // Registered phase word

    // Frequency accumulator
    reg [31:0] Freq_ACC;            // Accumulated frequency
    reg [11:0] Rom_Addr_reg;        // Registered ROM address

    // ROM address signal
    wire [11:0] Rom_Addr;           // ROM address signal

    // Registered ROM output signals for synchronization
    reg [13:0] Data_sine_reg, Data_square_reg, Data_triangular_reg; // Sine, square, and triangular waveform data
    wire [13:0] Data_sine, Data_square, Data_triangular;           // ROM output for sine, square, and triangular waveforms

    // Synchronized update of parameters
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n) begin
            Fword_r <= 32'd0;       // Reset frequency word
            Pword_r <= 12'd0;       // Reset phase word
            Mode_Sel_r <= 2'b00;    // Reset waveform mode to sine wave
        end else begin
            Fword_r <= Fword;       // Update frequency word
            Pword_r <= Pword;       // Update phase word
            Mode_Sel_r <= Mode_Sel; // Update waveform mode
        end
    end

    // Frequency accumulator logic
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n)
            Freq_ACC <= 32'd0;      // Reset frequency accumulator
        else
            Freq_ACC <= Fword_r + Freq_ACC; // Accumulate frequency
    end

    // Compute ROM address (ensure address wraps correctly within bounds)
    assign Rom_Addr = (Freq_ACC[31:20] + Pword_r) & 12'hFFF;

    // Address synchronization register
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n)
            Rom_Addr_reg <= 12'd0;  // Reset ROM address register
        else
            Rom_Addr_reg <= Rom_Addr; // Update ROM address register
    end

    // ROM instantiations for different waveform types
    rom_sine u_rom_sine (
        .clka(Clk),                 // Clock input
        .addra(Rom_Addr_reg),       // Address input
        .douta(Data_sine)           // Sine waveform data output
    );

    rom_square u_rom_square (
        .clka(Clk),                 // Clock input
        .addra(Rom_Addr_reg),       // Address input
        .douta(Data_square)         // Square waveform data output
    );

    rom_triangular u_rom_triangular (
        .clka(Clk),                 // Clock input
        .addra(Rom_Addr_reg),       // Address input
        .douta(Data_triangular)     // Triangular waveform data output
    );

    // Synchronize ROM data output to avoid glitches
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n) begin
            Data_sine_reg <= 14'd0;         // Reset sine waveform data register
            Data_square_reg <= 14'd0;       // Reset square waveform data register
            Data_triangular_reg <= 14'd0;   // Reset triangular waveform data register
        end else begin
            Data_sine_reg <= Data_sine;         // Synchronize sine waveform data
            Data_square_reg <= Data_square;     // Synchronize square waveform data
            Data_triangular_reg <= Data_triangular; // Synchronize triangular waveform data
        end
    end

    // Output logic (select waveform based on Mode_Sel)
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n)
            Data <= 14'd0;           // Default output to 0 on reset
        else begin
            case (Mode_Sel_r)
                2'b00: Data <= Data_sine_reg;       // Sine waveform
                2'b01: Data <= Data_square_reg;     // Square waveform
                2'b10: Data <= Data_triangular_reg; // Triangular waveform
                default: Data <= 14'd0;             // Default to 0 for unknown mode
            endcase
        end
    end

endmodule