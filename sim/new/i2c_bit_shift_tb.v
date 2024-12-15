`timescale 1ns / 1ps
module i2c_bit_shift_tb;
    
    // Declare testbench signals
    reg Clk;                    // System clock
    reg Rst_n;                  // Active-low reset
    reg [5:0] Cmd;              // I2C command input
    reg Go;                     // Start signal
    wire [7:0] Rx_DATA;         // Data received from I2C
    reg [7:0] Tx_DATA;          // Data to transmit over I2C
    wire Trans_Done;            // Transfer done flag
    wire ack_o;                 // Acknowledge signal
    wire i2c_sclk;              // I2C clock line
    wire i2c_sdat;              // I2C data line

    // Add pullup resistor for the bidirectional data line
    pullup PUP(i2c_sdat);

    // Define command constants
    localparam 
        WR   = 6'b000001,   // Write command
        STA  = 6'b000010,   // Start condition
        RD   = 6'b000100,   // Read command
        STO  = 6'b001000,   // Stop condition
        ACK  = 6'b010000,   // Acknowledge command
        NACK = 6'b100000;   // No acknowledge command
    
    // Instantiate the Device Under Test (DUT)
    i2c_bit_shift DUT(
        .Clk(Clk),
        .Rst_n(Rst_n),
        .Cmd(Cmd),
        .Go(Go),
        .Rx_DATA(Rx_DATA),
        .Tx_DATA(Tx_DATA),
        .Trans_Done(Trans_Done),
        .ack_o(ack_o),
        .i2c_sclk(i2c_sclk),
        .i2c_sdat(i2c_sdat)
    );

    // Instantiate a memory model (EEPROM) for simulation
    M24LC04B M24LC04B(
        .A0(0),         // Address pin A0
        .A1(0),         // Address pin A1
        .A2(0),         // Address pin A2
        .WP(0),         // Write protection pin
        .SDA(i2c_sdat), // Serial data line
        .SCL(i2c_sclk), // Serial clock line
        .RESET(~Rst_n)  // Active-high reset signal
    );

    // Generate a 50 MHz clock signal (period = 20 ns)
    always #10 Clk = ~Clk;

    // Testbench initialization
    initial begin
        // Initialize signals
        Clk = 1;
        Rst_n = 0;
        Cmd = 6'b000000;
        Go = 0;
        Tx_DATA = 8'd0;

        // Reset sequence
        #2001;
        Rst_n = 1;
        #2000;

        // Write data to EEPROM at address B1 with value DA
        // Step 1: Start condition + EEPROM address + Write bit
        Cmd = STA | WR;
        Go = 1;
        Tx_DATA = 8'hA0 | 8'd0; // Write command with device address
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;
        
        // Step 2: Write 8-bit EEPROM internal address (B1)
        Cmd = WR;
        Go = 1;
        Tx_DATA = 8'hB1; // Write address B1
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;
        
        // Step 3: Write 8-bit data value + Stop condition
        Cmd = WR | STO;
        Go = 1;
        Tx_DATA = 8'hda; // Data value DA
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;

        // Wait for some time before the next operation
        #5000000;

        // Read data from EEPROM at address B1
        // Step 1: Start condition + EEPROM address + Write bit
        Cmd = STA | WR;
        Go = 1;
        Tx_DATA = 8'hA0 | 8'd0; // Write command with device address
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;

        // Step 2: Write 8-bit EEPROM internal address (B1)
        Cmd = WR;
        Go = 1;
        Tx_DATA = 8'hB1; // Write address B1
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;

        // Step 3: Start condition + EEPROM address + Read bit
        Cmd = STA | WR;
        Go = 1;
        Tx_DATA = 8'hA0 | 8'd1; // Read command with device address
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;

        // Step 4: Read 8-bit data value + Stop condition
        Cmd = RD | STO;
        Go = 1;
        @ (posedge Clk);
        #1;
        Go = 0;
        @ (posedge Trans_Done);
        #200;

        // Final delay before ending simulation
        #2000;
        $stop;
    end
endmodule