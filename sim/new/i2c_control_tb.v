`timescale 1ns / 1ps

module i2c_control_tb;

    // Declare testbench signals
    reg Clk;                // System clock
    reg Rst_n;              // Active-low reset
    reg wrreg_req;          // Write register request
    reg rdreg_req;          // Read register request
    reg [15:0] addr;        // Register address (16-bit for compatibility with larger address space)
    reg addr_mode;          // Address mode (0 = 8-bit, 1 = 16-bit)
    reg [7:0] wrdata;       // Data to write to the register
    wire [7:0] rddata;      // Data read from the register
    reg [7:0] device_id;    // I2C device ID
    wire RW_Done;           // Read/Write operation done signal
    wire ack;               // Acknowledge signal from the device
    wire i2c_sclk;          // I2C clock line
    wire i2c_sdat;          // I2C data line

    // Pullup resistor for bidirectional I2C data line
    pullup PUP(i2c_sdat);

    // Instantiate the DUT (Device Under Test)
    i2c_control DUT(
        .Clk        (Clk      ),  // Clock input
        .Rst_n      (Rst_n    ),  // Reset input
        .wrreg_req  (wrreg_req),  // Write request
        .rdreg_req  (rdreg_req),  // Read request
        .addr       (addr     ),  // Address
        .addr_mode  (addr_mode),  // Address mode
        .wrdata     (wrdata   ),  // Write data
        .rddata     (rddata   ),  // Read data
        .device_id  (device_id),  // Device ID
        .RW_Done    (RW_Done  ),  // Read/Write done signal
        .ack        (ack      ),  // Acknowledge signal
        .i2c_sclk   (i2c_sclk ),  // I2C clock line
        .i2c_sdat   (i2c_sdat )   // I2C data line
    );

    // Instantiate the EEPROM memory model for testing
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
    initial Clk = 1;
    always #10 Clk = ~Clk;

    // Testbench initialization
    initial begin
        // Reset all signals
        Rst_n = 0;
        rdreg_req = 0;
        wrreg_req = 0;
        #2001;           // Hold reset for some time
        Rst_n = 1;       // Release reset
        #2000;           // Wait for DUT to stabilize

        // Write operations to EEPROM
        write_one_byte(8'hA0, 8'h0A, 8'hD1); // Write D1 to address 0x0A
        write_one_byte(8'hA0, 8'h0B, 8'hD2); // Write D2 to address 0x0B
        write_one_byte(8'hA0, 8'h0C, 8'hD3); // Write D3 to address 0x0C
        write_one_byte(8'hA0, 8'h0F, 8'hD4); // Write D4 to address 0x0F

        // Read operations from EEPROM
        read_one_byte(8'hA0, 8'h0A);         // Read from address 0x0A
        read_one_byte(8'hA0, 8'h0B);         // Read from address 0x0B
        read_one_byte(8'hA0, 8'h0C);         // Read from address 0x0C
        read_one_byte(8'hA0, 8'h0F);         // Read from address 0x0F

        // Stop simulation
        $stop;	
    end

    // Task to perform a write operation
    task write_one_byte;
        input [7:0] id;           // Device ID
        input [7:0] mem_address;  // Memory address to write to
        input [7:0] data;         // Data to write
        begin
            addr = {8'd0, mem_address}; // Set the address (8-bit)
            device_id = id;            // Set the device ID
            addr_mode = 0;             // Use 8-bit address mode
            wrdata = data;             // Set data to write
            wrreg_req = 1;             // Assert write request
            #20;                       // Wait for a short time
            wrreg_req = 0;             // Deassert write request
            @(posedge RW_Done);        // Wait for write operation to complete
            #5000000;                  // Add a delay after the operation
        end
    endtask

    // Task to perform a read operation
    task read_one_byte;
        input [7:0] id;           // Device ID
        input [7:0] mem_address;  // Memory address to read from
        begin
            addr = {8'd0, mem_address}; // Set the address (8-bit)
            device_id = id;            // Set the device ID
            addr_mode = 0;             // Use 8-bit address mode
            rdreg_req = 1;             // Assert read request
            #20;                       // Wait for a short time
            rdreg_req = 0;             // Deassert read request
            @(posedge RW_Done);        // Wait for read operation to complete
            #5000000;                  // Add a delay after the operation
        end
    endtask

endmodule