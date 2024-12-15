`timescale 1ns/1ps

module tb_dds_uart_i2c_controller();

    // Parameters
    parameter CLOCK_FREQ = 50_000_000; // System clock frequency (50 MHz)
    parameter BAUD = 115200;           // UART baud rate
    parameter CLOCK_PERIOD = 20;       // Clock period for 50 MHz (20 ns)
    parameter BIT_PERIOD = 1_000_000_000 / BAUD; // Bit period for the given baud rate (ns)

    // Testbench signals
    reg Clk;           // System clock
    reg Reset_n;       // Reset signal (active low)
    reg uart_rx;       // UART receive line

    wire uart_tx;      // UART transmit line
    wire i2c_sclk;     // I2C clock line
    wire i2c_sdat;     // I2C data line
    wire [13:0] DDS_Data; // DDS output data

    // Internal wires for UART communication
    wire uart_rx_done;        // UART RX completion flag
    wire [7:0] uart_rx_data;  // UART received data
    wire uart_frame_error;    // UART frame error flag

    // I2C pull-up resistor simulation
    pullup PUP(i2c_sdat);

    // EEPROM Model (e.g., M24LC04B)
    M24LC04B EEPROM (
        .A0(0),       // Address pin 0
        .A1(0),       // Address pin 1
        .A2(0),       // Address pin 2
        .WP(0),       // Write protection disabled
        .SDA(i2c_sdat), // I2C data line
        .SCL(i2c_sclk), // I2C clock line
        .RESET(~Reset_n) // Reset signal
    );

    // System clock generation
    always begin
        #(CLOCK_PERIOD / 2) Clk = ~Clk;
    end

    // UART RX module instantiation
    uart_byte_rx #(
        .CLOCK_FREQ(CLOCK_FREQ),
        .BAUD(BAUD)
    ) uart_rx_inst (
        .Clk(Clk),
        .Reset_n(Reset_n),
        .uart_rx(uart_rx),
        .Rx_Done(uart_rx_done),
        .Rx_Data(uart_rx_data),
        .Frame_Error(uart_frame_error)
    );

    // DUT (Device Under Test) instantiation
    dds_uart_i2c_controller uut (
        .Clk(Clk),
        .Reset_n(Reset_n),
        .uart_rx(uart_rx),
        .uart_tx(uart_tx),
        .i2c_sclk(i2c_sclk),
        .i2c_sdat(i2c_sdat),
        .DDS_Data(DDS_Data)
    );

    // Test sequence
    initial begin
        Clk = 0;         // Initialize clock
        Reset_n = 0;     // Assert reset
        uart_rx = 1;     // Idle state for UART RX

        // Wait for reset period
        #(100 * CLOCK_PERIOD);
        Reset_n = 1;     // Deassert reset

        // Test sequence for parameter write and read
        test_write_group(2'b00, 8'h01, 32'h01020304, 12'h123); // Write to Group 1
        #50000000; // Delay

        test_write_group(2'b01, 8'h02, 32'h05060708, 12'h456); // Write to Group 2
        #50000000; // Delay

        test_write_group(2'b10, 8'h00, 32'h01020304, 12'h123); // Write to Group 3
        #50000000; // Delay

        test_write_group(2'b11, 8'h01, 32'h05060708, 12'h456); // Write to Group 4
        #50000000; // Delay

        test_read_group(2'b00); // Read from Group 1
        #1000000; // Delay

        test_read_group(2'b01); // Read from Group 2
        #1000000; // Delay

        test_read_group(2'b10); // Read from Group 3
        #1000000; // Delay

        test_read_group(2'b11); // Read from Group 4
        #1000000; // Delay

        // Stop simulation
        $stop;
    end

    // Task: Write parameters to a specific group
    task test_write_group(input [1:0] group, input [7:0] waveform, input [31:0] fword, input [11:0] pword);
        begin
            send_uart_byte(8'h30 + group); // Send group selection command
            #(BIT_PERIOD);

            send_uart_byte(8'h20); // Send write command
            #(BIT_PERIOD);

            // Send parameters
            send_uart_byte(waveform);          // Waveform type
            #(BIT_PERIOD);
            send_uart_byte(fword[31:24]);      // High byte of Fword
            #(BIT_PERIOD);
            send_uart_byte(fword[23:16]);      // Second high byte of Fword
            #(BIT_PERIOD);
            send_uart_byte(fword[15:8]);       // Second low byte of Fword
            #(BIT_PERIOD);
            send_uart_byte(fword[7:0]);        // Low byte of Fword
            #(BIT_PERIOD);
            send_uart_byte(pword[11:4]);       // High byte of Pword
            #(BIT_PERIOD);
            send_uart_byte({pword[3:0], waveform[3:0]}); // Low byte of Pword and waveform bits
            #(BIT_PERIOD);
        end
    endtask

    // Task: Read parameters from a specific group
    task test_read_group(input [1:0] group);
        begin
            send_uart_byte(8'h30 + group); // Send group selection command
            #(BIT_PERIOD);

            send_uart_byte(8'h10); // Send read command
            #(BIT_PERIOD);

            // Delay for read operation completion
            #(50 * BIT_PERIOD);
        end
    endtask

    // Task: Simulate UART transmission
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            @(posedge Clk);
            uart_rx = 0; // Start bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);

            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i]; // Transmit each bit
                repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
            end

            uart_rx = 1; // Stop bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
        end
    endtask

endmodule