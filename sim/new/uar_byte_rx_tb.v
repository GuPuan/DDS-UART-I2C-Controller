`timescale 1ns/1ps

module uart_byte_rx_tb;

    // Parameter definitions
    parameter CLOCK_FREQ = 50_000_000; // System clock frequency (50 MHz)
    parameter BAUD = 115200;           // UART baud rate
    parameter CLOCK_PERIOD = 20;      // Clock period in nanoseconds (50 MHz -> 20 ns)
    parameter BIT_PERIOD = 1_000_000_000 / BAUD; // UART bit period in nanoseconds

    // Input signals
    reg Clk;         // System clock
    reg Reset_n;     // Active-low reset
    reg uart_rx;     // UART receive signal (input to DUT)

    // Output signals
    wire Rx_Done;       // Byte received signal (1 clock pulse when a byte is received)
    wire [7:0] Rx_Data; // Received byte data
    wire Frame_Error;   // Error flag for framing (e.g., missing or incorrect stop bit)

    // Instantiate the UART byte receiver (DUT - Device Under Test)
    uart_byte_rx #(
        .CLOCK_FREQ(CLOCK_FREQ), // System clock frequency parameter
        .BAUD(BAUD)              // UART baud rate parameter
    ) uut (
        .Clk(Clk),               // Clock input
        .Reset_n(Reset_n),       // Reset input
        .uart_rx(uart_rx),       // UART RX signal input
        .Rx_Done(Rx_Done),       // Output: byte received signal
        .Rx_Data(Rx_Data),       // Output: received data
        .Frame_Error(Frame_Error)// Output: framing error signal
    );

    // Clock generation: Generates a 50 MHz clock signal
    initial begin
        Clk = 0;                  // Initialize clock to 0
        forever #(CLOCK_PERIOD / 2) Clk = ~Clk; // Toggle clock every half period
    end

    // Initialize and apply test input signals
    initial begin
        // Initialize signals
        Reset_n = 0;               // Assert reset
        uart_rx = 1;               // Default idle state for UART (high)

        // Hold reset for 10 clock cycles
        #(10 * CLOCK_PERIOD);
        Reset_n = 1;               // Deassert reset

        // Test Case 1: Send byte 0xAA (binary 10101010)
        send_uart_byte(8'b10101010); // Transmit byte 0xAA
        #(10 * BIT_PERIOD);         // Wait for some time

        // Test Case 2: Send byte 0xCC (binary 11001100)
        send_uart_byte(8'b11001100); // Transmit byte 0xCC
        #(10 * BIT_PERIOD);         // Wait for some time

        // Test Case 3: (Optional) Send byte with stop bit error
        // Uncomment the following lines to test framing error
        // send_uart_byte_with_error(8'b11110000); // Transmit byte 0xF0 with stop bit error
        // #(10 * BIT_PERIOD);

        // Test Case 4: Send byte 0xBB (binary 10111011)
        send_uart_byte(8'b10111011); // Transmit byte 0xBB
        #(10 * BIT_PERIOD);

        // Stop simulation after all test cases
        #(100 * CLOCK_PERIOD);
        $stop;
    end

    // Task to simulate UART byte transmission (with valid framing)
    task send_uart_byte(input [7:0] data);
        integer i;
        begin
            // Start bit: UART line goes low
            @(posedge Clk);
            uart_rx = 0; // Start bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);

            // Transmit 8 data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i]; // Transmit each bit of the data byte
                repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
            end

            // Stop bit: UART line goes high
            uart_rx = 1; // Stop bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
        end
    endtask

    // Task to simulate UART byte transmission with a framing error (invalid stop bit)
    task send_uart_byte_with_error(input [7:0] data);
        integer i;
        begin
            // Start bit: UART line goes low
            @(posedge Clk);
            uart_rx = 0; // Start bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);

            // Transmit 8 data bits (LSB first)
            for (i = 0; i < 8; i = i + 1) begin
                uart_rx = data[i]; // Transmit each bit of the data byte
                repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
            end

            // Stop bit: UART line stays low instead of high (introducing a framing error)
            uart_rx = 0; // Incorrect stop bit
            repeat(BIT_PERIOD / CLOCK_PERIOD) @(posedge Clk);
        end
    endtask

endmodule