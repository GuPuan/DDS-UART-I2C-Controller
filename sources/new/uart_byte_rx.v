module uart_byte_rx(
    input Clk,              // System clock signal
    input Reset_n,          // Active-low reset signal
    input uart_rx,          // UART receive signal
    output reg Rx_Done,     // Data reception complete signal
    output reg [7:0] Rx_Data, // 8-bit received data output register
    output reg Frame_Error  // Frame error signal
);

    // UART parameter configuration
    parameter CLOCK_FREQ = 50_000_000;  // System clock frequency in Hz
    parameter BAUD = 115200;            // UART baud rate
    parameter MCNT_BAUD = CLOCK_FREQ / BAUD - 1; // Clock cycles per UART bit period

    // Internal register definitions
    reg [7:0] r_Rx_Data;                // Temporary register to store received data bits
    reg [29:0] baud_div_cnt;            // Baud rate clock divider counter
    reg en_baud_cnt;                    // Baud rate counter enable signal
    reg [3:0] bit_cnt;                  // Receive bit counter
    reg dff0_uart_rx, dff1_uart_rx;     // Double D flip-flops for UART input synchronization
    reg r_uart_rx;                      // Previous state of UART input
    reg [2:0] stop_bit_sample;          // Stop bit redundancy sampling register

    // Internal signal definitions
    wire nedge_uart_rx;                 // UART input falling edge detection signal
    wire w_Rx_Done;                     // Data reception completion signal

    // UART reception logic
    always @(posedge Clk or negedge Reset_n) begin
        if (!Reset_n) begin
            // Reset all registers
            baud_div_cnt <= 0;
            en_baud_cnt <= 0;
            bit_cnt <= 0;
            r_Rx_Data <= 8'd0;
            Rx_Data <= 8'd0;
            Rx_Done <= 0;
            Frame_Error <= 0;
            stop_bit_sample <= 3'b111; // Initialize to high, indicating no error
        end else begin
            // Baud rate counter logic
            if (en_baud_cnt) begin
                if (baud_div_cnt == MCNT_BAUD)
                    baud_div_cnt <= 0; // Reset counter after one bit period
                else
                    baud_div_cnt <= baud_div_cnt + 1; // Increment counter
            end else begin
                baud_div_cnt <= 0; // Reset counter if not enabled
            end

            // Receive bit counter logic
            if (en_baud_cnt && baud_div_cnt == MCNT_BAUD) begin
                if (bit_cnt == 9) 
                    bit_cnt <= 0; // Reset counter after all bits are received
                else
                    bit_cnt <= bit_cnt + 1; // Increment bit counter
            end

            // Sample data bits at the middle of each bit period
            if (baud_div_cnt == MCNT_BAUD / 2) begin
                case (bit_cnt)
                    1: r_Rx_Data[0] <= dff1_uart_rx; // LSB
                    2: r_Rx_Data[1] <= dff1_uart_rx;
                    3: r_Rx_Data[2] <= dff1_uart_rx;
                    4: r_Rx_Data[3] <= dff1_uart_rx;
                    5: r_Rx_Data[4] <= dff1_uart_rx;
                    6: r_Rx_Data[5] <= dff1_uart_rx;
                    7: r_Rx_Data[6] <= dff1_uart_rx;
                    8: r_Rx_Data[7] <= dff1_uart_rx; // MSB
                    9: stop_bit_sample <= {stop_bit_sample[1:0], dff1_uart_rx}; // Redundantly sample stop bit
                endcase
            end

            // Reception completion signal logic
            if (w_Rx_Done) begin
                Rx_Done <= 1; // Indicate data reception complete
                Rx_Data <= r_Rx_Data; // Update output data
                // Check stop bit validity (at least two of three samples must be high)
                if (stop_bit_sample[2] + stop_bit_sample[1] + stop_bit_sample[0] < 2)
                    Frame_Error <= 1; // Stop bit error
                else
                    Frame_Error <= 0; // No error
            end else begin
                Rx_Done <= 0; // Clear completion signal
            end

            // Enable baud rate counting on falling edge of start bit
            if (nedge_uart_rx)
                en_baud_cnt <= 1;
            else if (bit_cnt == 9 && baud_div_cnt == MCNT_BAUD)
                en_baud_cnt <= 0; // Disable counting after stop bit reception
        end
    end

    // Double flip-flops for synchronizing UART input signal
    always @(posedge Clk) begin
        dff0_uart_rx <= uart_rx;         // First stage synchronization
        dff1_uart_rx <= dff0_uart_rx;   // Second stage synchronization
        r_uart_rx <= dff1_uart_rx;      // Record previous state of synchronized signal
    end

    // Falling edge detection logic
    assign nedge_uart_rx = (dff1_uart_rx == 0) && (r_uart_rx == 1);

    // Reception completion signal logic
    assign w_Rx_Done = (bit_cnt == 9 && baud_div_cnt == MCNT_BAUD / 2);

endmodule