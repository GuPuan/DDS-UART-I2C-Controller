module dds_uart_i2c_controller(
    input wire Clk,             // System clock
    input wire Reset_n,         // Active-low reset
    input wire uart_rx,         // UART receive signal
    output wire uart_tx,        // UART transmit signal
    output wire i2c_sclk,       // I2C clock signal
    inout wire i2c_sdat,        // I2C data signal
    output wire [13:0] DDS_Data // DDS output data
);

    // UART receive signals
    wire uart_rx_done;          // UART data received indicator
    wire [7:0] uart_rx_data;    // Received UART data
    wire frame_error;           // UART frame error flag

    // I2C control signals and registers
    reg wrreg_req;              // I2C write request
    reg rdreg_req;              // I2C read request
    reg [15:0] iic_addr;        // I2C address register
    reg [7:0] iic_wrdata;       // I2C write data
    wire [7:0] iic_rddata;      // I2C read data
    wire iic_rw_done;           // I2C read/write completion flag

    // DDS configuration registers
    reg [31:0] Fword;           // DDS frequency word
    reg [11:0] Pword;           // DDS phase word
    reg [1:0] Mode_Sel;         // DDS mode selection
    reg [31:0] delay_counter;   // Delay counter

    // Base address for I2C memory mapping
    localparam [15:0] BASE_ADDR_GROUP1 = 16'h000A; // Base address for group 1
    localparam [15:0] BASE_ADDR_GROUP2 = 16'h0010; // Base address for group 2
    localparam [15:0] BASE_ADDR_GROUP3 = 16'h0016; // Base address for group 3
    localparam [15:0] BASE_ADDR_GROUP4 = 16'h001C; // Base address for group 4

    // Timing constants
    localparam DELAY_5MS = 250_000; // 5 ms delay for a 50 MHz clock
    localparam [31:0] DEFAULT_FWORD = 32'h00A00000; // Default frequency word
    localparam [11:0] DEFAULT_PWORD = 12'h000;      // Default phase word
    localparam [1:0] DEFAULT_MODE_SEL = 2'b00;      // Default waveform type

    // Instantiate the DDS module
    DDS_Module dds_inst (
        .Clk(Clk),
        .Reset_n(Reset_n),
        .Mode_Sel(Mode_Sel),
        .Fword(Fword),
        .Pword(Pword),
        .Data(DDS_Data)
    );

    // Instantiate the UART receiver module
    uart_byte_rx #(
        .CLOCK_FREQ(50_000_000), // Clock frequency (50 MHz)
        .BAUD(115200)            // UART baud rate
    ) uart_rx_inst (
        .Clk(Clk),
        .Reset_n(Reset_n),
        .uart_rx(uart_rx),
        .Rx_Done(uart_rx_done),
        .Rx_Data(uart_rx_data),
        .Frame_Error(frame_error)
    );

    // Instantiate the I2C control module
    i2c_control i2c_control_inst (
        .Clk(Clk),
        .Rst_n(Reset_n),
        .wrreg_req(wrreg_req),
        .rdreg_req(rdreg_req),
        .addr(iic_addr),
        .addr_mode(1'b0),        // 16-bit address mode
        .wrdata(iic_wrdata),
        .rddata(iic_rddata),
        .device_id(8'hA0),       // M24LC04B EEPROM device address
        .RW_Done(iic_rw_done),
        .ack(),                  // Acknowledge signal (unused)
        .i2c_sclk(i2c_sclk),
        .i2c_sdat(i2c_sdat)
    );

    // State machine signals
    reg [2:0] state;            // Current state
    reg [7:0] cmd;              // Received command
    reg [2:0] byte_counter;     // Byte counter for multi-byte commands
    reg [15:0] base_addr;       // Base address for I2C memory operations
    reg [7:0] waveform;         // DDS waveform type
    reg [15:0] fword_high;      // High part of frequency word
    reg [15:0] fword_low;       // Low part of frequency word
    reg [11:0] pword;           // Phase word
    reg [1:0] wait_flag;        // Wait state flag

    // State machine states
    localparam IDLE          = 3'd0, // Idle state
               PARSE_CMD     = 3'd1, // Command parsing state
               WRITE_EEPROM  = 3'd2, // I2C write state
               READ_EEPROM   = 3'd3, // I2C read state
               UPDATE_DDS    = 3'd4, // DDS update state
               LOAD_DEFAULT  = 3'd5, // Load default configuration state
               WAIT_DELAY    = 3'd6; // Delay wait state

    // Wait flags for different operations
    localparam WRITE_WAIT = 2'b01,  // Wait for write completion
               READ_WAIT  = 2'b10,  // Wait for read completion
               RESET_WAIT = 2'b00;  // Wait for reset
               
    always @(posedge Clk or negedge Reset_n) begin
    if (!Reset_n) begin
        // Reset all states and registers to their initial values
        state <= IDLE;
        base_addr <= BASE_ADDR_GROUP1; // Default to Group 1 base address
        byte_counter <= 3'b0;         // Reset byte counter
        iic_addr <= 16'h0;           // Reset I2C address
        waveform <= 8'h0;            // Reset waveform type
        fword_high <= 16'h0000;      // Reset frequency word (high part)
        fword_low <= 16'h0000;       // Reset frequency word (low part)
        pword <= 12'h000;            // Reset phase word
        Fword <= DEFAULT_FWORD;      // Load default frequency word
        Pword <= DEFAULT_PWORD;      // Load default phase word
        Mode_Sel <= DEFAULT_MODE_SEL;// Load default mode
    end else begin
        case (state)
            IDLE: begin
                if (uart_rx_done) begin
                    cmd <= uart_rx_data; // Capture received command byte
                    state <= PARSE_CMD;  // Transition to command parsing state
                end
            end

            PARSE_CMD: begin
                case (cmd)
                    8'h30: begin
                        base_addr <= BASE_ADDR_GROUP1; // Select Group 1
                        state <= IDLE;
                    end
                    8'h31: begin
                        base_addr <= BASE_ADDR_GROUP2; // Select Group 2
                        state <= IDLE;
                    end
                    8'h32: begin
                        base_addr <= BASE_ADDR_GROUP3; // Select Group 3
                        state <= IDLE;
                    end
                    8'h33: begin
                        base_addr <= BASE_ADDR_GROUP4; // Select Group 4
                        state <= IDLE;
                    end
                    8'h10: state <= READ_EEPROM;       // Transition to EEPROM read state
                    8'h20: begin
                        // Start receiving waveform, frequency, and phase parameters
                        if (uart_rx_done) begin
                            case (byte_counter)
                                0: begin
                                    waveform <= uart_rx_data; // Waveform type
                                    byte_counter <= byte_counter + 1;
                                end
                                1: begin
                                    fword_high[15:8] <= uart_rx_data; // Frequency word high byte
                                    byte_counter <= byte_counter + 1;
                                end
                                2: begin
                                    fword_high[7:0] <= uart_rx_data; // Frequency word low byte
                                    byte_counter <= byte_counter + 1;
                                end
                                3: begin
                                    fword_low[15:8] <= uart_rx_data; // Frequency word lower high byte
                                    byte_counter <= byte_counter + 1;
                                end
                                4: begin
                                    fword_low[7:0] <= uart_rx_data; // Frequency word lower low byte
                                    byte_counter <= byte_counter + 1;
                                end
                                5: begin
                                    pword[11:4] <= uart_rx_data; // Phase word (high bits)
                                    byte_counter <= byte_counter + 1;
                                end
                                6: begin
                                    pword[3:0] <= uart_rx_data[7:4]; // Phase word (low bits)
                                    state <= WRITE_EEPROM;           // Transition to EEPROM write state
                                    byte_counter <= 0;               // Reset counter
                                end
                            endcase
                        end
                    end

                    default: state <= IDLE; // Unknown command, return to idle state
                endcase
            end

            WRITE_EEPROM: begin
                // Validate parameters
                if (!valid_params({fword_high, fword_low}, pword, waveform)) begin
                    state <= IDLE; // Invalid parameters, return to idle
                end else begin
                    // Sequentially write parameters to EEPROM
                    wrreg_req <= 1'b1; // Trigger write request
                    iic_addr <= base_addr + byte_counter; // Set EEPROM address
                    case (byte_counter)
                        3'd0: iic_wrdata <= fword_high[15:8]; // Write frequency high byte
                        3'd1: iic_wrdata <= fword_high[7:0]; // Write frequency low byte
                        3'd2: iic_wrdata <= fword_low[15:8]; // Write frequency lower high byte
                        3'd3: iic_wrdata <= fword_low[7:0]; // Write frequency lower low byte
                        3'd4: iic_wrdata <= pword[11:4];    // Write phase high byte
                        3'd5: iic_wrdata <= {pword[3:0], waveform[3:0]}; // Write phase low + waveform
                    endcase
                    if (iic_rw_done) begin
                        wrreg_req <= 0; // Clear write request
                        delay_counter <= DELAY_5MS; // Start delay
                        wait_flag <= WRITE_WAIT; // Set write wait flag
                        state <= WAIT_DELAY; // Transition to delay wait state
                        byte_counter <= byte_counter + 1; // Increment byte counter
                        if (byte_counter > 3'd5) begin
                            state <= UPDATE_DDS; // All bytes written, update DDS
                            byte_counter <= 3'd0; // Reset counter
                        end
                    end
                end
            end

            READ_EEPROM: begin
                // Sequentially read parameters from EEPROM
                rdreg_req <= 1'b1; // Trigger read request
                iic_addr <= base_addr + byte_counter; // Set EEPROM address
                case (byte_counter)
                    3'd0: fword_high[15:8] <= iic_rddata; // Read frequency high byte
                    3'd1: fword_high[7:0] <= iic_rddata; // Read frequency low byte
                    3'd2: fword_low[15:8] <= iic_rddata; // Read frequency lower high byte
                    3'd3: fword_low[7:0] <= iic_rddata; // Read frequency lower low byte
                    3'd4: pword[11:4] <= iic_rddata;    // Read phase high byte
                    3'd5: begin
                        pword[3:0] <= iic_rddata[7:4]; // Read phase low bits
                        waveform <= iic_rddata[3:0];  // Read waveform
                    end
                endcase
                if (iic_rw_done) begin
                    rdreg_req <= 0; // Clear read request
                    byte_counter <= byte_counter + 1; // Increment counter
                    if (byte_counter > 3'd5) begin
                        state <= UPDATE_DDS; // All bytes read, update DDS
                        byte_counter <= 3'd0; // Reset counter
                    end
                end
            end

            WAIT_DELAY: begin
                if (delay_counter > 0) begin
                    delay_counter <= delay_counter - 1; // Decrement delay counter
                end else begin
                    // Transition back to appropriate state after delay
                    if (wait_flag == WRITE_WAIT) begin
                        state <= WRITE_EEPROM;
                    end else if (wait_flag == READ_WAIT) begin
                        state <= READ_EEPROM;
                    end
                end
            end

            UPDATE_DDS: begin
                // Update DDS registers with new values
                Fword <= {fword_high, fword_low};
                Pword <= pword;
                Mode_Sel <= waveform[1:0];
                state <= IDLE; // Return to idle state
            end

            LOAD_DEFAULT: begin
                // Load default parameters
                Fword <= DEFAULT_FWORD;
                Pword <= DEFAULT_PWORD;
                Mode_Sel <= DEFAULT_MODE_SEL;
                state <= IDLE;
            end

            default: state <= IDLE; // Default to idle state
        endcase
    end
end

// Function to validate parameters
function valid_params;
    input [31:0] Fword_check; // Frequency word
    input [11:0] Pword_check; // Phase word
    input [1:0] Mode_Sel_check; // Mode selection
    begin
        // Validate Fword, Pword, and Mode_Sel
        valid_params = (Fword_check > 32'd0 && Fword_check <= 32'hFFFFFFFF) &&
                       (Pword_check <= 12'hFFF) &&
                       (Mode_Sel_check <= 2'b10);
    end
endfunction

endmodule