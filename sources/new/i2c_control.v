module i2c_control(
    input Clk,                   // System clock
    input Rst_n,                 // Active-low reset

    input wrreg_req,             // Write register request
    input rdreg_req,             // Read register request
    input [15:0] addr,           // Address to read/write
    input addr_mode,             // Address mode: 0 = 8-bit, 1 = 16-bit
    input [7:0] wrdata,          // Data to write
    output reg [7:0] rddata,     // Data read from I2C device
    input [7:0] device_id,       // I2C device ID
    output reg RW_Done,          // Read/Write operation completion flag

    output reg ack,              // Acknowledge flag

    output i2c_sclk,             // I2C clock signal
    inout i2c_sdat               // I2C data signal
);

    // Internal signals
    reg [5:0] Cmd;               // Command register for I2C operations
    reg [7:0] Tx_DATA;           // Data to transmit
    wire Trans_Done;             // Transaction completion signal
    wire ack_o;                  // Acknowledge signal from I2C
    reg Go;                      // Start signal for I2C transactions
    wire [15:0] reg_addr;        // Internal register for address in the correct order

    // Assign address order based on address mode
    assign reg_addr = addr_mode ? addr : {addr[7:0], addr[15:8]};

    // Data received from I2C
    wire [7:0] Rx_DATA;

    // I2C command definitions
    localparam 
        WR   = 6'b000001,   // Write operation
        STA  = 6'b000010,   // Start condition
        RD   = 6'b000100,   // Read operation
        STO  = 6'b001000,   // Stop condition
        ACK  = 6'b010000,   // Acknowledge condition
        NACK = 6'b100000;   // No Acknowledge condition

    // Instantiate the I2C bit shift module
    i2c_bit_shift i2c_bit_shift(
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

    // State machine definition
    reg [6:0] state; // Current state
    reg [7:0] cnt;   // Counter for write/read steps

    // State definitions
    localparam
        IDLE         = 7'b0000001, // Idle state
        WR_REG       = 7'b0000010, // Write register state
        WAIT_WR_DONE = 7'b0000100, // Wait for write completion
        WR_REG_DONE  = 7'b0001000, // Write operation completed
        RD_REG       = 7'b0010000, // Read register state
        WAIT_RD_DONE = 7'b0100000, // Wait for read completion
        RD_REG_DONE  = 7'b1000000; // Read operation completed

    // Main state machine
    always @(posedge Clk or negedge Rst_n)
    if (!Rst_n) begin
        // Reset all registers
        Cmd <= 6'd0;
        Tx_DATA <= 8'd0;
        Go <= 1'b0;
        rddata <= 0;
        state <= IDLE;
        ack <= 0;
    end else begin
        case (state)
            IDLE: begin
                // Reset state and check for read/write requests
                cnt <= 0;
                ack <= 0;
                RW_Done <= 1'b0;                    
                if (wrreg_req)
                    state <= WR_REG;
                else if (rdreg_req)
                    state <= RD_REG;
                else
                    state <= IDLE;
            end

            WR_REG: begin
                // Initiate write operations based on current step (cnt)
                state <= WAIT_WR_DONE;
                case (cnt)
                    0: write_byte(WR | STA, device_id);      // Start and send device ID
                    1: write_byte(WR, reg_addr[15:8]);       // Write high byte of address
                    2: write_byte(WR, reg_addr[7:0]);       // Write low byte of address
                    3: write_byte(WR | STO, wrdata);        // Write data and stop
                    default: ;
                endcase
            end

            WAIT_WR_DONE: begin
                // Wait for write operation to complete
                Go <= 1'b0; 
                if (Trans_Done) begin
                    ack <= ack | ack_o; // Accumulate acknowledge signal
                    case (cnt)
                        0: begin cnt <= 1; state <= WR_REG; end
                        1: begin 
                                state <= WR_REG; 
                                cnt <= addr_mode ? 2 : 3; 
                           end
                        2: begin cnt <= 3; state <= WR_REG; end
                        3: state <= WR_REG_DONE;
                        default: state <= IDLE;
                    endcase
                end
            end

            WR_REG_DONE: begin
                // Write operation completed
                RW_Done <= 1'b1;
                state <= IDLE;
            end
                
            RD_REG: begin
                // Initiate read operations based on current step (cnt)
                state <= WAIT_RD_DONE;
                case (cnt)
                    0: write_byte(WR | STA, device_id);       // Start and send device ID
                    1: write_byte(WR, reg_addr[15:8]);        // Write high byte of address
                    2: write_byte(WR, reg_addr[7:0]);        // Write low byte of address
                    3: write_byte(WR | STA, device_id | 8'd1); // Start and prepare to read
                    4: read_byte(RD | NACK | STO);           // Read data with no acknowledge and stop
                    default: ;
                endcase
            end
                
            WAIT_RD_DONE: begin
                // Wait for read operation to complete
                Go <= 1'b0; 
                if (Trans_Done) begin
                    if (cnt <= 3)
                        ack <= ack | ack_o;
                    case (cnt)
                        0: begin cnt <= 1; state <= RD_REG; end
                        1: begin 
                                state <= RD_REG; 
                                cnt <= addr_mode ? 2 : 3; 
                           end
                        2: begin cnt <= 3; state <= RD_REG; end
                        3: begin cnt <= 4; state <= RD_REG; end
                        4: state <= RD_REG_DONE;
                        default: state <= IDLE;
                    endcase
                end
            end
                
            RD_REG_DONE: begin
                // Read operation completed
                RW_Done <= 1'b1;
                rddata <= Rx_DATA; // Capture received data
                state <= IDLE;                
            end

            default: state <= IDLE;
        endcase
    end

    // Task to initiate a read operation
    task read_byte;
        input [5:0] Ctrl_Cmd;
        begin
            Cmd <= Ctrl_Cmd;
            Go <= 1'b1; 
        end
    endtask

    // Task to initiate a write operation
    task write_byte;
        input [5:0] Ctrl_Cmd;
        input [7:0] Wr_Byte_Data;
        begin
            Cmd <= Ctrl_Cmd;
            Tx_DATA <= Wr_Byte_Data;
            Go <= 1'b1; 
        end
    endtask

endmodule