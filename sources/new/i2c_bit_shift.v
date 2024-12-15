module i2c_bit_shift(
    Clk,
    Rst_n,
    
    Cmd,
    Go,
    Rx_DATA,
    Tx_DATA,
    Trans_Done,
    ack_o,
    i2c_sclk,
    i2c_sdat
);
    // Port declarations
    input Clk;              // System clock
    input Rst_n;            // Active-low reset

    input [5:0]Cmd;         // I2C command input
    input Go;               // Start signal
    output reg [7:0]Rx_DATA; // Received data output
    input [7:0]Tx_DATA;     // Transmit data input
    output reg Trans_Done;  // Transfer done signal
    output reg ack_o;       // Acknowledge output
    output reg i2c_sclk;    // I2C clock line
    inout i2c_sdat;         // I2C data line (bidirectional)

    reg i2c_sdat_o;         // I2C data output control signal

    // Clock parameters
    parameter SYS_CLOCK = 50_000_000;  // System clock frequency (50 MHz)
    parameter SCL_CLOCK = 400_000;     // I2C clock frequency (400 kHz)
    localparam SCL_CNT_M = SYS_CLOCK / SCL_CLOCK / 4 - 1; // Clock divider count for SCL

    reg i2c_sdat_oe;        // I2C data output enable signal

    // Command definitions
    localparam 
        WR   = 6'b000001,   // Write command
        STA  = 6'b000010,   // Start condition
        RD   = 6'b000100,   // Read command
        STO  = 6'b001000,   // Stop condition
        ACK  = 6'b010000,   // Acknowledge signal
        NACK = 6'b100000;   // No acknowledge signal

    // Clock divider counter
    reg [19:0]div_cnt;
    reg en_div_cnt;         // Enable clock divider counter
    always @(posedge Clk or negedge Rst_n)
        if (!Rst_n)
            div_cnt <= 20'd0;
        else if (en_div_cnt) begin
            if (div_cnt < SCL_CNT_M)
                div_cnt <= div_cnt + 1'b1;
            else
                div_cnt <= 0;
        end else
            div_cnt <= 0;

    wire sclk_plus = div_cnt == SCL_CNT_M; // SCL clock pulse signal

    // I2C data line assignment
    assign i2c_sdat = !i2c_sdat_o && i2c_sdat_oe ? 1'b0 : 1'bz;

    // State machine definitions
    reg [7:0]state;

    localparam
        IDLE      = 8'b00000001, // Idle state
        GEN_STA   = 8'b00000010, // Generate start condition
        WR_DATA   = 8'b00000100, // Write data state
        RD_DATA   = 8'b00001000, // Read data state
        CHECK_ACK = 8'b00010000, // Check acknowledge
        GEN_ACK   = 8'b00100000, // Generate acknowledge
        GEN_STO   = 8'b01000000; // Generate stop condition

    reg [4:0]cnt;           // Counter for bit operations

    // State machine logic
    always @(posedge Clk or negedge Rst_n)
        if (!Rst_n) begin
            // Reset logic
            Rx_DATA <= 0;
            i2c_sdat_oe <= 1'd0;
            en_div_cnt <= 1'b0;
            i2c_sdat_o <= 1'd1;
            Trans_Done <= 1'b0;
            ack_o <= 0;
            state <= IDLE;
            cnt <= 0;
        end else begin
            case (state)
                IDLE: begin
                    // Idle state
                    Trans_Done <= 1'b0;
                    i2c_sdat_oe <= 1'd1;
                    if (Go) begin
                        en_div_cnt <= 1'b1;
                        if (Cmd & STA)
                            state <= GEN_STA;
                        else if (Cmd & WR)
                            state <= WR_DATA;
                        else if (Cmd & RD)
                            state <= RD_DATA;
                        else
                            state <= IDLE;
                    end else begin
                        en_div_cnt <= 1'b0;
                        state <= IDLE;
                    end
                end

                GEN_STA: begin
                    // Generate start condition
                    if (sclk_plus) begin
                        if (cnt == 3)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0: begin i2c_sdat_o <= 1; i2c_sdat_oe <= 1'd1; end
                            1: begin i2c_sclk <= 1; end
                            2: begin i2c_sdat_o <= 0; i2c_sclk <= 1; end
                            3: begin i2c_sclk <= 0; end
                        endcase
                        if (cnt == 3) begin
                            if (Cmd & WR)
                                state <= WR_DATA;
                            else if (Cmd & RD)
                                state <= RD_DATA;
                        end
                    end
                end

                WR_DATA: begin
                    // Write data state
                    if (sclk_plus) begin
                        if (cnt == 31)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0,4,8,12,16,20,24,28: begin i2c_sdat_o <= Tx_DATA[7-cnt[4:2]]; i2c_sdat_oe <= 1'd1; end // Set data
                            1,5,9,13,17,21,25,29: begin i2c_sclk <= 1; end // SCL positive edge
                            2,6,10,14,18,22,26,30: begin i2c_sclk <= 1; end // SCL keep high
                            3,7,11,15,19,23,27,31: begin i2c_sclk <= 0; end // SCL negative edge
                        endcase
                        if (cnt == 31)
                            state <= CHECK_ACK;
                    end
                end

                RD_DATA: begin
                    // Read data state
                    if (sclk_plus) begin
                        if (cnt == 31)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0,4,8,12,16,20,24,28: begin i2c_sdat_oe <= 1'd0; i2c_sclk <= 0; end // Prepare for data reception
                            1,5,9,13,17,21,25,29: begin i2c_sclk <= 1; end // SCL positive edge
                            2,6,10,14,18,22,26,30: begin i2c_sclk <= 1; Rx_DATA <= {Rx_DATA[6:0], i2c_sdat}; end // Receive data
                            3,7,11,15,19,23,27,31: begin i2c_sclk <= 0; end // SCL negative edge
                        endcase
                        if (cnt == 31)
                            state <= GEN_ACK;
                    end
                end

                CHECK_ACK: begin
                    // Check for acknowledge signal
                    if (sclk_plus) begin
                        if (cnt == 3)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0: begin i2c_sdat_oe <= 1'd0; i2c_sclk <= 0; end
                            1: begin i2c_sclk <= 1; end
                            2: begin ack_o <= i2c_sdat; i2c_sclk <= 1; end
                            3: begin i2c_sclk <= 0; end
                        endcase
                        if (cnt == 3) begin
                            if (Cmd & STO)
                                state <= GEN_STO;
                            else begin
                                state <= IDLE;
                                Trans_Done <= 1'b1;
                            end
                        end
                    end
                end

                GEN_ACK: begin
                    // Generate acknowledge signal
                    if (sclk_plus) begin
                        if (cnt == 3)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0: begin
                                i2c_sdat_oe <= 1'd1;
                                i2c_sclk <= 0;
                                if (Cmd & ACK)
                                    i2c_sdat_o <= 1'b0;
                                else if (Cmd & NACK)
                                    i2c_sdat_o <= 1'b1;
                            end
                            1: begin i2c_sclk <= 1; end
                            2: begin i2c_sclk <= 1; end
                            3: begin i2c_sclk <= 0; end
                        endcase
                        if (cnt == 3) begin
                            if (Cmd & STO)
                                state <= GEN_STO;
                            else begin
                                state <= IDLE;
                                Trans_Done <= 1'b1;
                            end
                        end
                    end
                end

                GEN_STO: begin
                    // Generate stop condition
                    if (sclk_plus) begin
                        if (cnt == 3)
                            cnt <= 0;
                        else
                            cnt <= cnt + 1'b1;
                        case (cnt)
                            0: begin i2c_sdat_o <= 0; i2c_sdat_oe <= 1'd1; end
                            1: begin i2c_sclk <= 1; end
                            2: begin i2c_sdat_o <= 1; i2c_sclk <= 1; end
                            3: begin i2c_sclk <= 1; end
                        endcase
                        if (cnt == 3) begin
                            Trans_Done <= 1'b1;
                            state <= IDLE;
                        end
                    end
                end

                default: state <= IDLE; // Default state
            endcase
        end

endmodule