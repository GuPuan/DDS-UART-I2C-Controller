`timescale 1ns / 1ps

module DDS_Test_tb;
    
    reg Clk;
    reg  Reset_n;
    reg [1:0] Mode_Sel;
    reg [31:0]Fword;  
    reg [11:0]Pword;
    wire [13:0]Data;   
    
    DDS_Module DDS_Module(
        .Clk(Clk),
        .Reset_n(Reset_n),
        .Mode_Sel(Mode_Sel),
        .Fword(Fword),
        .Pword(Pword),
        .Data(Data)
    );
    
    initial Clk = 1;
    always #10 Clk = ~Clk;
    
    initial begin
        Reset_n = 0;
        Mode_Sel = 2'b00;
        Fword = 32'h00A00000;     
        Pword = 12'h000;
        
        #201;
        Reset_n = 1;
        #100000;
        
        Mode_Sel = 2'b10;
        Fword = 32'h00000A0;     
        Pword = 12'h0AA;
        #100000;
        
        Mode_Sel = 2'b00;
        Fword = 32'h00A00000;     
        Pword = 12'h000;
        #100000;
        
        Mode_Sel = 2'b10;
        Fword= 32'h010B00B;     
        Pword=12'h0AAA;
        #100000;
        $stop;        
    end
    
    
endmodule