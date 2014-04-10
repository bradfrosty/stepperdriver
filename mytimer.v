// timer.v

module timer(
			pclk,
			nreset,
			bus_write_en, 
			bus_read_en,
			bus_addr,
			bus_write_data, //data_in
			bus_read_data //data_out
            );

input pclk, nreset, bus_write_en, bus_read_en;
input [7:0] bus_addr;
input [31:0] bus_write_data;
output reg [31:0] bus_read_data;

reg [31:0] overflowReg;
reg [31:0] counterReg;
reg [31:0] controlReg;
reg [31:0] nextCounter;
reg overflowReset; // Resets counterReg when new overflow value is written

wire timerEn;   // Timer Enable

//Control Bits
assign timerEn = controlReg[0];

always@(posedge pclk)
if(~nreset)
  begin
    overflowReset <= 1'b0;
    controlReg <= 32'h00000000;
    overflowReg <= 32'h00000000;
  end
else begin
	if(bus_write_en) begin : WRITE
		case(bus_addr[3:2])
            2'b00: // Timer Overflow Register
                begin 
                overflowReg <= bus_write_data;
                overflowReset <= 1'b1;
                end
            2'b01: // Timer Value, Read Only
                begin 
                overflowReset <= 1'b0;
                end
            2'b10: // Timer Control
                begin 
                controlReg <= bus_write_data;
                overflowReset <= 1'b0;
                end
            2'b11: // Spare
                begin 
                overflowReset <= 1'b0;
                end
        endcase
    end
	else if(bus_read_en) begin : READ
         case(bus_addr[3:2])
	    	2'b00: // Timer Overflow register
                begin 
		bus_read_data <= overflowReg;
 		end
            2'b01: // Timer Value, Read Only
                begin 
                bus_read_data <= counterReg;	
		end
            2'b10: // Timer Control
                begin 
                bus_read_data <= controlReg;
	    end
            2'b11: // Spare
                begin 
                end
          endcase
     end
	 else
		overflowReset <= 1'b0;
end

assign timerEn = controlReg[0];

always@*
    nextCounter <= counterReg + 1;

always@(posedge pclk)
if(~nreset)
begin
	counterReg <= 32'h00000000;
end
else begin
    if(overflowReset)
    begin
	counterReg <= 32'h00000000;
    end
    else if(timerEn)
    begin
	if(counterReg == overflowReg)
		counterReg <= 32'h00000000;
	else
	    	counterReg <= nextCounter;
    end
end
endmodule





// timerWrapper.v

module timerWrapper(// APB Bus Interface

				PCLK,
				PENABLE,
				PSEL,
				PRESETN,
				PWRITE,
				PREADY,
				PSLVERR,
				PADDR,
				PWDATA,
				PRDATA,
				// Test Interface
				TPS);

// APB Bus Interface
input PCLK,PENABLE, PSEL, PRESETN, PWRITE;
input  [31:0] PWDATA;
input  [7:0] PADDR;
output [31:0] PRDATA;
output PREADY, PSLVERR;

// Test Interface
output [4:0] TPS; // Use for your debugging

assign BUS_WRITE_EN = (PENABLE && PWRITE && PSEL);
assign BUS_READ_EN = (!PWRITE && PSEL); //Data is ready during first cycle to make it availble on the bus when PENABLE is asserted

assign PREADY = 1'b1;
assign PSLVERR = 1'b0;

timer timer_0(	.pclk(PCLK),
			    .nreset(PRESETN), 
			    .bus_write_en(BUS_WRITE_EN),
			    .bus_read_en(BUS_READ_EN),
			    .bus_addr(PADDR),
			    .bus_write_data(PWDATA),
			    .bus_read_data(PRDATA)
			);

endmodule

