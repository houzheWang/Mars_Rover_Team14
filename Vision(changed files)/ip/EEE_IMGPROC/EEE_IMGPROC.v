module EEE_IMGPROC(
	// global clock & reset
	clk,
	reset_n,
	
	// mm slave
	s_chipselect,
	s_read,
	s_write,
	s_readdata,
	s_writedata,
	s_address,

	// stream sink
	sink_data,
	sink_valid,
	sink_ready,
	sink_sop,
	sink_eop,
	
	// streaming source
	source_data,
	source_valid,
	source_ready,
	source_sop,
	source_eop,
	
	// conduit
	mode
	
);


// global clock & reset
input	clk;
input	reset_n;

// mm slave
input							s_chipselect;
input							s_read;
input							s_write;
output	reg	[31:0]	s_readdata;
input	[31:0]				s_writedata;
input	[2:0]					s_address;


// streaming sink
input	[23:0]            	sink_data;
input								sink_valid;
output							sink_ready;
input								sink_sop;
input								sink_eop;

// streaming source
output	[23:0]			  	   source_data;
output								source_valid;
input									source_ready;
output								source_sop;
output								source_eop;

// conduit export
input                         mode;

////////////////////////////////////////////////////////////////////////
//
parameter IMAGE_W = 11'd640;
parameter IMAGE_H = 11'd480;
parameter MESSAGE_BUF_MAX = 256;
parameter MSG_INTERVAL = 6;
parameter BB_COL_DEFAULT = 24'h00ff00;

wire [15:0]   max_11, max, min, min_11, diff, H , S, V;
wire [7:0]   red, green, blue, grey;
wire [7:0]   red_out, green_out, blue_out;

wire         sop, eop, in_valid, out_ready;
////////////////////////////////////////////////////////////////////////

// Detect red areas
//detect
wire red_detect;
wire yellow_detect;
wire blue_detect;
wire green_detect;
wire dark_green_detect;
wire pink_detect;

assign max_11 = ({8'b0,red}>{8'b0,green})? {8'b0,red}:{8'b0,green};
assign max = (max_11>{8'b0,blue})? max_11:{8'b0,blue};
assign min_11 = ({8'b0,red}<{8'b0,green})? {8'b0,red}:{8'b0,green};
assign min = (min_11<{8'b0,blue})? min_11:{8'b0,blue};
assign diff = max - min;
assign H = (diff == 0)? 0:
		   (max == {8'b0,red})?
				({8'b0,green}>{8'b0,blue})? (((60)*({8'b0,green}-{8'b0,blue}))/diff) : (((60)*({8'b0,blue}-{8'b0,green}))/diff)
				:
		   (max == {8'b0,green})? 
				({8'b0,blue}>{8'b0,red})? (((60)*({8'b0,blue}-{8'b0,red}))/diff+120): (((60)*({8'b0,red}-{8'b0,blue}))/diff+120)
				:
		   (max == {8'b0,blue})? 
				({8'b0,red}>{8'b0,green})? (((60)*({8'b0,red}-{8'b0,green}))/diff+240): (((60)*({8'b0,green}-{8'b0,red}))/diff+240)
				:0;
assign S = (max != 0)? (((16'hff)*diff)/max) : 0; 
assign V = max;

//red
wire compareflagRH = ((H>(340)) || (H<(32))) ? 1:0;
wire compareflagRS = ((S>(120)) && (S<(255))) ? 1:0;
wire compareflagRV = ((V>(135))) ? 1:0;
assign red_detect = compareflagRH && compareflagRS && compareflagRV && (~pink_detect);

// yellow
wire compareflagYH = ((H>(43) && H<(83)))? 1:0;
wire compareflagYS = (S>(160))? 1:0;
wire compareflagYV = (V>(130))? 1:0;
assign yellow_detect = compareflagYH && compareflagYS && compareflagYV;

//Blue
wire compareflagBH = ((H>(190)) && (H<(250)))? 1:0;
wire compareflagBS = ((S>(36)) && (S<(213)))? 1:0;
wire compareflagBV = ((V>(16)) && (V<(164)))? 1:0;
assign blue_detect = compareflagBH && compareflagBS && compareflagBV;

//Green
wire compareflagGH = ((H>(88)) && (H<(138)))? 1:0;
wire compareflagGS = ((S>(115)) && (S<(197)))? 1:0;
wire compareflagGV = (V>(105))? 1:0;
assign green_detect = compareflagGH && compareflagGS && compareflagGV;

//Dark Green
wire compareflagDGH = ((H>(105)) && (H<(165)))? 1:0;
wire compareflagDGS = ((S>(81)) && (S<(255)))? 1:0;
wire compareflagDGV = ((V>(36)) && (V<(136)))? 1:0;
assign dark_green_detect = compareflagDGH && compareflagDGS && compareflagDGV;

//Pink
wire compareflagPH = ((H>(0)) && (H<(36)))? 1:0;
wire compareflagPS = ((S>(110)) && (S<(200)))? 1:0;
wire compareflagPV = (V>(173))? 1:0;
assign pink_detect = compareflagPH && compareflagPS && compareflagPV;

// Find boundary of cursor box

//filter
reg red_r1, red_r2, red_r3, red_r4, red_r5, red_r6;
wire red_total;

reg yellow_r1, yellow_r2, yellow_r3, yellow_r4, yellow_r5, yellow_r6;
wire yellow_total;

reg blue_r1, blue_r2, blue_r3, blue_r4, blue_r5, blue_r6;
wire blue_total;

reg green_r1, green_r2, green_r3, green_r4, green_r5, green_r6;
wire green_total;

reg dark_green_r1, dark_green_r2, dark_green_r3, dark_green_r4, dark_green_r5, dark_green_r6;
wire dark_green_total;

reg pink_r1, pink_r2, pink_r3, pink_r4, pink_r5, pink_r6;
wire pink_total;

always@(posedge clk) begin
	red_r1 <= red_detect;
	red_r2 <= red_r1;
	red_r3 <= red_r2;
	red_r4 <= red_r3;
	// red_r5 <= red_r4;
	// red_r6 <= red_r5;

	yellow_r1 <= yellow_detect;
 	yellow_r2 <= yellow_r1;
 	yellow_r3 <= yellow_r2;
 	yellow_r4 <= yellow_r3;
	// yellow_r5 <= yellow_r4;
	// yellow_r6 <= yellow_r5;

	blue_r1 <= blue_detect;
 	blue_r2 <= blue_r1;
 	blue_r3 <= blue_r2;
 	blue_r4 <= blue_r3;
	// blue_r5 <= blue_r4;
	// blue_r6 <= blue_r5;

	green_r1 <= green_detect;
 	green_r2 <= green_r1;
 	green_r3 <= green_r2;
 	green_r4 <= green_r3;
	// green_r5 <= green_r4;
	// green_r6 <= green_r5;

	dark_green_r1 <= dark_green_detect;
 	dark_green_r2 <= dark_green_r1;
 	dark_green_r3 <= dark_green_r2;
 	dark_green_r4 <= dark_green_r3;
	// dark_green_r5 <= dark_green_r4;
	// dark_green_r6 <= dark_green_r5;
	
	pink_r1 <= pink_detect;
 	pink_r2 <= pink_r1;
 	pink_r3 <= pink_r2;
 	pink_r4 <= pink_r3;
	// pink_r5 <= pink_r4;
	// pink_r6 <= pink_r5;

end
assign red_total = (red_r1 & red_r2 & red_r3 & red_r4 & red_detect);
assign yellow_total = (yellow_r1 & yellow_r2 & yellow_r3 & yellow_r4 & yellow_detect);
assign blue_total = (blue_r1 & blue_r2 & blue_r3 & blue_r4 & blue_detect);
assign green_total = (green_r1 & green_r2 & green_r3 & green_r4 & green_detect);
assign dark_green_total = (dark_green_r1 & dark_green_r2 & dark_green_r3 & dark_green_r4 & dark_green_detect);
assign pink_total = (pink_r1 & pink_r2 & pink_r3 & pink_r4 & pink_detect);

// Find boundary of cursor box
// Highlight detected areas
wire [23:0] color_high;
assign grey = green[7:1] + red[7:2] + blue[7:2]; //Grey = green/2 + red/4 + blue/4
assign color_high  =  red_total ? {8'hff, 8'h0, 8'h0} : 
					yellow_total ? {8'hff, 8'hff, 8'h0} : 
					blue_total ? {8'h0, 8'h0, 8'hff} :
					green_total ? {8'h7f, 8'hff, 8'h00} :
					dark_green_total ? {8'h0, 8'h64, 8'h0} :
					pink_total ? {8'hff, 8'hb6, 8'hc1} :
					{grey, grey, grey};

// Show bounding box
wire [23:0] new_image;
wire bb_active;
assign bb_active = (x == left) | (x == right);
assign new_image = bb_active ? bb_col : color_high;

// Switch output pixels depending on mode switch
// Don't modify the start-of-packet word - it's a packet discriptor
// Don't modify data in non-video packets
assign {red_out, green_out, blue_out} = (mode & ~sop & packet_video) ? new_image : {red,green,blue};

//Count valid pixels to tget the image coordinates. Reset and detect packet type on Start of Packet.
reg [10:0] x_red, y_red;
reg [10:0] x_green, y_green;
reg [10:0] x_blue, y_blue;
reg [10:0] x_dark_green, y_dark_green;
reg [10:0] x_yellow, y_yellow;
reg [10:0] x_pink, y_pink;

reg packet_video;

reg [10:0] x, y;
always@(posedge clk) begin
	if (sop) begin
		x <= 11'h0;
		y <= 11'h0;
		packet_video <= (blue[3:0] == 3'h0);
	end
	else if (in_valid) begin
		if (x == IMAGE_W-1) begin
			x <= 11'h0;
			y <= y + 11'h1;
		end
		else begin
			x <= x + 11'h1;
		end
	end
end

//Find first and last red pixels
reg [10:0] x_min_red, y_min_red, x_max_red, y_max_red;
reg [10:0] x_min_green, y_min_green, x_max_green, y_max_green;
reg [10:0] x_min_blue, y_min_blue, x_max_blue, y_max_blue;
reg [10:0] x_min_dark_green, y_min_dark_green, x_max_dark_green, y_max_dark_green;
reg [10:0] x_min_yellow, y_min_yellow, x_max_yellow, y_max_yellow;
reg [10:0] x_min_pink, y_min_pink, x_max_pink, y_max_pink;

always@(posedge clk) begin
	if (red_total & in_valid) begin	//Update bounds when the pixel is red
		if (x < x_min_red) x_min_red <= x;
		if (x > x_max_red) x_max_red <= x;
		// if (y < y_min_red) y_min_red <= y;
		// y_max_red <= y;
	end
	
	if (green_total & in_valid) begin	//Update bounds when the pixel is green
	 	if (x < x_min_green) x_min_green <= x;
	 	if (x > x_max_green) x_max_green <= x;
	 	// if (y < y_min_green) y_min_green <= y;
	 	// y_max_green <= y;
	end

	if (blue_total & in_valid) begin	//Update bounds when the pixel is blue
	 	if (x < x_min_blue) x_min_blue <= x;
	 	if (x > x_max_blue) x_max_blue <= x;
	 	// if (y < y_min_blue) y_min_blue <= y;
	 	// y_max_blue <= y;
	end

	if (dark_green_total & in_valid) begin	//Update bounds when the pixel is dark_green
	 	if (x < x_min_dark_green) x_min_dark_green <= x;
	 	if (x > x_max_dark_green) x_max_dark_green <= x;
	 	// if (y < y_min_dark_green) y_min_dark_green <= y;
	 	// y_max_dark_green <= y;
	end

	if (pink_total & in_valid) begin	//Update bounds when the pixel is pink
	 	if (x < x_min_pink) x_min_pink <= x;
	 	if (x > x_max_pink) x_max_pink <= x;
	 	// if (y < y_min_pink) y_min_pink <= y;
	 	// y_max_pink <= y;
	end

	if (yellow_total & in_valid) begin	//Update bounds when the pixel is yellow
	 	if (x < x_min_yellow) x_min_yellow <= x;
	 	if (x > x_max_yellow) x_max_yellow <= x;
	 	// if (y < y_min_yellow) y_min_yellow <= y;
	 	// y_max_yellow <= y;
	end

	if (sop & in_valid) begin	//Reset bounds on start of packet
		x_min_red <= IMAGE_W-11'h1;
		x_max_red<= 0;
		// y_min_red <= IMAGE_H-11'h1;
		// y_max_red<= 0;

		x_min_green <= IMAGE_W-11'h1;
		x_max_green<= 0;
		// y_min_green <= IMAGE_H-11'h1;
		// y_max_green<= 0;

		x_min_blue <= IMAGE_W-11'h1;
		x_max_blue <= 0;
		// y_min_blue <= IMAGE_H-11'h1;
		// y_max_blue<= 0;

		x_min_dark_green <= IMAGE_W-11'h1;
		x_max_dark_green<= 0;
		// y_min_dark_green <= IMAGE_H-11'h1;
		// y_max_dark_green<= 0;

		x_min_yellow <= IMAGE_W-11'h1;
		x_max_yellow<= 0;
		// y_min_yellow <= IMAGE_H-11'h1;
		// y_max_yellow<= 0;

		x_min_pink <= IMAGE_W-11'h1;
		x_max_pink<= 0;
		// y_min_pink <= IMAGE_H-11'h1;
		// y_max_pink<= 0;
	end
end

//Process bounding box at the end of the frame.
reg [1:0] msg_state;
reg [10:0] left, right, top, bottom;
reg [7:0] frame_count;
always@(posedge clk) begin
	if (eop & in_valid & packet_video) begin  //Ignore non-video packets
		
		//Latch edges for display overlay on next frame
		left <= x_min_red;
		right <= x_max_red;
		// top <= y_min_red;
		// bottom <= y_max_red;
		
		
		//Start message writer FSM once every MSG_INTERVAL frames, if there is room in the FIFO
		frame_count <= frame_count - 1;
		
		if (frame_count == 0 && msg_buf_size < MESSAGE_BUF_MAX - 3) begin
			msg_state <= 2'b01;
			frame_count <= MSG_INTERVAL-1;
		end
	end
	
	//Cycle through message writer states once started
	if (msg_state != 2'b00) msg_state <= msg_state + 2'b01;

end
	
//Generate output messages for CPU
reg [31:0] msg_buf_in; 
wire [31:0] msg_buf_out;
reg msg_buf_wr;
wire msg_buf_rd, msg_buf_flush;
wire [7:0] msg_buf_size;
wire msg_buf_empty;

// `define RED_BOX_MSG_ID "RBB"
reg [7:0] the_color;
reg [10:0] x_min, x_max, y_min, y_max;
reg [10:0] max_width, red_width, green_width, blue_width, dark_green_width, pink_width, yellow_width, max_1, max_2, max_3, max_4;

always@(posedge clk) begin
	red_width <= x_max_red - x_min_red;
	green_width <= x_max_green - x_min_green;
	blue_width <= x_max_blue - x_min_blue;
	dark_green_width <= x_max_dark_green - x_min_dark_green;
	pink_width <= x_max_pink - x_min_pink;
	yellow_width <= x_max_yellow - x_min_yellow;

	max_1 <= (red_width > green_width)? red_width : green_width;
	max_2 <= (max_1 > blue_width)? max_1 : blue_width;
	max_3 <= (max_2 > dark_green_width)? max_2 : dark_green_width;
	max_4 <= (max_3 > pink_width)? max_3 : pink_width;
	max_width <= (max_4 > yellow_width)? max_4 : yellow_width;

	

	if (max_width == red_width && red_total) begin
 		the_color <= 8'b1;
 		x_min <= x_min_red;
 		x_max <= x_max_red;
		// y_min <= y_min_red;
		// y_max <= y_max_red;
	end
	if (max_width == green_width && green_total) begin
 		the_color <= 8'b10;
 		x_min <= x_min_green;
 		x_max <= x_max_green;
 		// y_min <= y_min_green;
 		// y_max <= y_max_green;
 	end
 	if (max_width == blue_width && blue_total) begin
 		the_color <= 8'b11;
 		x_min <= x_min_blue;
 		x_max <= x_max_blue;
		// y_min <= y_min_blue;
 		// y_max <= y_max_blue;
 	end
 	if (max_width == dark_green_width && dark_green_total) begin
 		the_color <= 8'b100;
 		x_min <= x_min_dark_green;
 		x_max <= x_max_dark_green;
		// y_min <= y_min_dark_green;
 		// y_max <= y_max_dark_green;
	end
 	if (max_width == yellow_width && yellow_total) begin
 		the_color <= 8'b101;
 		x_min <= x_min_yellow;
 		x_max <= x_max_yellow;
 		// y_min <= y_min_yellow;
 		// y_max <= y_max_yellow;
	end
 	if (max_width == pink_width && pink_total) begin
 		the_color <= 8'b110;
 		x_min <= x_min_pink;
 		x_max <= x_max_pink;
 		// y_min <= y_min_pink;
 		// y_max <= y_max_pink;
 	end
	// if (max_width > 11'h276) begin
	// // if(~red_total && ~green_total && ~blue_total && ~dark_green_total && ~yellow_total && ~pink_total) begin
 	  	// the_color <= 32'b0;
	  	// x_min <= 11'b0;
	  	// x_max <= 11'b0;
	// // 	// y_min <= 11'b0;
	// // 	// y_max <= 11'b0;
//	end
end
wire stop;
wire [10:0] mid;
wire [10:0] final_width;
//if(x_max - x_min == 0 || x_min>x_max ){
//	the_color = 0;
//	x_max = 0;
//	x_min = 0;
//}
assign mid = x_max/2 + x_min/2;
assign final_width = (x_max>x_min) ? (x_max-x_min) :0;
assign stop = (((x_max - x_min) > 8'h28))? 1:0;
always@(*) begin	//Write words to FIFO as state machine advances
	case(msg_state)
		2'b00: begin
			msg_buf_in = 32'b0;
			msg_buf_wr = 1'b0;
		end
		2'b01: begin
			msg_buf_in = {7'b0, stop, 24'b111111111111111111111111};	//Message ID
			msg_buf_wr = 1'b1;
		end
		2'b10: begin
			msg_buf_in = {5'b0,mid, 5'b0, final_width};	//mid-point and width
			msg_buf_wr = 1'b1;
		end
		2'b11: begin
			msg_buf_in = {the_color,24'b0}; //Bottom right coordinate
			msg_buf_wr = 1'b1;
		end
	endcase	
end

//Output message FIFO
MSG_FIFO	MSG_FIFO_inst (
	.clock (clk),
	.data (msg_buf_in),
	.rdreq (msg_buf_rd),
	.sclr (~reset_n | msg_buf_flush),
	.wrreq (msg_buf_wr),
	.q (msg_buf_out),
	.usedw (msg_buf_size),
	.empty (msg_buf_empty)
	);


//Streaming registers to buffer video signal
STREAM_REG #(.DATA_WIDTH(26)) in_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(sink_ready),
	.valid_out(in_valid),
	.data_out({red,green,blue,sop,eop}),
	.ready_in(out_ready),
	.valid_in(sink_valid),
	.data_in({sink_data,sink_sop,sink_eop})
);

STREAM_REG #(.DATA_WIDTH(26)) out_reg (
	.clk(clk),
	.rst_n(reset_n),
	.ready_out(out_ready),
	.valid_out(source_valid),
	.data_out({source_data,source_sop,source_eop}),
	.ready_in(source_ready),
	.valid_in(in_valid),
	.data_in({red_out, green_out, blue_out, sop, eop})
);


/////////////////////////////////
/// Memory-mapped port		 /////
/////////////////////////////////

// Addresses
`define REG_STATUS    			0
`define READ_MSG    				1
`define READ_ID    				2
`define REG_BBCOL					3

//Status register bits
// 31:16 - unimplemented
// 15:8 - number of words in message buffer (read only)
// 7:5 - unused
// 4 - flush message buffer (write only - read as 0)
// 3:0 - unused


// Process write

reg  [7:0]   reg_status;
reg	[23:0]	bb_col;

always @ (posedge clk)
begin
	if (~reset_n)
	begin
		reg_status <= 8'b0;
		bb_col <= BB_COL_DEFAULT;
	end
	else begin
		if(s_chipselect & s_write) begin
		   if      (s_address == `REG_STATUS)	reg_status <= s_writedata[7:0];
		   if      (s_address == `REG_BBCOL)	bb_col <= s_writedata[23:0];
		end
	end
end


//Flush the message buffer if 1 is written to status register bit 4
assign msg_buf_flush = (s_chipselect & s_write & (s_address == `REG_STATUS) & s_writedata[4]);


// Process reads
reg read_d; //Store the read signal for correct updating of the message buffer

// Copy the requested word to the output port when there is a read.
always @ (posedge clk)
begin
   if (~reset_n) begin
	   s_readdata <= {32'b0};
		read_d <= 1'b0;
	end
	
	else if (s_chipselect & s_read) begin
		if   (s_address == `REG_STATUS) s_readdata <= {16'b0,msg_buf_size,reg_status};
		if   (s_address == `READ_MSG) s_readdata <= {msg_buf_out};
		if   (s_address == `READ_ID) s_readdata <= 32'h1234EEE2;
		if   (s_address == `REG_BBCOL) s_readdata <= {8'h0, bb_col};
	end
	
	read_d <= s_read;
end

//Fetch next word from message buffer after read from READ_MSG
assign msg_buf_rd = s_chipselect & s_read & ~read_d & ~msg_buf_empty & (s_address == `READ_MSG);
						


endmodule

