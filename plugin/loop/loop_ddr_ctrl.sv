// Loop registers
// Should be BAR2 at address box250_addr_offset + base address 0x1000
// ie 0x101000

// DDR4_NUM_RD_REGS is number of 32 bit registers that can be read by host 
// i.e. includes writeable registers as well as read-only registers.

// Reg at addr 0 is reset on write and ID on read.
// Writing a 1 to the register will reset the ddr4 control logic.
// Reading this reg will always return the ID value (DDR4_ID_RST_VAL)

`timescale 1ns/1ps
module loop_ddr_ctrl #(
  parameter int REG_ADDR_W = 12,
  parameter int DDR4_NUM_RD_REGS = 4     // Number of readable registers
) (
    //loop_registers side
    input wire                  ddr4_reg_we,
    input wire [REG_ADDR_W-1:0] ddr4_reg_addr,
    input wire           [31:0] ddr4_reg_wdata,
    output reg [DDR4_NUM_RD_REGS*32-1:0] ddr4_reg_rdata,
    input wire                  ddr4_reg_rst,// user soft reset

    // DDR4 controller
    output reg c0_ddr4_app_en,
    output reg c0_ddr4_app_hi_pri,
    output reg c0_ddr4_app_wdf_end,
    output reg c0_ddr4_app_wdf_wren,
    output reg c0_ddr4_app_correct_en_i,
    input  wire c0_ddr4_app_rd_data_end,
    input  wire c0_ddr4_app_rd_data_valid,
    input  wire c0_ddr4_app_rdy,
    input  wire c0_ddr4_app_wdf_rdy,
    output reg [30 : 0]  c0_ddr4_app_addr,
    output reg [2 : 0]   c0_ddr4_app_cmd,
    output reg [511 : 0] c0_ddr4_app_wdf_data,
    output reg [63 : 0]  c0_ddr4_app_wdf_mask,
    input  wire [511 : 0] c0_ddr4_app_rd_data,
    input  wire c0_ddr4_ui_clk,
    input  wire c0_ddr4_ui_clk_sync_rst,
    input  wire c0_init_calib_complete
);


localparam DDR4_ADDR_CTRL     = 'h4;                     // 0x04
localparam DDR4_ADDR_STATUS   = DDR4_ADDR_CTRL + 'h4;    // 0x08
localparam DDR4_ADDR_COUNT    = DDR4_ADDR_STATUS + 'h4;  // 0x0c
localparam DDR4_ADDR_WDATA    = DDR4_ADDR_COUNT + 'h4;   // 0x10
localparam DDR4_ADDR_UI_CNT   = DDR4_ADDR_WDATA + 'h4;   // 0x14
localparam DDR4_ADDR_RD_DATA  = DDR4_ADDR_UI_CNT + 'h4;  // 0x18 RO
localparam DDR4_ADDR_RD_EVNTS = DDR4_ADDR_RD_DATA + 'h4; // 0x1c  RW
localparam DDR4_ADDR_ERR_ADDR = DDR4_ADDR_RD_EVNTS + 'h4;// 0x20  RW


wire is_ctrl_reg   = ddr4_reg_addr == DDR4_ADDR_CTRL;
wire is_status_reg = ddr4_reg_addr == DDR4_ADDR_STATUS;
wire is_count_reg  = ddr4_reg_addr == DDR4_ADDR_COUNT;
wire is_wdata_reg  = ddr4_reg_addr == DDR4_ADDR_WDATA;
wire is_ui_cnt_reg = ddr4_reg_addr == DDR4_ADDR_UI_CNT;
wire is_rd_events  = ddr4_reg_addr == DDR4_ADDR_RD_EVNTS;
wire is_err_addr   = ddr4_reg_addr == DDR4_ADDR_ERR_ADDR;

reg rst;
reg ctrl_bit_start;
reg ctrl_bit_checkrd;
reg ctrl_bit_type;    // 0 = WRITE 1 = READ
reg ctrl_bit_xor;     // XOR data with addr.
reg [3:0] ctrl_bit_rdwd_sel; // which of 16 32b words is read from register
reg status_bit_done;
reg status_bit_running;
reg status_bit_rd_pass;
reg status_bit_rd_fail;
reg status_bit_calib_complete;

reg [30:0] num_cmds; // number of cmds to execute
reg [30:0] counter;   // counts up the number of commands executed
reg [31:0] ui_clk_cnt;   // free running up counter. Check c0_ddr4_ui_clk freq.
reg [31:0] ui_clk_cnt_d1; 
reg [31:0] wdata_base; // used to construct write data

reg [511 : 0] rd_data_f;
reg [511 : 0] rd_data;
reg           rd_data_valid_f;
reg           rd_data_valid;
reg [31:0] selected_rd32;
reg [31:0] selected_rd32_r;
reg [31:0] rd_counter;   // addr of next read data
reg [31:0] rd_counter_nxt;
reg [31:0] num_rd_events;  // count of valid read data events
reg [31:0] rd_counter_d1;
reg [31:0] first_err_addr;
reg err_addr_vld;

localparam NUM_STATE_BITS = 3;
reg [NUM_STATE_BITS-1:0] state;
reg [NUM_STATE_BITS-1:0] state_nxt;

localparam S_IDLE     = 3'b001;
localparam S_WAIT_RDY = 3'b010;
localparam S_DELAY    = 3'b100;

reg drive_ddr4_signals;

reg incr_addr;
wire [30:0] counter_nxt;
assign counter_nxt = incr_addr ? counter + 32'h1 : counter;

wire  cmds_is_non_zero;
assign cmds_is_non_zero =  | num_cmds;

wire counter_done;
assign  counter_done = counter == num_cmds;
wire DDR4_xact_accepted;
assign DDR4_xact_accepted = c0_ddr4_app_rdy & c0_ddr4_app_wdf_rdy;

reg [511:0] actual_wdata;
always_comb for (integer i=0; i<16; i++)
    actual_wdata[32*i +: 32 ] = ctrl_bit_xor ? {counter[23:0], 4'd0, i[3:0]} ^ wdata_base : wdata_base;

assign c0_ddr4_app_en = drive_ddr4_signals;
assign c0_ddr4_app_hi_pri = '0;
assign c0_ddr4_app_wdf_end = drive_ddr4_signals & ~ctrl_bit_type;
assign c0_ddr4_app_wdf_wren = drive_ddr4_signals & ~ctrl_bit_type;  // 0  = write
assign c0_ddr4_app_correct_en_i = '0;
assign c0_ddr4_app_addr = {counter[27:0],3'd0};
assign c0_ddr4_app_cmd = {2'b0, ctrl_bit_type};  // 0 = write  1 = read
assign c0_ddr4_app_wdf_data = actual_wdata;
assign c0_ddr4_app_wdf_mask = '0;

assign rd_counter_nxt = rd_data_valid_f ? rd_counter + 32'h1 : rd_counter;

reg rd_counter_done;
assign rd_counter_done = rd_data_valid & (rd_counter == num_cmds);

reg [511:0] exp_rdata;
reg [511:0] exp_rdata_r;
always_comb for (integer i=0; i<16; i++)
    exp_rdata[32*i +: 32 ] = ctrl_bit_xor ? {rd_counter[23:0], 4'd0, i[3:0]} ^ wdata_base : wdata_base;

wire check_rd_data_now = rd_data_valid & ctrl_bit_checkrd;
wire rd_error;
assign rd_error = check_rd_data_now & (exp_rdata_r !== rd_data);


always_comb begin
    state_nxt = state;
    incr_addr = 1'b0;
    drive_ddr4_signals = 1'b0;
    unique case(state)
        S_IDLE: begin if (ctrl_bit_start & cmds_is_non_zero) 
            state_nxt = S_WAIT_RDY;
        end
        S_WAIT_RDY: begin // drive wr request and wait for accept
            drive_ddr4_signals = 1'b1;
            if (DDR4_xact_accepted) begin
                incr_addr = 1'b1;
                state_nxt =  S_DELAY;
            end
        end
        S_DELAY: begin  // waste a cycle. Should be able to remove this once up and running.
            if (counter_done) state_nxt = S_IDLE;
            else state_nxt = S_WAIT_RDY;
        end
    endcase

    for (integer i=0; i < 16; i++) begin
        if (ctrl_bit_rdwd_sel == i[3:0]) selected_rd32 = rd_data[i*32 +: 32];
    end
end

always_ff @(posedge c0_ddr4_ui_clk) begin
    rst <= ddr4_reg_rst ;
end

always_ff @(posedge c0_ddr4_ui_clk) begin
    if (rst) begin
        ctrl_bit_start <= 1'b0;
        ctrl_bit_type <= 1'b0;
        ctrl_bit_xor <= 1'b0;
        ctrl_bit_checkrd <= 1'b0;
        ctrl_bit_rdwd_sel <= 4'h0;
        counter <= '0;
        num_cmds <= '0;
        wdata_base <= '0;
        ui_clk_cnt <= '0;
        ui_clk_cnt_d1 <= '0;

        status_bit_done <= '0;
        status_bit_running <= '0;
        status_bit_rd_pass <= 1'b0;
        status_bit_rd_fail <= 1'b0;
        status_bit_calib_complete <= '0;
        state <= S_IDLE;
        rd_data_valid_f <= 1'b0;
        rd_data_valid   <= 1'b0;
        rd_counter <= '0;
        num_rd_events <= '0;
        err_addr_vld <= 1'b0;
        first_err_addr <= '0;

    end    
    else begin 
        ctrl_bit_start    <= (ddr4_reg_we & is_ctrl_reg) & ddr4_reg_wdata[0] & (state == S_IDLE);
        ctrl_bit_type     <= (ddr4_reg_we & is_ctrl_reg) ? ddr4_reg_wdata[1] : ctrl_bit_type;
        ctrl_bit_xor      <= (ddr4_reg_we & is_ctrl_reg) ? ddr4_reg_wdata[2] : ctrl_bit_xor;
        ctrl_bit_checkrd  <= (ddr4_reg_we & is_ctrl_reg) ? ddr4_reg_wdata[3] : ctrl_bit_checkrd;
        wdata_base        <= (ddr4_reg_we & is_wdata_reg) ? ddr4_reg_wdata : wdata_base;

        ctrl_bit_rdwd_sel <= (ddr4_reg_we & is_ctrl_reg) ? ddr4_reg_wdata[7:4] : ctrl_bit_rdwd_sel;
        num_cmds          <= (ddr4_reg_we & is_count_reg) ? ddr4_reg_wdata : num_cmds;
        ui_clk_cnt        <= (ddr4_reg_we & is_ui_cnt_reg) ? ddr4_reg_wdata : ui_clk_cnt + 32'h1;
        ui_clk_cnt_d1     <= ui_clk_cnt;
        counter           <= state == S_IDLE ? '0 : counter_nxt;

        status_bit_done    <= state == S_IDLE;
        status_bit_running <= state == S_WAIT_RDY;
        status_bit_rd_pass <= ctrl_bit_start ? (ctrl_bit_type & ctrl_bit_checkrd) : status_bit_rd_pass & ~rd_error;
        status_bit_rd_fail <= ctrl_bit_start ? 1'b0 : (rd_error | status_bit_rd_fail);

        status_bit_calib_complete <= c0_init_calib_complete;
        state <= state_nxt;

        rd_data_f       <= c0_ddr4_app_rd_data;
        rd_data_valid_f <= c0_ddr4_app_rd_data_valid;
        rd_data         <= rd_data_valid_f ? rd_data_f : rd_data;
        rd_data_valid   <= rd_data_valid_f;
        selected_rd32_r <= selected_rd32;
        exp_rdata_r     <= rd_data_valid_f ? exp_rdata : exp_rdata;

        rd_counter <= (ctrl_bit_start & ctrl_bit_type)  ? '0 : rd_counter_nxt;
        rd_counter_d1 <= rd_counter;
        num_rd_events <= (ddr4_reg_we & is_rd_events) ? ddr4_reg_wdata : 
                                        rd_data_valid ? num_rd_events + 32'h1 
                                                      : num_rd_events;
        err_addr_vld <= ctrl_bit_start & ctrl_bit_type & ctrl_bit_checkrd ? 1'b0 :
                        check_rd_data_now ? err_addr_vld | rd_error : err_addr_vld;
        first_err_addr <= (ddr4_reg_we & is_err_addr) ? ddr4_reg_wdata :
                          check_rd_data_now & ~err_addr_vld & rd_error ? rd_counter_d1 : 
                          first_err_addr;
    end

end


always_comb begin
    ddr4_reg_rdata = '0;
    ddr4_reg_rdata[0]   = ctrl_bit_start;
    ddr4_reg_rdata[1]   = ctrl_bit_type;
    ddr4_reg_rdata[2]   = ctrl_bit_xor;
    ddr4_reg_rdata[3]   = ctrl_bit_checkrd;
    ddr4_reg_rdata[7:4] = ctrl_bit_rdwd_sel;

    ddr4_reg_rdata[32] = status_bit_done;           // status reg [0]
    ddr4_reg_rdata[33] = status_bit_running;        // status reg [1]
    ddr4_reg_rdata[34] = status_bit_rd_pass;        // status reg [2]
    ddr4_reg_rdata[35] = status_bit_rd_fail;        // status reg [3]
    ddr4_reg_rdata[48 +: NUM_STATE_BITS] = state;   // status reg [18:16]
    ddr4_reg_rdata[62] = DDR4_xact_accepted;        // status reg [30]
    ddr4_reg_rdata[63] = status_bit_calib_complete; // status reg [31]

    ddr4_reg_rdata[2*32 +: 32] = {1'b0,num_cmds};
    ddr4_reg_rdata[3*32 +: 32] = wdata_base;
    ddr4_reg_rdata[4*32 +: 32] = ui_clk_cnt_d1;
    ddr4_reg_rdata[5*32 +: 32] = selected_rd32_r;
    ddr4_reg_rdata[6*32 +: 32] = num_rd_events;
    ddr4_reg_rdata[7*32 +: 32] = first_err_addr;
end

endmodule: loop_ddr_ctrl
