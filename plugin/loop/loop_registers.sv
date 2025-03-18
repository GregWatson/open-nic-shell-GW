// Loop registers
// Should be BAR2 at address box250_addr_offset + base address 0x1000
// ie 0x101000

// DDR4_NUM_RD_REGS is number of 32 bit registers that can be read by host 
// i.e. includes writeable registers as well as read-only registers.

// Reg at addr 0 is reset on write and ID on read.
// Writing a 1 to the register will reset the ddr4 control logic.
// Reading this reg will always return the ID value (DDR4_ID_RST_VAL)

// This is instantiated in loop_lite_slave.sv

`timescale 1ns/1ps
module loop_registers #(
  parameter int REG_ADDR_W = 12,
  parameter int REG_PREFIX = 0,
  parameter int DDR4_NUM_RD_REGS = 4,     // Number of readable registers
  parameter int DDR4_RD_REG_BASE = 12'h4, // Address of first readable register
  parameter int DDR4_RD_REG_INC  = 12'h4  // Increment in address between registers
) (
    // --- BOX 250MHz clock domain ---
    input wire                  reg_en,
    input wire                  reg_we,
    input wire [REG_ADDR_W-1:0] reg_addr,
    input wire           [31:0] reg_din,
    output reg           [31:0] reg_dout,
    input wire                  reg_clk,
    input wire                  reg_rst,   // hard block reset

    // --- DDR4 clock domain (1/4 DDR4 i/f) ---

    output reg                  ddr4_reg_we,
    output reg [REG_ADDR_W-1:0] ddr4_reg_addr,
    output reg           [31:0] ddr4_reg_wdata,
    output reg                  ddr4_reg_rst,  // soft user reset

    output reg                  ddr4_0_rst,

    input wire [DDR4_NUM_RD_REGS*32-1:0] ddr4_reg_rdata,
    input                       ddr4_ui_clk,
    input                       ddr4_ui_rst   // hard ddr4 reset
);

    // Local signals, some of which cross the clock domains.

    (* ASYNC_REG = "TRUE" *)reg                           sync_reg_rdata_vld;
    (* ASYNC_REG = "TRUE" *)reg                           sync_reg_rdata_vld_d1;
    (* ASYNC_REG = "TRUE" *)reg [DDR4_NUM_RD_REGS*32-1:0] sync_ddr4_reg_rdata;

    reg                  l_reg_we;
    reg                  l_reg_we_d1;
    reg                  l_reg_we_long;
    reg [REG_ADDR_W-1:0] l_reg_addr;
    reg           [31:0] l_reg_wdata;
    reg                  l_reg_rst;
    reg [15:0]           l_reg_rst_d1;
    reg                  l_reg_rst_long;


    reg [DDR4_NUM_RD_REGS*32-1:0] l_ddr4_reg_rdata;
    reg                           l_ddr4_reg_rdata_vld;


    // START --- BOX 250MHz clock domain ---
    localparam DDR4_ID_RST = 12'h000;
    localparam DDR4_ID_RST_VAL = 32'h2dd4_0222;

    wire is_DDR4_rst = reg_addr == DDR4_ID_RST; // reset on write, ID on read
    wire sample_rd_data = sync_reg_rdata_vld & ~sync_reg_rdata_vld_d1;

    reg [DDR4_NUM_RD_REGS-1:0] id_rd_reg;
    (* ASYNC_REG = "TRUE" *) reg [DDR4_NUM_RD_REGS*32-1:0] ddr4_reg_rdata_f;

    always_ff @(posedge reg_clk) begin
        if (reg_rst) begin
            l_reg_we   <= '0;
            l_reg_we_d1 <= '0;
            l_reg_we_long <= '0;
            l_reg_addr <= '0;
            l_reg_wdata <= '0;
            l_reg_rst   <= 1'b1;
            l_reg_rst_d1 <= 16'd1;
            l_reg_rst_long <= 1'b1;
            sync_reg_rdata_vld <= '0;
            sync_reg_rdata_vld_d1 <= '0;
            sync_ddr4_reg_rdata <= '0;
            ddr4_reg_rdata_f <= '0;
            ddr4_0_rst <= 1'b1;
        end else begin
            l_reg_we   <= reg_we;
            l_reg_we_d1 <= l_reg_we;
            l_reg_we_long <= l_reg_we | l_reg_we_d1;
            l_reg_addr <= reg_en ? reg_addr : l_reg_addr;
            l_reg_wdata <= reg_we ? reg_din : l_reg_wdata;
            l_reg_rst   <= (reg_en & is_DDR4_rst & reg_we & ~ddr4_reg_we) ? reg_din[0] : 1'b0;
            l_reg_rst_d1 <= l_reg_rst ? 16'd1 : {l_reg_rst_d1[14:0],1'b0};
            l_reg_rst_long <= |l_reg_rst_d1;
            sync_reg_rdata_vld  <= l_ddr4_reg_rdata_vld;
            sync_reg_rdata_vld_d1  <= sync_reg_rdata_vld;
            // Only load sync_ddr4_reg_rdata when l_ddr4_reg_rdata has been stable for a couple of clocks.
            sync_ddr4_reg_rdata <= sample_rd_data ? l_ddr4_reg_rdata : sync_ddr4_reg_rdata;
            ddr4_reg_rdata_f    <=  sync_ddr4_reg_rdata;
            ddr4_0_rst <= l_reg_rst_long;
        end
    end

    always_comb begin
        reg_dout[31:16] = REG_PREFIX;
        reg_dout[15:0]  = {4'h0, reg_addr};

        if (is_DDR4_rst) reg_dout = DDR4_ID_RST_VAL;
        else
            for (integer i=0; i < DDR4_NUM_RD_REGS;i++) begin
                id_rd_reg[i] = reg_addr == (DDR4_RD_REG_BASE + i*DDR4_RD_REG_INC);
                if (id_rd_reg[i]) reg_dout = ddr4_reg_rdata_f[32*i +: 32];
            end
    end
    // END --- BOX 250MHz clock domain ---

    // START --- DDR4 clock domain ---
(* ASYNC_REG = "TRUE" *)    reg                  sync_reg_we;
(* ASYNC_REG = "TRUE" *)    reg [REG_ADDR_W-1:0] sync_reg_addr;
(* ASYNC_REG = "TRUE" *)    reg           [31:0] sync_reg_wdata;
(* ASYNC_REG = "TRUE" *)    reg                  sync_reg_rst_long;

    reg                 l_ddr4_we_d1;
    reg                 l_ddr4_wr_strobe;
    reg [15:0] ddr4_rst_vec;
    reg [19:0] l_ddr4_rdata_strobe;

    always_ff @(posedge ddr4_ui_clk) begin
        if (ddr4_ui_rst ) begin
            sync_reg_we       <= '0;
            sync_reg_addr     <= '0;
            sync_reg_wdata    <= '0;
            sync_reg_rst_long <= '0;
            l_ddr4_we_d1      <= '0;
            l_ddr4_wr_strobe  <= '0;
            ddr4_reg_we       <= '0;
            ddr4_reg_addr     <= '0;
            ddr4_reg_wdata    <= '0;
            l_ddr4_reg_rdata     <= '0;
            l_ddr4_rdata_strobe  <= 20'd1;
            l_ddr4_reg_rdata_vld <= 1'b0;
        end else begin
            sync_reg_we       <= l_reg_we_long;
            sync_reg_addr     <= l_reg_addr;
            sync_reg_wdata    <= l_reg_wdata;
            sync_reg_rst_long <= l_reg_rst_long;
            l_ddr4_we_d1      <= sync_reg_we;
            l_ddr4_wr_strobe  <= sync_reg_we & ~l_ddr4_we_d1;  // rising edge detect

            ddr4_reg_we       <= l_ddr4_wr_strobe;
            ddr4_reg_addr     <= l_ddr4_wr_strobe ? sync_reg_addr : sync_reg_wdata;
            ddr4_reg_wdata    <= l_ddr4_wr_strobe ? sync_reg_wdata : ddr4_reg_wdata;

            l_ddr4_reg_rdata     <= l_ddr4_rdata_strobe[0] ? ddr4_reg_rdata : l_ddr4_reg_rdata;
            l_ddr4_rdata_strobe  <= {l_ddr4_rdata_strobe[18:0], l_ddr4_rdata_strobe[19]};  // rotate the 1 bit
            l_ddr4_reg_rdata_vld <= |l_ddr4_rdata_strobe[11:9];
        end

        ddr4_reg_rst  <= reg_rst ? 1'b1 : sync_reg_rst_long;

    end


    // END   --- DDR4 clock domain ---

endmodule: loop_registers
