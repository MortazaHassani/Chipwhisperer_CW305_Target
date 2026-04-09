`timescale 1ns / 1ps
`default_nettype none
//////////////////////////////////////////////////////////////////////////////////
// CW305-100t top-level
// Wraps cw305_usb_reg_fe with IOBUF primitives for the bidirectional USB data bus.
//////////////////////////////////////////////////////////////////////////////////

module cw305_top #(
    parameter pBYTECNT_SIZE = 7,
    parameter pADDR_WIDTH   = 21
)(
    // USB interface
    input  wire                   usb_clk,
    inout  wire [7:0]             usb_data,          // bidirectional - split via IOBUF
    input  wire [pADDR_WIDTH-1:0] usb_addr,
    input  wire                   usb_rdn,
    input  wire                   usb_wrn,
    input  wire                   usb_cen,
    input  wire                   usb_trigger,       // unused input; declared for XDC constraint

    // Clocks
    input  wire                   pll_clk1,
    input  wire                   tio_clkin,         // declared for XDC create_clock constraint

    // 20-pin connector
    output wire                   tio_trigger,
    output wire                   tio_clkout,

    // DIP switches / button
    input  wire                   j16_sel,
    input  wire                   k16_sel,
    input  wire                   l14_sel,
    input  wire                   k15_sel,
    input  wire                   pushbutton,

    // LEDs
    output wire                   led1,
    output wire                   led2,
    output wire                   led3
);

    // -------------------------------------------------------------------------
    // IOBUF: split inout usb_data into usb_din / usb_dout / usb_isout
    // -------------------------------------------------------------------------
    wire [7:0] usb_din;    // data received from host  (IOBUF O)
    wire [7:0] usb_dout;   // data driven to host      (IOBUF I)
    wire       usb_isout;  // 1 = drive the bus        (IOBUF T = ~usb_isout)

    genvar gi;
    generate
        for (gi = 0; gi < 8; gi = gi + 1) begin : gen_iobuf
            IOBUF u_iobuf (
                .IO(usb_data[gi]),
                .I (usb_dout[gi]),
                .O (usb_din[gi]),
                .T (~usb_isout)     // T=0 → drive, T=1 → high-Z
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Clocking
    // -------------------------------------------------------------------------
    wire crypt_clk;
    BUFG u_bufg (.I(pll_clk1), .O(crypt_clk));

    // Drive tio_clkout with ODDR so Vivado routes it through a clock-capable path
    ODDR #(
        .DDR_CLK_EDGE("SAME_EDGE"),
        .INIT(1'b0),
        .SRTYPE("SYNC")
    ) u_clkout (
        .Q (tio_clkout),
        .C (crypt_clk),
        .CE(1'b1),
        .D1(1'b1),
        .D2(1'b0),
        .R (1'b0),
        .S (1'b0)
    );

    // -------------------------------------------------------------------------
    // USB register front-end
    // reg_address width = pADDR_WIDTH - pBYTECNT_SIZE = 21 - 7 = 14 bits
    // reg_datao  : data the host wrote to us   (output from fe, input to decoder)
    // reg_datai  : data we send back to host   (input to fe,    driven by read mux)
    // -------------------------------------------------------------------------
    wire [pADDR_WIDTH-1:pBYTECNT_SIZE] reg_address;   // 14-bit register select
    wire [pBYTECNT_SIZE-1:0]           reg_bytecnt;
    wire [7:0]                          reg_datao;     // write data FROM host
    wire [7:0]                          reg_datai;     // read  data TO   host
    wire                                reg_read;
    wire                                reg_write;
    wire                                reg_addrvalid;

    cw305_usb_reg_fe #(
        .pBYTECNT_SIZE(pBYTECNT_SIZE),
        .pADDR_WIDTH  (pADDR_WIDTH)
    ) u_usb_reg_fe (
        .rst          (1'b0),
        .usb_clk      (usb_clk),
        // split data bus
        .usb_din      (usb_din),
        .usb_dout     (usb_dout),
        .usb_isout    (usb_isout),
        // address and control
        .usb_addr     (usb_addr),
        .usb_rdn      (usb_rdn),
        .usb_wrn      (usb_wrn),
        .usb_alen     (1'b1),      // unused in fe; tie inactive (active-low)
        .usb_cen      (usb_cen),
        // register bus
        .reg_address  (reg_address),
        .reg_bytecnt  (reg_bytecnt),
        .reg_datao    (reg_datao),
        .reg_datai    (reg_datai),
        .reg_read     (reg_read),
        .reg_write    (reg_write),
        .reg_addrvalid(reg_addrvalid)
    );

    // -------------------------------------------------------------------------
    // Register definitions (14-bit addresses to match reg_address width)
    // -------------------------------------------------------------------------
    `include "cw305_user_defines.v"

    // -------------------------------------------------------------------------
    // Registers
    // -------------------------------------------------------------------------
    reg [7:0] reg_operand_a;
    reg [7:0] reg_operand_b;
    reg       reg_go;

    // Write decoder — use reg_datao (data sent by the host)
    always @(posedge usb_clk) begin
        reg_go <= 1'b0;
        if (reg_write && reg_addrvalid) begin
            case (reg_address)
                `REG_OPERAND_A : reg_operand_a <= reg_datao;
                `REG_OPERAND_B : reg_operand_b <= reg_datao;
                `REG_CRYPT_GO  : reg_go        <= reg_datao[0];
                default: ;
            endcase
        end
    end

    // -------------------------------------------------------------------------
    // CDC: trig_stretch (usb_clk, ~1.33 µs) → crypt_clk domain
    //
    // reg_go is only ~21 ns wide (1 usb_clk cycle at 48 MHz).
    // crypt_clk at 10 MHz has a 100 ns period → it almost never catches reg_go.
    // trig_stretch is already held high for 64 usb_clk cycles (~1.33 µs),
    // which is >> 2 crypt_clk periods (200 ns), so a 2-FF synchronizer is safe.
    // Rising-edge detect gives exactly one go pulse per trigger in crypt_clk domain.
    // -------------------------------------------------------------------------
    reg go_sync1, go_sync2, go_prev;
    always @(posedge crypt_clk) begin
        go_sync1 <= (trig_stretch != 6'd0);
        go_sync2 <= go_sync1;
        go_prev  <= go_sync2;
    end
    wire go_pulse_crypt = go_sync2 & ~go_prev;   // single rising-edge pulse in crypt_clk

    // -------------------------------------------------------------------------
    // DUT: adder runs on crypt_clk for measurable switching activity
    // -------------------------------------------------------------------------
    reg [8:0] result_r;
    always @(posedge crypt_clk) begin
        if (go_pulse_crypt)
            result_r <= reg_operand_a + reg_operand_b;
    end

    // -------------------------------------------------------------------------
    // Read mux — drive reg_datai (data returned to the host)
    // -------------------------------------------------------------------------
    reg [7:0] reg_read_data;
    always @(*) begin
        case (reg_address)
            `REG_OPERAND_A : reg_read_data = reg_operand_a;
            `REG_OPERAND_B : reg_read_data = reg_operand_b;
            `REG_RESULT_LO : reg_read_data = result_r[7:0];
            `REG_RESULT_HI : reg_read_data = {7'b0, result_r[8]};
            `REG_CRYPT_GO  : reg_read_data = {7'b0, reg_go};
            default        : reg_read_data = 8'hAD;
        endcase
    end
    assign reg_datai = reg_read_data;

    // -------------------------------------------------------------------------
    // Trigger pulse stretcher
    // reg_go is only 1 usb_clk cycle wide (~21 ns at 48 MHz).
    // Stretch to 64 cycles (~1.3 µs) so the CW-Lite reliably detects it.
    // -------------------------------------------------------------------------
    reg [5:0] trig_stretch;
    always @(posedge usb_clk) begin
        if (reg_go)
            trig_stretch <= 6'd63;
        else if (trig_stretch != 6'd0)
            trig_stretch <= trig_stretch - 6'd1;
    end

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    assign tio_trigger = (trig_stretch != 6'd0) | reg_go;

    assign led1 = result_r[8];
    assign led2 = reg_go;
    assign led3 = 1'b0;

    // Suppress unused-input warnings from synthesis
    (* keep = "true" *) wire unused = usb_trigger | tio_clkin
                                    | j16_sel | k16_sel | l14_sel | k15_sel
                                    | pushbutton | (|reg_bytecnt) | reg_read;

endmodule

`default_nettype wire
