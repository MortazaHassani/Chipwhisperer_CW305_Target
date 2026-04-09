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
    // reg_user_led holds a 2-bit value written by the host.
    // Bit 0 drives led2 (physical LED5 on board).
    // Bit 1 drives led3 (physical LED6 on board).
    // Example: write 3 (0b11) → both LEDs ON; write 0 → both OFF.
    reg [1:0] reg_user_led;

    // Write decoder — use reg_datao (data sent by the host)
    always @(posedge usb_clk) begin
        reg_go <= 1'b0;                          // reg_go auto-clears every cycle
        if (reg_write && reg_addrvalid) begin
            case (reg_address)
                `REG_OPERAND_A : reg_operand_a <= reg_datao;
                `REG_OPERAND_B : reg_operand_b <= reg_datao;
                `REG_CRYPT_GO  : reg_go        <= reg_datao[0];
                // REG_USER_LED: lower 2 bits select which LEDs are ON.
                // Value 0=both OFF, 1=LED5 only, 2=LED6 only, 3=both ON.
                `REG_USER_LED  : reg_user_led  <= reg_datao[1:0];
                default: ;
            endcase
        end
    end

    // =========================================================================
    // CDC + trigger — fully synchronised design
    //
    // PROBLEM (previous design):
    //   tio_trigger came from trig_stretch in usb_clk domain.
    //   The adder fired from go_pulse_crypt in crypt_clk domain.
    //   Because the two clocks are asynchronous, the time between the scope
    //   seeing the trigger and the adder actually switching power changed
    //   randomly every trace → traces misaligned, power analysis corrupted.
    //
    // SOLUTION — three steps:
    //
    //   Step 1 (usb_clk): Stretch reg_go to a stable level for 64 usb_clk
    //     cycles (~1.33 µs).  reg_go itself is only ~21 ns (1 usb_clk cycle
    //     at 48 MHz); crypt_clk at 10 MHz has a 100 ns period, so a direct
    //     2-FF sync would miss the pulse ~79 % of the time.
    //     go_stretch_active is a clean registered single bit — safer to cross
    //     into a different clock domain than a raw multi-bit comparison.
    //
    //   Step 2 (crossing): 2-FF synchroniser with (* ASYNC_REG = "TRUE" *).
    //     ASYNC_REG tells Vivado to:
    //       • place both FFs in the same slice (minimises routing delay
    //         between them, maximising metastability recovery time), and
    //       • apply set_false_path / set_max_delay constraints automatically
    //         so timing analysis does not flag a false violation.
    //     go_stretch_active is stable for 1.33 µs >> 2 crypt_clk periods
    //     (200 ns), so the synchroniser is guaranteed to catch the level.
    //
    //   Step 3 (crypt_clk): Rising-edge detect → go_pulse_crypt fires once
    //     per trigger.  BOTH the adder result AND trig_stretch are driven by
    //     this same pulse on the same clock edge.
    //     tio_trigger comes from crypt_clk trig_stretch → the scope trigger
    //     and the power event are now aligned to within one crypt_clk cycle,
    //     every single trace.
    // =========================================================================

    // --- Step 1: stretch reg_go to a stable level in usb_clk domain --------
    reg [5:0] go_stretch_ctr;      // countdown counter (usb_clk)
    reg       go_stretch_active;   // registered single bit — clean sync input
    always @(posedge usb_clk) begin
        if (reg_go)
            go_stretch_ctr <= 6'd63;           // load 64-cycle hold on every go
        else if (go_stretch_ctr != 6'd0)
            go_stretch_ctr <= go_stretch_ctr - 6'd1;
        // Register the level: ensures the synchroniser sees a glitch-free bit
        go_stretch_active <= (go_stretch_ctr != 6'd0) | reg_go;
    end

    // --- Step 2: 2-FF synchroniser into crypt_clk domain -------------------
    // ASYNC_REG packs both FFs into the same slice and suppresses false timing
    // errors — do not remove this attribute.
    (* ASYNC_REG = "TRUE" *) reg go_sync1, go_sync2;
    reg go_prev;
    always @(posedge crypt_clk) begin
        go_sync1 <= go_stretch_active;   // first synchroniser stage
        go_sync2 <= go_sync1;            // second synchroniser stage (metastability resolved)
        go_prev  <= go_sync2;            // delayed copy for edge detection
    end

    // --- Step 3: rising-edge detect — one pulse per trigger -----------------
    wire go_pulse_crypt = go_sync2 & ~go_prev;

    // -------------------------------------------------------------------------
    // DUT + trigger stretcher — both driven by go_pulse_crypt in crypt_clk
    //
    // Running trig_stretch here (not in usb_clk) means tio_trigger fires on
    // the exact crypt_clk edge where result_r is being computed.
    // The scope trigger and the power switching event are now locked together.
    // -------------------------------------------------------------------------
    reg [8:0] result_r;
    reg [5:0] trig_stretch;        // trigger width counter (crypt_clk domain)
    always @(posedge crypt_clk) begin
        // Adder fires on the rising edge of the synchronised go signal
        if (go_pulse_crypt)
            result_r <= reg_operand_a + reg_operand_b;

        // Trigger stretcher: hold tio_trigger high for 64 crypt_clk cycles
        // (~6.4 µs at 10 MHz) so the CW-Lite reliably detects the edge.
        if (go_pulse_crypt)
            trig_stretch <= 6'd63;
        else if (trig_stretch != 6'd0)
            trig_stretch <= trig_stretch - 6'd1;
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
            // Read back the stored LED value so Python can verify the write.
            `REG_USER_LED  : reg_read_data = {6'b0, reg_user_led};
            default        : reg_read_data = 8'hAD;
        endcase
    end
    assign reg_datai = reg_read_data;

    // -------------------------------------------------------------------------
    // Outputs
    // -------------------------------------------------------------------------
    // tio_trigger is purely crypt_clk domain — aligned with the power event.
    assign tio_trigger = (trig_stretch != 6'd0);

    assign led1 = result_r[8];          // carry-out from the adder
    assign led2 = reg_user_led[0];      // LED5: bit 0 of REG_USER_LED
    assign led3 = reg_user_led[1];      // LED6: bit 1 of REG_USER_LED

    // Suppress unused-input warnings from synthesis
    (* keep = "true" *) wire unused = usb_trigger | tio_clkin
                                    | j16_sel | k16_sel | l14_sel | k15_sel
                                    | pushbutton | (|reg_bytecnt) | reg_read;

endmodule

`default_nettype wire
