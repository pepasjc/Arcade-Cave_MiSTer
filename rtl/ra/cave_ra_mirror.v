/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * RetroAchievements RAM mirror for the Cave MiSTer core.
 *
 * Adapted from jtframe_ra_mirror.v of the RetroAchievements fork of jotego's
 * jtcores / JTFRAME (modules/jtframe/target/mister/hdl/jtframe_ra_mirror.v,
 * https://github.com/jotego/jtcores, GPL-3.0-or-later). Change for Cave:
 * there is no shadow RAM. The core's 64 kB 68000 work RAM (Main.sv mainRam)
 * was turned into a true dual-port M10K whose second port is read-only and
 * 64 bits wide in this module's clock domain, so the copier reads the real
 * RAM directly (rd_addr/rd_data). A 64 kB shadow would cost 64 more M10Ks,
 * and the Cave core already uses every M10K on the device.
 *
 * Copies a 64 kB work RAM window to DDR3 every VBlank so the ARM side
 * (odelot's RetroAchievements fork of Main_MiSTer) can evaluate achievements.
 *
 * DDR layout at byte address 0x3D000000 (the "RACH" Full Mirror header used
 * by the RA fork, see Main_MiSTer ra_ramread.h):
 *   0x00  magic "RACH" (0x52414348 LE), region count 0, flags (bit0 busy),
 *         core version
 *   0x08  frame counter (u32)
 *   0x10  zero (keeps stale mailbox/protocol bytes of other cores clear)
 *   0x100 64 kB of RAM. Each 16-bit word is stored as-is in a little-endian
 *         DDR lane, so byte k is the CPU byte at (k ^ 1) - the layout
 *         FinalBurn Neo exposes for 68000 RAM, which RA arcade sets expect.
 *
 * Copy order: header with busy=1, data, frame counter, header with busy=0.
 * The ARM only accepts a snapshot when busy is clear and the frame counter
 * did not move during its copy.
 *
 * The DDR port is shared with the Cave core through cave_ra_ddr_arb (below):
 * ddr_busy is held high whenever the arbiter has not granted the port, so the
 * copier simply waits between bursts.
 */

module cave_ra_mirror #(parameter
    AW        = 16,            // shadow size as a byte address width: 16 = 64 kB
    DDR_BASE  = 29'h07A0_0000, // 0x3D000000 / 8
    VERSION   = 16'h0100
)(
    input               rst,
    input               clk,        // DDR client clock
    input               lvbl,       // active-low vblank, synchronous to clk
    input               hold,       // ROM download / save state in progress
    // Work RAM read port (clk domain): qword address, data one cycle later.
    // Qword n holds 68000 words 4n..4n+3, word 4n+j in bits 16j+15:16j, each
    // word as-is (bits 15:8 = even CPU byte)
    output     [AW-4:0] rd_addr,
    input        [63:0] rd_data,
    // DDR client
    output reg          active,     // copy in progress
    input               ddr_busy,
    output reg   [ 7:0] ddr_burstcnt,
    output reg   [28:0] ddr_addr,
    output reg          ddr_we,
    output       [ 7:0] ddr_be,
    output reg   [63:0] ddr_din
);

localparam [31:0] MAGIC  = 32'h5241_4348;
localparam  [7:0] BURST  = 8'd32;     // 256 B: aligned, never crosses 4 kB
localparam        QW     = AW-3;     // qword address width
localparam        QWORDS = 1<<QW;

localparam [2:0] IDLE=0, HDR_BUSY=1, PRE=2, DATA=3, FRAME=4, HDR_DONE=5, ZERO=6;

// ---------------------------------------------------------------------------
// Work RAM read port: the address for the next beat is presented one cycle
// ahead, exactly like the jtframe shadow RAM read port
reg  [ 2:0] st;
reg  [QW-1:0] ptr;
wire        accept = ddr_we && !ddr_busy && st == DATA;
wire [63:0] q = rd_data;
assign rd_addr = accept ? ptr + 1'd1 : ptr;

// ---------------------------------------------------------------------------
// Copier
reg         lvbl_l;
reg  [31:0] frame;
reg  [ 4:0] beat;
reg         zeroed;

assign ddr_be = 8'hff;

always @(*) begin
    ddr_din = q;
    case( st )
        HDR_BUSY: ddr_din = { VERSION, 8'h01, 8'd0, MAGIC };
        HDR_DONE: ddr_din = { VERSION, 8'h00, 8'd0, MAGIC };
        FRAME:    ddr_din = { 32'd0, frame };
        ZERO:     ddr_din = 64'd0;
        default:;
    endcase
end

always @(posedge clk, posedge rst) begin
    if( rst ) begin
        st           <= IDLE;
        active       <= 0;
        ddr_we       <= 0;
        ddr_burstcnt <= 8'd1;
        ddr_addr     <= DDR_BASE;
        ptr          <= 0;
        beat         <= 0;
        frame        <= 0;
        lvbl_l       <= 1;
        zeroed       <= 0;
    end else begin
        lvbl_l <= lvbl;
        case( st )
            IDLE: begin
                ddr_we <= 0;
                active <= 0;
                if( lvbl_l && !lvbl && !hold ) begin
                    active       <= 1;
                    ddr_addr     <= DDR_BASE;
                    ddr_burstcnt <= 8'd1;
                    ddr_we       <= 1;
                    st           <= HDR_BUSY;
                end
            end
            HDR_BUSY: if( !ddr_busy ) begin
                ddr_we <= 0;
                ptr    <= 0;
                st     <= PRE;
            end
            PRE: begin // ptr settles into the read port
                ddr_addr     <= DDR_BASE + 29'h20;
                ddr_burstcnt <= BURST;
                beat         <= 0;
                ddr_we       <= 1;
                st           <= DATA;
            end
            DATA: if( accept ) begin
                ptr  <= ptr + 1'd1;
                beat <= beat + 5'd1;
                if( &beat ) begin
                    ddr_addr <= ddr_addr + { 21'd0, BURST };
                    if( &ptr ) begin
                        ddr_addr     <= DDR_BASE + 29'h1;
                        ddr_burstcnt <= 8'd1;
                        frame        <= frame + 32'd1;
                        st           <= FRAME;
                    end
                end
            end
            FRAME: if( !ddr_busy ) begin
                ddr_addr <= zeroed ? DDR_BASE : DDR_BASE + 29'h2;
                st       <= zeroed ? HDR_DONE : ZERO;
            end
            ZERO: if( !ddr_busy ) begin
                zeroed   <= 1;
                ddr_addr <= DDR_BASE;
                st       <= HDR_DONE;
            end
            HDR_DONE: if( !ddr_busy ) begin
                ddr_we <= 0;
                active <= 0;
                st     <= IDLE;
            end
            default: st <= IDLE;
        endcase
    end
end

endmodule

/* Two-client Avalon-MM arbiter for the single MiSTer DDRAM port.
 *
 * Client 0 is the Cave core (its registered command stage respects
 * waitrequest, does at most one transaction at a time and may read), client 1
 * is the RA mirror (write bursts only). Ownership only changes at a
 * transaction boundary: no write burst half-sent and no read beats still due.
 * When both want the port at a boundary, the client that did not start the
 * previous transaction wins, so during the mirror's VBlank copy the two
 * alternate burst by burst and neither is starved. The client not selected
 * sees waitrequest (busy) and holds its command, as Avalon requires.
 */
module cave_ra_ddr_arb (
    input               clk,
    input               rst,
    input               ddr_busy,
    input               ddr_dout_ready,
    // client 0: Cave core
    input               c0_rd,
    input               c0_we,
    input        [28:0] c0_addr,
    input        [ 7:0] c0_be,
    input        [63:0] c0_din,
    input        [ 7:0] c0_burstcnt,
    output              c0_busy,
    // client 1: RA mirror (writes only)
    input               c1_we,
    input        [28:0] c1_addr,
    input        [ 7:0] c1_be,
    input        [63:0] c1_din,
    input        [ 7:0] c1_burstcnt,
    output              c1_busy,
    // DDR
    output              ddr_rd,
    output              ddr_we,
    output       [28:0] ddr_addr,
    output       [ 7:0] ddr_be,
    output       [63:0] ddr_din,
    output       [ 7:0] ddr_burstcnt
);

reg  [8:0] c0_rd_left;   // read beats still to be returned to the core
reg  [7:0] c0_wr_left;   // beats left in the core's open write burst
reg  [7:0] c1_wr_left;   // beats left in the mirror's open write burst
reg        last_c1;      // the mirror started the previous transaction

wire c0_open = c0_rd_left != 0 || c0_wr_left != 0;
wire c1_open = c1_wr_left != 0;
wire c0_req  = c0_rd | c0_we;

wire sel1 = c1_open ? 1'b1 :
            c0_open ? 1'b0 :
            c1_we && (!c0_req || !last_c1);

wire c0_acc_rd = !sel1 && c0_rd && !ddr_busy;
wire c0_acc_we = !sel1 && c0_we && !ddr_busy;
wire c1_acc    =  sel1 && c1_we && !ddr_busy;

wire [7:0] c0_len = c0_burstcnt == 0 ? 8'd1 : c0_burstcnt;
wire [7:0] c1_len = c1_burstcnt == 0 ? 8'd1 : c1_burstcnt;

assign c0_busy      = ddr_busy | sel1;
assign c1_busy      = ddr_busy | ~sel1;
assign ddr_rd       = sel1 ? 1'b0         : c0_rd;
assign ddr_we       = sel1 ? c1_we        : c0_we;
assign ddr_addr     = sel1 ? c1_addr      : c0_addr;
assign ddr_be       = sel1 ? c1_be        : c0_be;
assign ddr_din      = sel1 ? c1_din       : c0_din;
assign ddr_burstcnt = sel1 ? c1_burstcnt  : c0_burstcnt;

always @(posedge clk) begin
    if( rst ) begin
        c0_rd_left <= 0;
        c0_wr_left <= 0;
        c1_wr_left <= 0;
        last_c1    <= 0;
    end else begin
        c0_rd_left <= c0_rd_left + (c0_acc_rd ? {1'b0, c0_len} : 9'd0)
                                 - (ddr_dout_ready && c0_rd_left != 0 ? 9'd1 : 9'd0);
        if( c0_acc_we )
            c0_wr_left <= c0_wr_left != 0 ? c0_wr_left - 8'd1 : c0_len - 8'd1;
        if( c1_acc )
            c1_wr_left <= c1_wr_left != 0 ? c1_wr_left - 8'd1 : c1_len - 8'd1;
        // who started the last transaction
        if( c1_acc && c1_wr_left == 0 )
            last_c1 <= 1;
        else if( (c0_acc_rd || c0_acc_we) && c0_wr_left == 0 )
            last_c1 <= 0;
    end
end

endmodule
