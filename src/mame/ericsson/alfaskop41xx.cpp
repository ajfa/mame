// license:BSD-3-Clause
// copyright-holders: Joakim Larsson Edström
/***************************************************************************

Alfaskop 41 series

This driver is a part of a revivel project for Alfaskop 41 series where
no known working system exists today because of its distributed nature.
All parts network boots over SS3 (SDLC) from a Floppy Disk unit and nothing
works unless there is a floppy in that unit. These floppies are rare and
many parts have been discarded because they are useless stand alone.

The goal is to emulate missing parts so a full system can be demonstrated again.

Links and credits
-----------------
Project home page: https://github.com/MattisLind/alfaskop_emu
Dalby Datormusem - http://www.datormuseum.se/peripherals/terminals/alfaskop
Bitsavers - http://bitsavers.org/pdf/ericsson/alfaskop/
Dansk Datahistorisk Forening - http://datamuseum.dk/

****************************************************************************/

#include "emu.h"

#include "cpu/m6800/m6800.h"
#include "imagedev/floppy.h"
#include "machine/6821pia.h"
#include "machine/6840ptm.h"
#include "machine/6850acia.h"
#include "machine/clock.h"
#include "machine/mc6852.h"
#include "../skeleton/alfaskop_s41_kb.h"
#include "machine/input_merger.h"
#include "machine/mc6844.h"
#include "machine/mc6854.h"
#include "machine/output_latch.h"
#include "machine/pla.h"
#include "machine/wd_fdc.h"
#include "video/mc6845.h"

#include "formats/img_dsk.h"

#include "screen.h"

//#include "bus/rs232/rs232.h"
//#include "machine/clock.h"

#define LOG_IO    (1U << 1)
#define LOG_NVRAM (1U << 2)
#define LOG_MIC   (1U << 3)
#define LOG_DIA   (1U << 4)
#define LOG_DMA   (1U << 5)
#define LOG_IRQ   (1U << 6)
#define LOG_ADLC  (1U << 7)
#define LOG_FDC   (1U << 8)

#define VERBOSE (LOG_MIC|LOG_ADLC|LOG_IRQ|LOG_DMA|LOG_IO|LOG_FDC)
//#define LOG_OUTPUT_STREAM std::cout

#include "logmacro.h"

#define LOGIO(...)    LOGMASKED(LOG_IO,    __VA_ARGS__)
#define LOGNVRAM(...) LOGMASKED(LOG_NVRAM, __VA_ARGS__)
#define LOGMIC(...)   LOGMASKED(LOG_MIC,   __VA_ARGS__)
#define LOGDIA(...)   LOGMASKED(LOG_DIA,   __VA_ARGS__)
#define LOGDMA(...)   LOGMASKED(LOG_DMA,   __VA_ARGS__)
#define LOGIRQ(...)   LOGMASKED(LOG_IRQ,   __VA_ARGS__)
#define LOGADLC(...)  LOGMASKED(LOG_ADLC,  __VA_ARGS__)
#define LOGFDC(...)   LOGMASKED(LOG_FDC,   __VA_ARGS__)


namespace {

#define PLA1_TAG "ic50"
#define PLA1_INUSE 0 // 0=disabled until a PLA converter between DATAIO and MAXLOADER (mame format) exists

// returned by pending_level() when no interrupt is valid at vector fetch time
static constexpr uint8_t NO_IRQ = 0xff;

class alfaskop4110_state : public driver_device
{
public:
	alfaskop4110_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_kbd_acia(*this, "kbd_acia")
		, m_mic_pia(*this, "mic_pia")
		, m_dia_pia(*this, "dia_pia")
		, m_crtc(*this, "crtc")
		, m_screen(*this, "screen")
		, m_vram(*this, "vram")
		, m_pla(*this, PLA1_TAG)
		, m_chargen(*this, "chargen")
		, m_tia_adlc(*this, "tia_adlc")
		, m_tia_dma(*this, "tia_dma")
		, m_irq(0)
		, m_imsk(0)
	{ }

	void alfaskop4110(machine_config &config);
private:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void mem_map(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	required_device<acia6850_device> m_kbd_acia;
	required_device<pia6821_device> m_mic_pia;
	required_device<pia6821_device> m_dia_pia;
	required_device<mc6845_device> m_crtc;
	required_device<screen_device> m_screen;
	required_shared_ptr<uint8_t> m_vram;
	required_device<pls100_device> m_pla;

	/* Video controller */
	required_region_ptr<uint8_t> m_chargen;
	MC6845_UPDATE_ROW(crtc_update_row);

	/* TIA */
	required_device<mc6854_device> m_tia_adlc;
	required_device<mc6844_device> m_tia_dma;

	/* Interrupt handling */
	template <unsigned N> void irq_w(int state);
	uint8_t pending_level() const;
	void update_irq();
	void set_imsk(uint8_t level);
	uint8_t m_irq;
	uint8_t m_imsk;

	/* Debug stuff */
	/* Timer callbacks */
	TIMER_CALLBACK_MEMBER(poll_start);
	TIMER_CALLBACK_MEMBER(poll_bit);

	// DEBUG stuff, will be removed when hooked up towards remote peer
	/* zero extended SDLC poll message frame to feed into receiver as a test
	   0 1 1 1 1 1 1 0   ; opening flag 0x7e
	   0 0 0 0 0 0 0 0   ; 0x00
	   1 1 1 1 1 0 1 1 1 ; 0xff <- a zero needs to be inserted, done by test code
	   0 0 0 0 0 0 1 1   ; 0xc0
	   0 0 0 0 0 1 0 1   ; 0xa0
	   1 0 1 1 0 0 0 1   ; CRC 0x8d
	   1 0 1 0 1 0 1 0   ; CRC 0x55
	   0 1 1 1 1 1 1 0   ; closing flag 0x7e
	*/
	uint8_t txBuf[10] = {0x7e, 0x00, 0xff, 0xc0, 0xa0, 0x8d, 0x55, 0x7e};
	emu_timer *m_poll_start_timer = nullptr;
	emu_timer *m_poll_bit_timer = nullptr;
	int index = 0;
	int pos   = 0;
	int ones  = 0;
	bool flank = false;
};

class alfaskop4120_state : public driver_device
{
public:
	alfaskop4120_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_ram(*this, "ram")
		, m_mic_pia(*this, "mic_pia")
		, m_fdapia(*this, "dia_pia")
		, m_dma(*this, "dma")
		, m_adlc(*this, "adlc")
		, m_fdc(*this, "fdc%u", 0U)
		, m_floppy(*this, "fdc%u:0", 0U)
		, m_cpcpu(*this, "cpcpu")
		, m_cpram(*this, "cpram")
		, m_cp_mic_pia(*this, "cp_mic_pia")
		, m_cp_cs_pia(*this, "cp_cs_pia")
		, m_cp_dma(*this, "cp_dma")
		, m_cp_adlc(*this, "cp_adlc%u", 0U)
		, m_cp_ptm(*this, "cp_ptm")
		, m_cp_ssda(*this, "cp_ssda")
		, m_ducpu(*this, "ducpu")
		, m_du_vram(*this, "du_vram")
		, m_du_kbd_acia(*this, "du_kbd_acia")
		, m_du_kbd(*this, "du_kbd")
		, m_du_mic_pia(*this, "du_mic_pia")
		, m_du_dia_pia(*this, "du_dia_pia")
		, m_du_crtc(*this, "du_crtc")
		, m_du_screen(*this, "du_screen")
		, m_du_chargen(*this, "duchargen")
		, m_du_adlc(*this, "du_adlc")
		, m_du_dma(*this, "du_dma")
		, m_du_aca_pia(*this, "du_aca_pia")
		, m_du_aca_ptm(*this, "du_aca_ptm")
		, m_du_aca_acia(*this, "du_aca_acia")
	{ }

	void alfaskop4120(machine_config &config);
private:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void mem_map(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	required_shared_ptr<uint8_t> m_ram;
	required_device<pia6821_device> m_mic_pia;
	required_device<pia6821_device> m_fdapia;
	required_device<mc6844_device> m_dma;
	required_device<mc6854_device> m_adlc;
	required_device_array<fd1771_device, 2> m_fdc;
	required_device_array<floppy_connector, 2> m_floppy;

	/* Scaffolding: a synthetic SS3 configuration master that polls this unit,
	   so the disk side can be exercised before the CP 4101 is emulated.  The
	   frame layout is the one in the Communication chapter, Appendix 2:
	   Todev, Dsa, Frdev, Status/Msgtyp, contents. */
	TIMER_CALLBACK_MEMBER(ss3_poll);
	void ss3_frame_from_unit(uint8_t *data, int length);

	emu_timer *m_ss3_timer = nullptr;
	unsigned m_ss3_data_frames = 0;

	/* Periodic TIMER interrupt: a discrete timer feeds the MIC PIA's CA1 input
	   (FD chapter, MIC PIA section); the PIA's IRQA output is interrupt I1.
	   The interrupt is re-enabled by reading data register A, which is the
	   PIA's own CA1 semantics. */
	TIMER_CALLBACK_MEMBER(sys_tick);
	emu_timer *m_tick_timer = nullptr;
	bool m_tick = false;
	TIMER_CALLBACK_MEMBER(fd_rx_pace);
	emu_timer *m_fd_rx_pace_timer = nullptr;
	// One of these per receiving station: the drain of a received frame is
	// metered out at the line rate (26.67 us a byte), which is the half of
	// the overlapping model that makes a frame occupy the wire for as long
	// as the wire really needs.  Pacing only the flexible disk left the
	// display and the processor draining a 3328-byte block in 6.5 ms
	// instead of 88.9 (measured with run/block_time.py).
	TIMER_CALLBACK_MEMBER(du_rx_pace);
	emu_timer *m_du_rx_pace_timer = nullptr;
	TIMER_CALLBACK_MEMBER(cp_rx_pace);
	emu_timer *m_cp_rx_pace_timer[4] = { nullptr, nullptr, nullptr, nullptr };
	// The two-wire runs at 300 kbit/s - "nominally 37.5 kbytes/s" per the
	// Communication chapter - so a byte takes 26.67 us.  mc6854_device models
	// frames, not bit timing, and asks for the next transmit byte the instant
	// it has swallowed the last one; the DMAC then empties a whole frame in a
	// few microseconds.  That burst is what starves the disk: on this board
	// the ADLC transmit is DMA channel 0 and the FD1771 is channel 2, and the
	// 6844's fixed priority puts channel 0 first, so while the burst lasts the
	// floppy controller gets nothing and loses a byte.
	// ★ The two-wire runs at 300 kbit/s - "nominally 37.5 kbytes/s" per the
	// Communication chapter - so a byte takes 26.67 us, and mc6854_device
	// models frames, not bit timing: it asks for the next transmit byte the
	// instant it has swallowed the last one, so the DMAC empties a whole
	// frame in a few microseconds.  Pacing that at the real line rate, and
	// even just releasing the bus between bytes as mode 1 requires, was
	// tried and REFUTED as the cause of the disk timeout: the four late
	// requests keep exactly the same timestamps (qH3 against qE7).  Pacing
	// at the honest rate additionally breaks the IPL, because our frame
	// delivery does not overlap transmission with reception and the two add
	// up instead of running together (qH0-qH2).
	// ★ And a fact worth keeping: measured with run/overlap.py over qE7, the
	// disk and the two-wire NEVER overlap - every block the unit serves has
	// exactly zero disk requests in the 35 ms around it, because the FD reads
	// the whole block into RAM first and sends it afterwards.  m_disk_busy is
	// therefore a stale flag during a transmission: it goes up on the command
	// byte and only comes down on INTRQ.
	bool m_disk_busy = false;
	TIMER_CALLBACK_MEMBER(du_acia_clk);
	emu_timer *m_du_acia_clk_timer = nullptr;
	bool m_du_acia_clk_on = false;
	bool m_du_acia_clk_level = false;
	bool m_du_kbd_line_kbd = true;  // keyboard's driver level on the shared pair
	bool m_du_kbd_line_acia = true; // DTC ACIA's driver level on the shared pair
	TIMER_CALLBACK_MEMBER(du_kbd_echo);
	emu_timer *m_du_kbd_echo_timer = nullptr;
	int m_du_kbd_echo_level = 1;
	unsigned m_ss3_polls = 0;
	bool m_ss3_rx_tail = false;
	uint8_t m_ss3_last[4] = { 0, 0, 0, 0 };

	/* Scaffolding: a synthetic BSC host behind the SCA's SSDA.  The CP's
	   host driver (BCC50300) configures the 6852 as an EBCDIC tributary
	   (sync code 32) and listens; this end plays the 3270 BSC control
	   station: EOT, then a general poll of the cluster, and it captures
	   whatever the cluster answers.  Byte-level, like the modem sees it. */
	TIMER_CALLBACK_MEMBER(bsc_host_tick);
	emu_timer *m_bsc_timer = nullptr;
	std::vector<uint16_t> m_bsc_txq;   // host -> CU, one byte per tick; 0xffff = idle tick
	std::vector<uint8_t> m_bsc_rx;     // CU -> host capture
	uint8_t m_cp_ssda_c1 = 0x03;       // last C1 written by the CP
	uint8_t m_cp_ssda_scr = 0xff;      // its sync code register (parked at FF after tx)
	bool m_bsc_hunting = true;         // its receiver needs a sync char first
	int m_bsc_listen = 0;
	int m_bsc_settle = 1200;           // first poll ~2 s after rx enable
	int m_bsc_cu = 0;                  // CU address scan position
	int m_bsc_phase = 0;               // 0 poll, 1 selected, 2 text sent
	int m_bsc_polls_ok = 0;            // EOT answers seen
	// Write the screen ONCE and then only poll.  Re-selecting kept re-sending
	// the Erase/Write every few seconds, and each one erases the buffer and
	// re-inserts the cursor, so anything the operator had typed was wiped
	// before it could be read back (measured: the cursor advances with every
	// key - 161, 162, 163 - and then jumps back to 161 when the host writes
	// again).  A real application writes its screen and waits.
	bool m_bsc_written = false;
	uint32_t m_ssda_rd_count = 0;      // data-register reads by the CP
	uint32_t m_bsc_rd_mark = 0;        // read count when SYN-idle began
	uint32_t m_bsc_syn_div = 0;        // SYN-idle rate divider
	uint32_t m_bsc_ls_mark = 0;        // lockstep: reads at last injection
	int m_bsc_ls_wait = 0;             // lockstep: ticks waited for a read
	uint32_t m_bsc_rd_seen = 0;        // fill budget: reads high-water mark
	int m_bsc_fill_budget = 2;         // fill bytes allowed while unread
	uint8_t m_bsc_ack = 0x61;          // alternating ack (the CP xors 0x11)
	int m_bsc_sent = 0;                // bytes injected since envelope start
	std::vector<uint16_t> m_bsc_env;   // copy of the current envelope
	bool m_bsc_capped = false;         // bare envelopes: prime 3, then wait
	uint32_t m_bsc_tok_seen = 0;       // token: reads at previous token tick
	bool m_bsc_tok_armed = false;      // token: seen at least one tick

	/* Interrupt handling, see the Flexible Disk Unit chapter, Fig. 6:
	   I0 SOFT, I1 TIMER, I4 ADLC, I5 FDC1, I6 FDC0, I7 DMAC.  The prioritized
	   vector lives in the alternative table in RWM at 01E8-01F7 while the IPL
	   signal is active, which is what the IPL PROM sets up. */
	template <unsigned N> void irq_w(int state);
	uint8_t pending_level() const;
	void update_irq();
	void set_imsk(uint8_t level);
	uint8_t m_irq = 0;
	uint8_t m_imsk = 0;

	/* FDA board */
	uint8_t fda_pa_r();
	uint8_t fda_pb_r();
	void fda_pa_w(uint8_t data);
	void fda_pb_w(uint8_t data);
	uint8_t m_fda_pa = 0;
	uint8_t m_fda_pb = 0;
	bool m_strap_ipl_source = false; // strapped: the unit IPLs from its own diskette

	/* CP 4101, the communication processor and real master of the SS3 bus,
	   second CPU of this machine.  CPR chapter: DMAC at F700, four ADLCs at
	   F720/28/30/38 (channels 0-3 = interrupts I7-I4), crosspoint PIA at F740,
	   MIC PIA at F7C4 (CA2 = I0), PTM at F7C8 (I1), NVRAM at F600, switches at
	   F7FC, IPL PROM at F800.  The IPL loads the operating software from the FD
	   over the two-wire into the MRW expansion memory at 8000 and jumps there. */
	required_device<cpu_device> m_cpcpu;
	required_shared_ptr<uint8_t> m_cpram;
	required_device<pia6821_device> m_cp_mic_pia;
	required_device<pia6821_device> m_cp_cs_pia;
	required_device<mc6844_device> m_cp_dma;
	required_device_array<mc6854_device, 4> m_cp_adlc;
	required_device<ptm6840_device> m_cp_ptm;
	required_device<mc6852_device> m_cp_ssda; // SCA board, SSDA toward the host

	void cp_mem_map(address_map &map) ATTR_COLD;
	template <unsigned N> void cp_irq_w(int state);
	uint8_t cp_pending_level() const;
	void cp_update_irq();
	void cp_set_imsk(uint8_t level);
	uint8_t m_cp_irq = 0;
	uint8_t m_cp_imsk = 0;
	bool m_cp_rx_tail[4] = { false, false, false, false };
	bool m_cp_rdsr[4] = { false, false, false, false }; // each ADLC's rx and tx share one DMA channel
	bool m_cp_tdsr[4] = { false, false, false, false };

	/* DU 4110, the display unit, third CPU of this machine.  Its own IPL only
	   knows how to answer polls; the operating software arrives over the
	   two-wire from the CP (which fetches it from the FD).  Same interrupt
	   controller style as the CP: vector cells in the PROM point at RAM
	   trampolines.  The unit address comes from the NVRAM (zeros = port 0,
	   type DU = address 00, which is a display in the diskette's default
	   customising data). */
	required_device<cpu_device> m_ducpu;
	required_shared_ptr<uint8_t> m_du_vram;
	required_device<acia6850_device> m_du_kbd_acia;
	required_device<alfaskop_s41_keyboard_device> m_du_kbd;
	required_device<pia6821_device> m_du_mic_pia;
	required_device<pia6821_device> m_du_dia_pia;
	required_device<mc6845_device> m_du_crtc;
	required_device<screen_device> m_du_screen;
	required_region_ptr<uint8_t> m_du_chargen;
	required_device<mc6854_device> m_du_adlc;
	required_device<mc6844_device> m_du_dma;
	// ACA, "Asynchronous Communication Adapter" (E34193 2000): the display's
	// printer interface.  Three chips behind one 32x8 decode ROM fed by
	// AB7..AB3 and Per I/O - CSP -> the 6821, CST -> the 6840, CSA -> the
	// 6850 - with J4/J5 strapping AB7/AB6 so a second board can sit at
	// $F7A0/$F7A8/$F7B0 (the IPL PROM initialises both positions).
	required_device<pia6821_device> m_du_aca_pia;
	required_device<ptm6840_device> m_du_aca_ptm;
	required_device<acia6850_device> m_du_aca_acia;
	// Is the ACA fitted?  It is a real option board, and this diskette's
	// LOGICAL ADDRESSES form declares a printer on this display's port, so
	// the faithful answer is yes - but see the map for why it is not the
	// default yet.
	bool m_du_aca_fitted = true;

	void du_mem_map(address_map &map) ATTR_COLD;
	MC6845_UPDATE_ROW(du_crtc_update_row);
	template <unsigned N> void du_irq_w(int state);
	uint8_t du_pending_level() const;
	void du_update_irq();
	void du_set_imsk(uint8_t level);
	uint8_t m_du_irq = 0;
	uint8_t m_du_imsk = 0;
	bool m_du_rx_tail = false;

	void ss3_frame_from_du(uint8_t *data, int length);
	void ss3_frame_from_cp_ch(int ch, uint8_t *data, int length);
	template <unsigned CH> void ss3_frame_from_cp_t(uint8_t *data, int length) { ss3_frame_from_cp_ch(CH, data, length); }
	TIMER_CALLBACK_MEMBER(ss3_to_du);
	TIMER_CALLBACK_MEMBER(du_carrier_tail);
	emu_timer *m_ss3_to_du_timer = nullptr;
	emu_timer *m_du_dcd_timer = nullptr;

	/* The SS3 two-wire between the CP's channel 0 and the FD, at frame level.
	   When one side's closing flag has gone out its modem drops CTS (that is
	   how the software detects transmit-complete), and the frame reaches the
	   other side a moment later, restoring CTS on the receiver of the *other*
	   direction just as a live line would. */
	TIMER_CALLBACK_MEMBER(ss3_to_cp);
	TIMER_CALLBACK_MEMBER(ss3_to_fd);
	TIMER_CALLBACK_MEMBER(cp_carrier_tail);
	emu_timer *m_ss3_to_cp_timer = nullptr;
	emu_timer *m_ss3_to_fd_timer = nullptr;
	emu_timer *m_cp_dcd_timer[4] = { nullptr, nullptr, nullptr, nullptr };

	/* One short delivery queue per destination.  A single mailbox loses frames
	   when two conversations interleave (the CP talks to the FD on channel 0
	   - polls - and channel 1 - file sessions - at the same time). */
	struct ss3_queue
	{
		/* The FD's file service hands out a whole library member as a single
		   frame (CPEMUL is 6485 bytes), so a slot must hold far more than the
		   4096-byte wire maximum of the poll protocol. */
		static constexpr int SLOT = 32768;
		uint8_t buf[4][SLOT];
		int len[4] = { 0, 0, 0, 0 };
		int head = 0, count = 0;
		unsigned dropped = 0;
		char const *name = "?";     // which destination, so a drop names itself
		void clear() { head = 0; count = 0; }
		void push(const uint8_t *d, int l)
		{
			// Overrun: the oldest frame is lost.  This used to be silent, which
			// is a blind spot - a dropped frame looks like a protocol failure
			// several layers up.  Count them and say so, because with the
			// drain paced at the line rate a long frame occupies a receiver
			// for tens of milliseconds and everything offered meanwhile piles
			// up here.
			if (count == 4)
			{
				head = (head + 1) & 3;
				count--;
				if ((++dropped % 50) == 1)
					osd_printf_error("ss3_queue -> %s: queue full, frame dropped (%u so far)\n",
							 name, dropped);
			}
			int const t = (head + count) & 3;
			if (l > SLOT)
				osd_printf_error("ss3_queue: frame of %d bytes truncated to %d\n", l, SLOT);
			len[t] = std::min(l, SLOT);
			memcpy(buf[t], d, len[t]);
			count++;
		}
		uint8_t *front() { return buf[head]; }
		int front_len() const { return len[head]; }
		void pop() { if (count) { head = (head + 1) & 3; count--; } }
	};
	ss3_queue m_q_to_fd, m_q_to_cp, m_q_to_du;

	/* How long a frame occupies the two-wire.
	 *
	 * The Communication chapter gives 300 kbit/s, "nominally 37.5 kbytes/s",
	 * so a byte is 26.67 us, and on the wire a frame carries its contents
	 * plus Todev, Dsa, Frdev, Msgtyp and two CRC bytes between the flags.
	 *
	 * Delivery used to be scheduled a flat millisecond after the sender
	 * finished, which is wrong in BOTH directions and by a lot - measured
	 * over a whole run with run/wire_timing.py:
	 *
	 *     4-8 byte frames (20645 of them, the polls)   1072 us vs   226 us
	 *     1-4 kbyte frames (the IPL blocks)            1016 us vs 75499 us
	 *     over 4 kbyte (a library member)              1016 us vs 164987 us
	 *
	 * so the polls crawled and the bulk transfers went by seventy to a
	 * hundred and sixty times faster than the line allows.
	 *
	 * ★Charging the whole frame's time as a DELAY before handover was tried
	 * and REFUTED: the communication processor's IPL never completes (one
	 * data block served in 400 s against dozens before, and the display sits
	 * at LOAD forever - qU0/qU1).  The reason is that its PROM expects the
	 * block to START arriving right after its request, not to appear 89 ms
	 * later in one piece.
	 *
	 * So the latency here is ONE BYTE - the head of the frame reaches the
	 * receiver promptly - and the rest of the frame's time comes from pacing
	 * the DRAIN at the line rate, which is where it belongs: mc6854_device
	 * already keeps the frame and refills the receive FIFO byte by byte as
	 * the receiver pops it, so transmission and reception genuinely overlap
	 * instead of being added end to end. */
	static attotime ss3_wire_time(int length)
	{
		(void)length;
		return attotime::from_nsec(26667);
	}
	// EXPERIMENT (off) - which two-wire pair the display unit is plugged into.
	// The shipped diskette configures port 8, which the firmware maps to SS3
	// station $01.  Delivering only that station's frames to the display does
	// NOT work: its IPL PROM latches its own address at $F952 from the first
	// frame carrying Dsa = 00, and station $01 is always polled with Dsa = 01,
	// so the display never leaves LOAD (run qB5).  Left here, disarmed, until
	// it is known how a display is meant to come up on a configured port.
	bool    m_ss3_du_port_filter = false;   // experiment: set true to arm the probe
	bool    m_ss3_du_latched = false;
	uint8_t m_ss3_du_port = 0x01;
	void cp_cancel_tails() { for (auto &t : m_cp_dcd_timer) t->adjust(attotime::never); }
};

class alfaskop4101_state : public driver_device
{
public:
	alfaskop4101_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_mic_pia(*this, "mic_pia")
	{ }

	void alfaskop4101(machine_config &config);
private:
	void mem_map(address_map &map) ATTR_COLD;

	required_device<cpu_device> m_maincpu;
	required_device<pia6821_device> m_mic_pia;
};

void alfaskop4110_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0x7fff).ram();
	map(0x7800, 0x7fff).ram().share(m_vram); // TODO: Video RAM base address is configurable via NVRAM - this is the default
	map(0x8000, 0xefff).ram();

	// NVRAM
	map(0xf600, 0xf6ff).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGNVRAM("nvram_r %04x: %02x\n", offset, 0); return (uint8_t) 0; }),
				 NAME( [this](offs_t offset, uint8_t data) {    LOGNVRAM("nvram_w %04x: %02x\n", offset, data); }));
	// TIA board
	map(0xf700, 0xf71f).mirror(0x00).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGDMA("TIA DMA_r %04x: %02x\n", offset, 0); return m_tia_dma->read(offset); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGDMA("TIA DMA_w %04x: %02x\n", offset, data); m_tia_dma->write(offset, data); }));
	map(0xf720, 0xf723).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGADLC("TIA ADLC_r %04x: %02x\n", offset, 0); return m_tia_adlc->read(offset); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGADLC("TIA ADLC_w %04x: %02x\n", offset, data); m_tia_adlc->write(offset, data); }));

	// Main PCB
	map(0xf7d9, 0xf7d9).mirror(0x06).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGIO("CRTC reg r %04x: %02x\n", offset, 0); return m_crtc->register_r(); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGIO("CRTC reg w %04x: %02x\n", offset, data); m_crtc->register_w(data);}));
	map(0xf7d8, 0xf7d8).mirror(0x06).lw8(NAME([this](offs_t offset, uint8_t data) { LOGIO("CRTC adr w %04x: %02x\n", offset, data); m_crtc->address_w(data); }));
	map(0xf7d0, 0xf7d3).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGDIA("DIA pia_r %04x: %02x\n", offset, 0); return m_dia_pia->read(offset & 3); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGDIA("DIA pia_w %04x: %02x\n", offset, data); m_dia_pia->write(offset & 3, data); }));
	map(0xf7c4, 0xf7c7).mirror(0x00).lrw8(NAME([this](offs_t offset) -> uint8_t    { uint8_t tmp = m_mic_pia->read(offset & 3); LOGMIC("\nMIC pia_r %04x: %02x\n", offset, tmp); return tmp; }),
						  NAME([this](offs_t offset, uint8_t data) { LOGMIC("\nMIC pia_w %04x: %02x\n", offset, data); m_mic_pia->write(offset & 3, data); }));
	map(0xf7c0, 0xf7c1).mirror(0x02).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGIO("KBD acia_r %04x: %02x\n", offset, 0); return m_kbd_acia->read(offset & 1); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGIO("KBD acia_w %04x: %02x\n", offset, data); m_kbd_acia->write(offset & 1, data); }));

	map(0xf7fc, 0xf7fc).mirror(0x00).lr8(NAME([this](offs_t offset) -> uint8_t { LOGIO("Address Switch 0-7\n"); return 0; }));

	// Interrupt controller, as described in the Technical Description,
	// "Microcomputer" chapter, Fig. 10 (E90002331E / EE356-810D):
	//
	//   FFE8,9 .. FFF6,7   vectors for I0..I7, I7 being the highest priority
	//   FFF8,9             "default" vector, used when no valid interrupt is
	//                      pending at the time of the vector fetch
	//   FFFA..FFFF         SWI, NMI and RESET vectors
	//
	// "MPU reading FFF8-FFF9 will transfer the contents of two cells within
	// FFE8-FFF9, depending on highest priority valid interrupt, to the MPU."
	// "A (read or preferably) write operation into one of the cells FFE8-FFF7
	// will result in a setting of a corresponding mask value.  As an example,
	// FFEE or FFEF from the MPU will disable interrupts of priority < 3."
	//
	// The discrete logic and the IC50 PLA implement this; the behaviour is
	// modelled directly from the documentation so no fuse map is needed.
	map(0xf800, 0xffe7).rom().region("roms", 0);

	map(0xffe8, 0xfff7).lrw8(NAME([this](offs_t offset) -> uint8_t
					{
						if (!machine().side_effects_disabled()) set_imsk(offset >> 1);
						return memregion("roms")->base()[0x7e8 + offset];
					}),
					NAME([this](offs_t offset, uint8_t data)
					{
						set_imsk(offset >> 1);
					}));

	map(0xfff8, 0xfff9).lr8(NAME([this](offs_t offset) -> uint8_t
					{
						uint8_t const level = pending_level();
						offs_t const src = (level == NO_IRQ) ? 0x7f8 : (0x7e8 + (level << 1));
						if (!machine().side_effects_disabled())
							LOGIRQ("Vector fetch: irq %02x mask %d ==> %s vector at %04X\n",
								   m_irq, m_imsk, level == NO_IRQ ? "default" : "prioritized",
								   0xf800 + src);
						return memregion("roms")->base()[src + offset];
					}));

	map(0xfffa, 0xffff).rom().region("roms", 0x7fa);
}

void alfaskop4120_state::mem_map(address_map &map)
{
	// Technical Description, "Flexible Disk Unit" chapter (EE369-810), pages 5-7.
	//
	// CS-PROM1 decodes the high address byte:
	//
	//   00-3F  RWM1 (16 kbytes)      E8-EF  IPL ROM, first  2 kbytes (A12 = 0)
	//   40-7F  RWM2 (16 kbytes)      F8-FF  IPL ROM, second 2 kbytes (A12 = 1)
	//   F7     I/O area
	//
	// CS-PROM2/3 decode the I/O area itself.  Note there is no battery backed
	// RAM at F600 in this unit, unlike the display unit and the CP.
	map.unmap_value_high();
	map(0x0000, 0x7fff).ram().share(m_ram);
	// DEBUG: trace every write to the kernel's current/next task pointers at
	// $0000-$0003 with the program counter, to catch who corrupts them
	map(0x0000, 0x0003).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_ram[offset]; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					 logerror("ZP%04X <- %02X  (PC=%04X t=%f)\n", offset, data, m_maincpu->pcbase(), machine().time().as_double());
					 m_ram[offset] = data;
				 }));
	// DEBUG: same for the communication task's status block pointer at $0485
	map(0x0485, 0x0486).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_ram[0x485 + offset]; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					 logerror("PTR%04X <- %02X  (PC=%04X t=%f)\n", 0x485 + offset, data, m_maincpu->pcbase(), machine().time().as_double());
					 m_ram[0x485 + offset] = data;
				 }));
	// DEBUG: protocol state variables of the loaded software: $17 response
	// code, $18 session state, $19 length, $38 status byte, $A3 message type
	map(0x0017, 0x0019).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_ram[0x17 + offset]; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					 logerror("VAR%04X <- %02X  (PC=%04X t=%f)\n", 0x17 + offset, data, m_maincpu->pcbase(), machine().time().as_double());
					 m_ram[0x17 + offset] = data;
				 }));
	map(0x0038, 0x0038).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_ram[0x38]; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					 logerror("VAR0038 <- %02X  (PC=%04X t=%f)\n", data, m_maincpu->pcbase(), machine().time().as_double());
					 m_ram[0x38] = data;
				 }));
	map(0x00a3, 0x00a3).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_ram[0xa3]; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					 logerror("VAR00A3 <- %02X  (PC=%04X t=%f)\n", data, m_maincpu->pcbase(), machine().time().as_double());
					 m_ram[0xa3] = data;
				 }));
	// EXPERIMENT: the operating software on the 4016 system diskette keeps its
	// task control blocks above 32K (pointers like $85F0 show up in ZP $00/$02),
	// which only makes sense on a unit extended with an MRW memory board.
	map(0x8000, 0xe7ff).ram();
	map(0xe800, 0xefff).rom().region("roms", 0x0000);

	map(0xf700, 0xf71f).lrw8(NAME([this](offs_t offset) -> uint8_t    { uint8_t tmp = m_dma->read(offset); LOGDMA("DMAC r %02x: %02x\n", offset, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGDMA("DMAC w %02x: %02x  (canal %d)\n", offset, data, offset < 0x10 ? offset / 4 : -1); m_dma->write(offset, data); }));
	map(0xf720, 0xf723).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { uint8_t tmp = m_adlc->read(offset & 3); LOGADLC("ADLC r %d: %02x\n", offset & 3, tmp); return tmp; }),
						  NAME([this](offs_t offset, uint8_t data) { LOGADLC("ADLC w %d: %02x\n", offset & 3, data); m_adlc->write(offset & 3, data); }));
	// The FD1771 has an active low data bus and the FDA board has inverting
	// buffers on it, so the two inversions cancel: undo the one the device
	// model applies internally.
	map(0xf730, 0xf733).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { uint8_t tmp = m_fdc[0]->read(offset & 3) ^ 0xff; LOGFDC("%f FDC0 r %d: %02x\n", machine().time().as_double(), offset & 3, tmp); return tmp; }),
						  NAME([this](offs_t offset, uint8_t data) { LOGFDC("%f FDC0 w %d: %02x\n", machine().time().as_double(), offset & 3, data); if ((offset & 3) == 0) m_disk_busy = (data & 0xf0) != 0xd0; m_fdc[0]->write(offset & 3, data ^ 0xff); }));
	map(0xf738, 0xf73b).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { uint8_t tmp = m_fdc[1]->read(offset & 3) ^ 0xff; LOGFDC("%f FDC1 r %d: %02x\n", machine().time().as_double(), offset & 3, tmp); return tmp; }),
						  NAME([this](offs_t offset, uint8_t data) { LOGFDC("%f FDC1 w %d: %02x\n", machine().time().as_double(), offset & 3, data); if ((offset & 3) == 0) m_disk_busy = (data & 0xf0) != 0xd0; m_fdc[1]->write(offset & 3, data ^ 0xff); }));
	map(0xf740, 0xf743).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGIO("FDA pia_r %04x: %02x\n", offset, 0); return m_fdapia->read(offset & 3); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGIO("FDA pia_w %04x: %02x\n", offset, data); m_fdapia->write(offset & 3, data); }));
	map(0xf7c4, 0xf7c7).mirror(0x00).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGMIC("MIC pia_r %04x: %02x\n", offset, 0); return m_mic_pia->read(offset & 3); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGMIC("MIC pia_w %04x: %02x\n", offset, data); m_mic_pia->write(offset & 3, data); }));
	map(0xf7f8, 0xf7ff).lr8(NAME([this](offs_t offset) -> uint8_t { LOGIO("FDP TEST_r %04x\n", offset); return 0; }));

	// Interrupt mask and vector modification, same hardware as in the display
	// unit but with the vectors taken from the alternative table in RWM at
	// 01E8-01F7 (01F8 for the default one) while the IPL signal is active.
	map(0xf800, 0xffe7).rom().region("roms", 0x0800);

	// Reading these cells still returns what the PROM holds - the IPL checksums
	// itself over F802-FFFF and would fail otherwise - the side effect is that
	// the mask register gets set to the level the cell belongs to.
	map(0xffe8, 0xfff7).lrw8(NAME([this](offs_t offset) -> uint8_t
					{
						if (!machine().side_effects_disabled()) set_imsk(offset >> 1);
						return memregion("roms")->base()[0x0fe8 + offset];
					}),
					NAME([this](offs_t offset, uint8_t data) { set_imsk(offset >> 1); }));

	map(0xfff8, 0xfff9).lr8(NAME([this](offs_t offset) -> uint8_t
					{
						// with no valid interrupt the address is not modified and
						// the PROM answers, which is what the checksum test needs
						uint8_t const level = pending_level();
						if (level == NO_IRQ)
							return memregion("roms")->base()[0x0ff8 + offset];
						offs_t const src = 0x01e8 + (level << 1);
						if (!machine().side_effects_disabled())
							LOGIRQ("Vector fetch: irq %02x mask %d ==> I%d, vector en RWM %04X\n",
								   m_irq, m_imsk, level, src);
						return m_ram[src + offset];
					}));

	map(0xfffa, 0xfffb).lr8(NAME([this](offs_t offset) -> uint8_t
					{
						// The SWI vector is also taken from the alternative table
						// (the loaded operating software installs its system-call
						// dispatcher at 01FA/B and every SWI must go through it),
						// but an ordinary data read - the IPL checksumming itself -
						// must still see the PROM.  The IPL and the checksum run
						// from the PROM while every SWI of the loaded software
						// executes from RWM, so the program counter tells the
						// two apart.
						if (m_maincpu->pcbase() < 0x8000 && m_ram[0x01fa] != 0xaa)
						{
							if (!machine().side_effects_disabled())
								LOGIRQ("Vector fetch: SWI ==> vector en RWM 01FA\n");
							return m_ram[0x01fa + offset];
						}
						return memregion("roms")->base()[0x0ffa + offset];
					}));
	map(0xfffc, 0xffff).rom().region("roms", 0x0ffc);
}

void alfaskop4120_state::cp_mem_map(address_map &map)
{
	// CPR chapter, Fig. 3 (CP memory map) and Appendix 1 (I/O addresses)
	map.unmap_value_high();
	map(0x0000, 0x7fff).ram().share(m_cpram); // 32K on the CPB
	map(0x8000, 0xf5ff).ram(); // MRW expansion board: the IPL loads the OS at 8000 and jumps there
	map(0xf600, 0xf6ff).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGNVRAM("CP nvram_r %04x\n", offset); return 0; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGNVRAM("CP nvram_w %04x: %02x\n", offset, data); }));
	map(0xf700, 0xf71f).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_cp_dma->read(offset); LOGDMA("CP DMAC r %02x: %02x\n", offset, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGDMA("CP DMAC w %02x: %02x  (canal %d)\n", offset, data, offset < 0x10 ? offset / 4 : -1); m_cp_dma->write(offset, data); }));
	map(0xf720, 0xf727).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_cp_adlc[0]->read(offset & 3); LOGADLC("CP ADLC0 r %x: %02x\n", offset & 3, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGADLC("CP ADLC0 w %x: %02x\n", offset & 3, data); m_cp_adlc[0]->write(offset & 3, data); }));
	map(0xf728, 0xf72f).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_cp_adlc[1]->read(offset & 3); }),
				 NAME([this](offs_t offset, uint8_t data) { m_cp_adlc[1]->write(offset & 3, data); }));
	map(0xf730, 0xf737).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_cp_adlc[2]->read(offset & 3); }),
				 NAME([this](offs_t offset, uint8_t data) { m_cp_adlc[2]->write(offset & 3, data); }));
	map(0xf738, 0xf73f).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_cp_adlc[3]->read(offset & 3); }),
				 NAME([this](offs_t offset, uint8_t data) { m_cp_adlc[3]->write(offset & 3, data); }));
	map(0xf740, 0xf747).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_cp_cs_pia->read(offset & 3); LOGIO("CP CS pia_r %x: %02x\n", offset & 3, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGIO("CP CS pia_w %x: %02x\n", offset & 3, data); m_cp_cs_pia->write(offset & 3, data); }));
	// SCA, the host computer adapter.  The host driver (BCC50300) expects the
	// board to be present - a real CPR always carries one - and runs off into
	// nothing with a hollow stub.  The SSDA toward the host answers at
	// F770/F771; the rest of the board's registers still read as zero.
	map(0xf750, 0xf76f).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGIO("CP SCA r %04x\n", 0xf750 + offset); return 0x00; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGIO("CP SCA w %04x: %02x\n", 0xf750 + offset, data); }));
	map(0xf770, 0xf771).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_cp_ssda->read(offset & 1); if (offset & 1) m_ssda_rd_count++; LOGIO("%f CP SSDA r %x: %02x\n", machine().time().as_double(), offset & 1, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					LOGIO("%f CP SSDA w %x: %02x\n", machine().time().as_double(), offset & 1, data);
					if ((offset & 1) == 0)
					{
						// clearing sync or resetting the receiver puts it back
						// into hunt: the next thing it can latch onto is a
						// character matching its sync code register
						if (data & 0x09)
							m_bsc_hunting = true;
						m_cp_ssda_c1 = data;   // C1 is always register 0
					}
					else if ((m_cp_ssda_c1 & 0xc0) == 0x80)
						m_cp_ssda_scr = data;  // AC = 10: register 1 is the sync code
					m_cp_ssda->write(offset & 1, data);
				 }));
	map(0xf772, 0xf77f).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGIO("CP SCA r %04x\n", 0xf772 + offset); return 0x00; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGIO("CP SCA w %04x: %02x\n", 0xf772 + offset, data); }));
	map(0xf7c4, 0xf7c7).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_cp_mic_pia->read(offset & 3); LOGMIC("CP MIC pia_r %x: %02x\n", offset & 3, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGMIC("CP MIC pia_w %x: %02x\n", offset & 3, data); m_cp_mic_pia->write(offset & 3, data); }));
	map(0xf7c8, 0xf7cf).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_cp_ptm->read(offset & 7); }),
				 NAME([this](offs_t offset, uint8_t data) { LOGIO("CP PTM w %x: %02x\n", offset & 7, data); m_cp_ptm->write(offset & 7, data); }));
	// unit address switches; bit 7 = 0 selects the normal cold start path
	map(0xf7fc, 0xf7fc).lr8(NAME([this](offs_t offset) -> uint8_t { LOGIO("CP switches read\n"); return 0x00; }));

	// Same interrupt controller as the display unit: the vector cells live in
	// the PROM and point at trampolines in RAM which the loaded OS overwrites
	map(0xf800, 0xffe7).rom().region("cproms", 0);
	map(0xffe8, 0xfff7).lrw8(NAME([this](offs_t offset) -> uint8_t
					{
						if (!machine().side_effects_disabled()) cp_set_imsk(offset >> 1);
						return memregion("cproms")->base()[0x7e8 + offset];
					}),
					NAME([this](offs_t offset, uint8_t data) { cp_set_imsk(offset >> 1); }));
	map(0xfff8, 0xfff9).lr8(NAME([this](offs_t offset) -> uint8_t
					{
						uint8_t const level = cp_pending_level();
						offs_t const src = (level == NO_IRQ) ? 0x7f8 : (0x7e8 + (level << 1));
						if (!machine().side_effects_disabled())
							LOGIRQ("CP vector fetch: irq %02x mask %d ==> %s en %04X\n",
								   m_cp_irq, m_cp_imsk, level == NO_IRQ ? "default" : "prioritized",
								   0xf800 + src);
						return memregion("cproms")->base()[src + offset];
					}));
	map(0xfffa, 0xffff).rom().region("cproms", 0x7fa);
}

uint8_t alfaskop4120_state::cp_pending_level() const
{
	for (int level = 7; level >= 0; level--)
		if (BIT(m_cp_irq, level) && level >= m_cp_imsk)
			return uint8_t(level);
	return NO_IRQ;
}

void alfaskop4120_state::cp_update_irq()
{
	m_cpcpu->set_input_line(M6800_IRQ_LINE, cp_pending_level() != NO_IRQ ? ASSERT_LINE : CLEAR_LINE);
}

void alfaskop4120_state::cp_set_imsk(uint8_t level)
{
	if (m_cp_imsk != (level & 7))
	{
		m_cp_imsk = level & 7;
		LOGIRQ("CP IRQ mask set to %d\n", m_cp_imsk);
		cp_update_irq();
	}
}

template <unsigned N> void alfaskop4120_state::cp_irq_w(int state)
{
	m_cp_irq = (m_cp_irq & ~(1 << N)) | ((state ? 1 : 0) << N);
	LOGIRQ("%f CP IRQ %d: %d ==> %02x\n", machine().time().as_double(), N, state, m_cp_irq);
	cp_update_irq();
}

void alfaskop4120_state::du_mem_map(address_map &map)
{
	// Display Unit chapter: same common microcomputer map as the standalone
	// 4110 machine, with the TIA board at F700/F720
	map.unmap_value_high();
	map(0x0000, 0x7fff).ram();
	map(0x7800, 0x7fff).ram().share(m_du_vram); // video RAM base from NVRAM; zeros give the default
	map(0x8000, 0xefff).ram();
	map(0xf600, 0xf6ff).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGNVRAM("DU nvram_r %04x\n", offset); return 0; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGNVRAM("DU nvram_w %04x: %02x\n", offset, data); }));
	map(0xf700, 0xf71f).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_dma->read(offset); }),
				 NAME([this](offs_t offset, uint8_t data) { LOGDMA("DU DMAC w %02x: %02x\n", offset, data); m_du_dma->write(offset, data); }));
	map(0xf720, 0xf727).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_du_adlc->read(offset & 3); LOGADLC("DU ADLC r %x: %02x\n", offset & 3, tmp); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGADLC("DU ADLC w %x: %02x\n", offset & 3, data); m_du_adlc->write(offset & 3, data); }));
	// ACA, the printer interface - E34193 2000, sheets 1 and 3.  Its decode
	// ROM takes AB7..AB3 with Per I/O as chip enable and produces three
	// selects; this board is strapped (J4/J5) to the first position:
	//
	//   CSP -> 68A21 PIA  $F750-$F753   RS0/RS1 = A0/A1
	//   CST -> 68A40 PTM  $F758-$F75F   RS0-RS2 = A0-A2
	//   CSA -> 68A50 ACIA $F760-$F761   RS = A0
	//
	// The display will not act as a cluster device without it: $414D..$4195
	// of the loaded software checks the memory size and then writes $55 and
	// $AA to the PIA's port B (with DDRB all outputs) and reads them back;
	// only if both come back does it set bit 6 of $0423, and only with
	// $0423 == $C0 does $461E keep the SECOND SS3 address it computes
	// ($023A |= 1 = subunit 1 of its own pair = station $01).  Otherwise
	// $4632 throws it away and writes $FC = none.  Station $01 is exactly
	// where the diskette's LOGICAL ADDRESSES form puts the printer: logical
	// address 8, 3270 device $C8, on the display's own port.
	//
	// NOT FITTED BY DEFAULT, and the reason is measured, not cautionary.
	// With the board answering, the display does exactly what it should: the
	// self-test passes ($0423 = $C0), it keeps station $01 on its own
	// ($023A = 01 at t=49.7, no pokes) and it goes and fetches the printer
	// software from the diskette - member "APRDEF   " of library "SYSLIB  ",
	// which the FD serves, 1287 bytes to $9A07.  Then it enters that module
	// ONE BYTE LATE: $9A07 holds CE 02 A3 ("LDX #$02A3"), the processor
	// starts at $9A08 on the $02 and dies on illegal opcodes, taking the
	// working 3274 emulation with it at t~=50.  Until that last byte is
	// understood the display is left without its printer interface, which is
	// also a perfectly real configuration.
	if (m_du_aca_fitted)
	{
		map(0xf750, 0xf753).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_aca_pia->read(offset & 3); }),
					 NAME([this](offs_t offset, uint8_t data) { LOGIO("DU ACA pia_w %x: %02x\n", offset & 3, data); m_du_aca_pia->write(offset & 3, data); }));
		map(0xf758, 0xf75f).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_aca_ptm->read(offset & 7); }),
					 NAME([this](offs_t offset, uint8_t data) { LOGIO("DU ACA ptm_w %x: %02x\n", offset & 7, data); m_du_aca_ptm->write(offset & 7, data); }));
		map(0xf760, 0xf761).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_aca_acia->read(offset & 1); }),
					 NAME([this](offs_t offset, uint8_t data) { LOGIO("DU ACA acia_w %x: %02x\n", offset & 1, data); m_du_aca_acia->write(offset & 1, data); }));
	}
	map(0xf7c0, 0xf7c1).mirror(0x02).lrw8(NAME([this](offs_t offset) -> uint8_t { uint8_t tmp = m_du_kbd_acia->read(offset & 1); logerror("%f DU KBD acia r %x: %02x (PC=%04X)\n", machine().time().as_double(), offset & 1, tmp, m_ducpu->pcbase()); return tmp; }),
				 NAME([this](offs_t offset, uint8_t data)
				 {
					logerror("%f DU KBD acia w %x: %02x (PC=%04X)\n", machine().time().as_double(), offset & 1, data, m_ducpu->pcbase());
					m_du_kbd_acia->write(offset & 1, data);
					// The real board clocks the ACIA at 76.8 kHz (1200 baud
					// in /64 mode) permanently, but a free-running 6.5 us
					// timer drags the whole machine to a tiny scheduling
					// quantum and the FD's disk DMA starts losing bytes.
					// Start the bit clock only when the DTC actually
					// configures the interface (first non-reset control
					// write), well after the bulk of the disk loading.
					if ((offset & 1) == 0 && (data & 3) != 3 && !m_du_acia_clk_on)
					{
						m_du_acia_clk_on = true;
						// The keyboard's bit engine runs off its own 3.579545
						// MHz crystal: one bit = 754 counts of the 894886.25 Hz
						// E clock (841.5 us, ~1187 baud).  The DTC's ACIA clock
						// must be sibling-locked to that grid: with 76800 Hz
						// (1200 baud) the start-bit phase drifts ~110 us per
						// byte across a back-to-back stream and the keyboard's
						// tick-quantized start detect slips one bit within 2-4
						// bytes (kills the strapping download).  894886.25 * 64
						// / 754 = 75958.5 Hz keeps the phase locked.
						attotime const half = attotime::from_ticks(754, 3'579'545 / 4 * 64 * 2);
						m_du_acia_clk_timer->adjust(half, 0, half);
						logerror("DU KBD acia bit clock started (75.96 kHz, sibling of the keyboard crystal)\n");
					}
				 }));
	map(0xf7c4, 0xf7c7).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_mic_pia->read(offset & 3); }),
				 NAME([this](offs_t offset, uint8_t data) { LOGMIC("DU MIC pia_w %x: %02x\n", offset & 3, data); m_du_mic_pia->write(offset & 3, data); }));
	map(0xf7d0, 0xf7d3).mirror(0x04).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_dia_pia->read(offset & 3); }),
				 NAME([this](offs_t offset, uint8_t data) { LOGDIA("DU DIA pia_w %x: %02x\n", offset & 3, data); m_du_dia_pia->write(offset & 3, data); }));
	map(0xf7d8, 0xf7d8).mirror(0x06).lw8(NAME([this](offs_t offset, uint8_t data) { m_du_crtc->address_w(data); }));
	map(0xf7d9, 0xf7d9).mirror(0x06).lrw8(NAME([this](offs_t offset) -> uint8_t { return m_du_crtc->register_r(); }),
				 NAME([this](offs_t offset, uint8_t data) { m_du_crtc->register_w(data); }));
	map(0xf7fc, 0xf7fc).lr8(NAME([this](offs_t offset) -> uint8_t { LOGIO("DU switches read\n"); return 0x00; }));

	map(0xf800, 0xffe7).rom().region("duroms", 0);
	map(0xffe8, 0xfff7).lrw8(NAME([this](offs_t offset) -> uint8_t
					{
						if (!machine().side_effects_disabled()) du_set_imsk(offset >> 1);
						return memregion("duroms")->base()[0x7e8 + offset];
					}),
					NAME([this](offs_t offset, uint8_t data) { du_set_imsk(offset >> 1); }));
	map(0xfff8, 0xfff9).lr8(NAME([this](offs_t offset) -> uint8_t
					{
						uint8_t const level = du_pending_level();
						offs_t const src = (level == NO_IRQ) ? 0x7f8 : (0x7e8 + (level << 1));
						if (!machine().side_effects_disabled())
							LOGIRQ("DU vector fetch: irq %02x mask %d ==> %s en %04X\n",
								   m_du_irq, m_du_imsk, level == NO_IRQ ? "default" : "prioritized",
								   0xf800 + src);
						return memregion("duroms")->base()[src + offset];
					}));
	map(0xfffa, 0xffff).rom().region("duroms", 0x7fa);
}

uint8_t alfaskop4120_state::du_pending_level() const
{
	for (int level = 7; level >= 0; level--)
		if (BIT(m_du_irq, level) && level >= m_du_imsk)
			return uint8_t(level);
	return NO_IRQ;
}

void alfaskop4120_state::du_update_irq()
{
	m_ducpu->set_input_line(M6800_IRQ_LINE, du_pending_level() != NO_IRQ ? ASSERT_LINE : CLEAR_LINE);
}

void alfaskop4120_state::du_set_imsk(uint8_t level)
{
	if (m_du_imsk != (level & 7))
	{
		m_du_imsk = level & 7;
		LOGIRQ("DU IRQ mask set to %d\n", m_du_imsk);
		du_update_irq();
	}
}

template <unsigned N> void alfaskop4120_state::du_irq_w(int state)
{
	m_du_irq = (m_du_irq & ~(1 << N)) | ((state ? 1 : 0) << N);
	LOGIRQ("%f DU IRQ %d: %d ==> %02x\n", machine().time().as_double(), N, state, m_du_irq);
	du_update_irq();
}

MC6845_UPDATE_ROW( alfaskop4120_state::du_crtc_update_row )
{
	offs_t const base = ma + 0x4000;
	u32 *px = &bitmap.pix(y);

	for (int i = 0; i < x_count; i++)
	{
		u8 chr = m_du_vram[(base + i) & 0x07ff] & 0x7f;
		rgb_t const bg = rgb_t::white();
		rgb_t const fg = rgb_t::black();

		u8 dots = m_du_chargen[chr * 16 + ra];

		// 9 dot positions per character cell (columns 0-8), techtext EE360-810C p.3:
		// "Each character position is divided into nine dot positions or columns
		// (col 0 to 8).  Seven columns (col 1 to 7) are used for character
		// presentation."  Bit 7 of the chargen byte is column 0 and is never a dot
		// (verified over the whole 2 kbyte chargen: 0 occurrences); bits 6-0 are
		// columns 1-7.  Column 8 has no bit in the ROM - it is the trailing gap.
		for (int n = 8; n > 0; n--, dots <<= 1)
			*px++ = BIT(dots, 7) ? fg : bg;
		// The chargen is active low, so in this code a 1-bit selects fg and that is
		// what a BLANK dot position looks like: fg is the background colour and bg
		// is the lit dot.  Column 8 is the inter-character gap, so it takes fg.
		*px++ = fg; // column 8: the gap between character cells
	}
}

void alfaskop4101_state::mem_map(address_map &map)
{
	map.unmap_value_high();
	map(0x0000, 0xefff).ram();
	map(0xf600, 0xf6ff).lrw8(NAME([this](offs_t offset) -> uint8_t { LOGNVRAM("nvram_r %04x: %02x\n", offset, 0); return (uint8_t) 0; }),
				 NAME([this](offs_t offset, uint8_t data) { LOGNVRAM("nvram_w %04x: %02x\n", offset, data); }));
	map(0xf7c4, 0xf7c7).mirror(0x00).lrw8(NAME([this](offs_t offset) -> uint8_t    { LOGMIC("MIC pia_r %04x: %02x\n", offset, 0); return m_mic_pia->read(offset & 3); }),
						  NAME([this](offs_t offset, uint8_t data) { LOGMIC("MIC pia_w %04x: %02x\n", offset, data); m_mic_pia->write(offset & 3, data); }));
	map(0xf800, 0xffff).rom().region("roms", 0);
}

/* Input ports */
static INPUT_PORTS_START( alfaskop4110 )
INPUT_PORTS_END

static INPUT_PORTS_START( alfaskop4120 )
INPUT_PORTS_END

static INPUT_PORTS_START( alfaskop4101 )
INPUT_PORTS_END

/* Interrupt handling - vector address modifyer, irq prioritizer and irq mask */

// Highest priority pending interrupt at or above the current mask level, or
// NO_IRQ when none of them is valid and the default vector should be used.
uint8_t alfaskop4110_state::pending_level() const
{
	for (int level = 7; level >= 0; level--)
		if (BIT(m_irq, level) && level >= m_imsk)
			return uint8_t(level);
	return NO_IRQ;
}

void alfaskop4110_state::update_irq()
{
	m_maincpu->set_input_line(M6800_IRQ_LINE, pending_level() != NO_IRQ ? ASSERT_LINE : CLEAR_LINE);
}

void alfaskop4110_state::set_imsk(uint8_t level)
{
	if (m_imsk != (level & 7))
	{
		m_imsk = level & 7;
		LOGIRQ("4110 IRQ mask set to %d\n", m_imsk);
		update_irq();
	}
}

template <unsigned N> void alfaskop4110_state::irq_w(int state)
{
	m_irq = (m_irq & ~(1 << N)) | ((state ? 1 : 0) << N);
	LOGIRQ("4110 IRQ %d: %d ==> %02x\n", N, state, m_irq);
	update_irq();
}

/* Simplified chargen, no attributes or special formats/features yet  */
MC6845_UPDATE_ROW( alfaskop4110_state::crtc_update_row )
{
	offs_t const base = ma + 0x4000;
	u32 *px = &bitmap.pix(y);

	for (int i = 0; i < x_count; i++)
	{
		u8 chr = m_vram[(base + i) & 0x07ff] & 0x7f;
		rgb_t const bg = rgb_t::white();
		rgb_t const fg = rgb_t::black();

		u8 dots = m_chargen[chr * 16 + ra];

		// 9 dot positions per character cell (columns 0-8), techtext EE360-810C p.3:
		// "Each character position is divided into nine dot positions or columns
		// (col 0 to 8).  Seven columns (col 1 to 7) are used for character
		// presentation."  Bit 7 of the chargen byte is column 0 and is never a dot
		// (verified over the whole 2 kbyte chargen: 0 occurrences); bits 6-0 are
		// columns 1-7.  Column 8 has no bit in the ROM - it is the trailing gap.
		for (int n = 8; n > 0; n--, dots <<= 1)
			*px++ = BIT(dots, 7) ? fg : bg;
		// The chargen is active low, so in this code a 1-bit selects fg and that is
		// what a BLANK dot position looks like: fg is the background colour and bg
		// is the lit dot.  Column 8 is the inter-character gap, so it takes fg.
		*px++ = fg; // column 8: the gap between character cells
	}
}

void alfaskop4110_state::alfaskop4110(machine_config &config)
{
	/* basic machine hardware */
	M6800(config, m_maincpu, XTAL(19'170'000) / 18); // Verified from service manual
	m_maincpu->set_addrmap(AS_PROGRAM, &alfaskop4110_state::mem_map);

	/* Interrupt controller and address modifier PLA */
	/*
	 * 82S100 data sheet
	 * -----------------
	 *
	 * The 82S100 is a bipolar, fuse-link programmable logic array. It uses the
	 * standard AND/OR/invert architecture to directly implement custom
	 * um-of-product logic equations.
	 *
	 * Each device consists of 16 dedicated inputs and 8 dedicated outputs. Each
	 * output is capable of being actively controlled by any or all of the 48
	 * product terms. The true, complement, or don't care condition of each of the
	 * 16 inputs ANDed together comprise one P-Term. All 48 P-Terms are then OR-d
	 * to each output. The user must then only select which P-Terms will activate
	 * an output by disconnecting terms which do not affect the output. In addition,
	 * each output can be fused as active high or active low.
	 *
	 * The 82S100 is fully TTL compatible and includes chip-enable control for
	 * expansion of input variables and output inhibit. It features three state
	 * outputs.
	 *
	 * Field programmable Ni-Cr links
	 * 16 inputs
	 * 8 outputs
	 * 48 product terms
	 * Commercial verion - N82S100 - 50ns max address access time
	 * Power dissipation - 600mW typ
	 * Input loading - 100uA max
	 * Chip enable input
	 * Three state outputs
	 *
	 *
	 */
	/*                   _____   _____
	 *        nc FE   1 |*    \_/     | 28  Vcc
	 *      IRQ7 I7   2 |             | 27  I8  mask 1
	 *      IRQ6 I6   3 |             | 26  I9  mask 2
	 *      IRQ5 I5   4 |             | 25  I10 mask 3
	 *      IRQ4 I4   5 |             | 24  I11 Address &== 1111 1111 111x xxxx
	 *      IRQ3 I3   6 |    82S100   | 23  I12 AI 1 A1
	 *      IRQ2 I2   7 |             | 22  I13 AI 2 A2
	 *      IRQ1 I1   8 |     IC50    | 21  I14 AI 3 A3
	 *      IRQ0 I0   9 |             | 20  I15 AI 4 A4
	 *        P4 F7  10 |  Interrupt  | 19  _CE
	 *   mask P3 F6  11 |  Controller | 18  F0   IRQ
	 *   mask P2 F5  12 |     PLA     | 17  F1   mask register
	 *   mask P1 F4  13 |             | 16  F2   interrupt latch
	 *          GND  14 |_____________| 15  F3   nc
	 */
	PLS100(config, m_pla);

	MC6845(config, m_crtc, XTAL(19'170'000) / 9);
	m_crtc->set_screen("screen");
	m_crtc->set_show_border_area(false);
	m_crtc->set_char_width(9);
	m_crtc->set_update_row_callback(FUNC(alfaskop4110_state::crtc_update_row));
	// VSYNC should goto IRQ1 through some logic involving MIC PIA CRA bits 0 ( 1 == enable) & 1 (1 == positive edge)
	//m_crtc->out_vsync_callback().set(FUNC(alfaskop4110_state::crtc_vsync);
	//m_crtc->out_vsync_callback().set([this](bool state) { LOGIRQ("CRTC VSYNC: %d\n", state); });
	//m_crtc->out_vsync_callback().set("irq1", FUNC(input_merger_device::in_w<1>));

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	// 100 character positions of 9 dots = 900 dots per sweep, 426 sweeps per
	// frame: 19.17 MHz / (900 * 426) = 50.000 Hz exactly.  techtext EE360-810C
	// p.3 and osref ch.9 INITAB (all six screen formats give 426 sweeps).
	// Visible: 80 * 9 = 720 by 25 lines * 16 sweeps = 400.
	m_screen->set_raw(19'170'000, 900, 0, 720, 426, 0, 400);
	m_screen->set_screen_update("crtc", FUNC(mc6845_device::screen_update));

	PIA6821(config, m_mic_pia); // Main board PIA
	m_mic_pia->cb1_w(0);
	m_mic_pia->cb2_handler().set([this](offs_t offset, uint8_t data) { LOGMIC("->MIC PIA: CB2 write %d\n", data); });

	/*
	 * MIC PIA interface
	 *
	 * Port A (DDRA=0x7a)
	 * 0 - PA0 input  - not used
	 * 1 - PA1 output - KB reset P11 pin 23 at connector  1 == KB reset           0 == no KB reset
	 * 2 - PA2 input  - MCP test mode                     1 == no test mode       0 == in test mode,
	 * 3 - PA3 output - not used (in DTC)
	 * 4 - PA4 output - not used (in DTC)
	 * 5 - PA5 output - Interrupt enable                  1 == Int. out on P1:7   0 == no Int. out
	 * 6 - PA6 output - I4 latch enable                   1 == I4 will be latched 0 == no I4 latch
	 * 7 - PA7 input  - Button/MCP NMI                    1 == NMI from DU button 0 == NMI from MCP P4:1=low
	 * Note: At initialization a KB reset pulse will be sent as DDRA contains all zeros: PA I functions as a
	 *       high impedance input: "active level" for KB reset generation.
	 *
	 * Port B (DDRB=0xff)
	 * 0 - PB0 output - Reset PC-error                    1 == Reset PC error FF  0 == Memory PC used
	 *                                                         or PC not used
	 * 1 - PB1 output - VMAX/VMA 1 MPU                    1 == VMAX gen by MPU    0 == VMA 1 gen by MPU
	 * 2 - PB2 output - VMAX/VMA 1 DMA                    1 == VMAX gen by DMA    0 == VMA 1 gen by DMA
	 * 3 - PB3 output - Display Memory                    1 == 4KB Display Memory 0 == 8KB Display Memory
	 * 4 - PB4 output - Option Character Generator        1 == Enabled to MIC bus 0 == Disabled from MIC bus
	 * 5 - PB5 output - MPU Addr                          1 == Mode 1             0 == Mode 0
	 * 6 - PB6 output - Reset                             1 == Reset all but MPU  0 == No reset
	 *                                                         and MIC PIA
	 * 7 - PB7 output - not used
	 */
	m_mic_pia->writepa_handler().set([this](offs_t offset, uint8_t data)
					{
						LOGMIC("->MIC PIA: Port A write %02x\n", data);
						LOGMIC(" PA1 - KBD reset %s\n", BIT(data, 1) ? "active" : "inactive");
						LOGMIC(" PA5 - Int out %s\n", BIT(data, 5) ? "enabled": "disabled");
						LOGMIC(" PA6 - I4 latch %s\n", BIT(data, 6) ? "enabled": "disabled");
					});

	m_mic_pia->writepb_handler().set([this](offs_t offset, uint8_t data)
					{
						LOGMIC("->MIC PIA: Port B write %02x\n", data);
						LOGMIC(" PB0 - Reset PC-error %s\n", BIT(data, 0) ? "active" : "inactive");
						LOGMIC(" PB1 - %s generated by MPU\n", BIT(data, 1) ? "VMAX" : "VMA 1");
						LOGMIC(" PB2 - %s generated by DMA\n", BIT(data, 2) ? "VMAX" : "VMA 1");
						LOGMIC(" PB3 - %sKB Display Memory\n", BIT(data, 3) ? "4" : "8");
						LOGMIC(" PB4 - Option Char Generator %s\n", BIT(data, 4) ? "enabled" : "disabled");
						LOGMIC(" PB5 - MPU Address Mode %s\n", BIT(data, 5) ? "1" : "0");
						LOGMIC(" PB6 - Reset of devices %s\n", BIT(data, 6) ? "active" : "inactive");
					});

	m_mic_pia->readpa_handler().set([this](offs_t offset) -> uint8_t
					{
						uint8_t data = (1U << 2); // MCU is not in test mode
						LOGMIC("<-MIC PIA: Port A read\n");
						LOGMIC(" PA2 - MCU test mode %s\n", BIT(data, 2) ? "inactive" : "active");
						return 0;
					});
	m_mic_pia->readpb_handler().set([this](offs_t offset) -> uint8_t { LOGMIC("<-MIC PIA: Port B read\n"); return 0;});
	m_mic_pia->ca1_w(0);
	m_mic_pia->ca2_w(0);

	PIA6821(config, m_dia_pia); // Display PIA, controls how the CRTC accesses memory etc
	m_dia_pia->cb1_w(0);
	m_dia_pia->cb2_handler().set([this](offs_t offset, uint8_t data) { LOGDIA("DIA PIA: CB2_w %d\n", data); });
	m_dia_pia->writepa_handler().set([this](offs_t offset, uint8_t data) { LOGDIA("DIA PIA: PA_w %02x\n", data); });
	m_dia_pia->writepb_handler().set([this](offs_t offset, uint8_t data) { LOGDIA("DIA PIA: PB_w %02x\n", data); });
	m_dia_pia->readpa_handler().set([this](offs_t offset) -> uint8_t { LOGDIA("DIA PIA: PA_r\n"); return 0;});
	m_dia_pia->readpb_handler().set([this](offs_t offset) -> uint8_t { LOGDIA("DIA PIA: PB_r\n"); return 0;});
	m_dia_pia->ca1_w(0);
	m_dia_pia->ca2_w(0);

	ACIA6850(config, m_kbd_acia, 0);
	//CLOCK(config, "acia_clock", ACIA_CLOCK).signal_handler().set(FUNC(alfaskop4110_state::write_acia_clock));
	m_kbd_acia->irq_handler().set("irq3", FUNC(input_merger_device::in_w<3>));

	MC6854(config, m_tia_adlc, XTAL(19'170'000) / 18); // TODO: attach IRQ by IRQ 7 through descrete interrupt prioritization instead
	//m_tia_adlc->out_irq_cb().set([this](bool state){ LOGDMA("TIA ADLC IRQ: %s\n", state == ASSERT_LINE ? "asserted" : "cleared"); m_maincpu->set_input_line(M6800_IRQ_LINE, state); });
	//m_tia_adlc->out_irq_cb().set([this](bool state){ LOGDMA("TIA ADLC IRQ: %s\n", state == ASSERT_LINE ? "asserted" : "cleared"); m_maincpu->set_input_line(M6800_IRQ_LINE, state); });
	m_tia_adlc->out_irq_cb().set("irq7", FUNC(input_merger_device::in_w<7>));
	m_tia_adlc->out_rdsr_cb().set([this](bool state){ LOGDMA("TIA ADLC RDSR: %d\n", state); m_tia_dma->dreq_w<1>(state); });
	m_tia_adlc->out_tdsr_cb().set([this](bool state){ LOGDMA("TIA ADLC TDSR: %d\n", state); m_tia_dma->dreq_w<0>(state); });

	MC6844(config, m_tia_dma, XTAL(19'170'000) / 18);
	//m_tia_dma->out_int_callback().set([this](bool state){ LOGDMA("TIA DMA IRQ: %d\n", state); }); // Used as DEND (end of dma) towards the ADLC through some logic
	m_tia_dma->out_drq1_callback().set([this](bool state){ LOGDMA("TIA DMA DRQ1: %d\n", state); m_tia_dma->dgrnt_w(state); });
	//m_tia_dma->out_drq2_callback().set([this](bool state){ LOGDMA("TIA DMA DRQ2: %d\n", state); }); // Not connected
	m_tia_dma->in_ior_callback<1>().set([this](offs_t offset) -> uint8_t { return m_tia_adlc->dma_r(); });
	m_tia_dma->out_memw_callback().set([this](offs_t offset, uint8_t data) { m_maincpu->space(AS_PROGRAM).write_byte(offset, data); });

	/* 74LS273 latch inputs of interruptt sources */
	INPUT_MERGER_ANY_HIGH(config, "irq0").output_handler().set(FUNC(alfaskop4110_state::irq_w<0>));
	INPUT_MERGER_ANY_HIGH(config, "irq1").output_handler().set(FUNC(alfaskop4110_state::irq_w<1>));
	INPUT_MERGER_ANY_HIGH(config, "irq2").output_handler().set(FUNC(alfaskop4110_state::irq_w<2>));
	INPUT_MERGER_ANY_HIGH(config, "irq3").output_handler().set(FUNC(alfaskop4110_state::irq_w<3>));
	INPUT_MERGER_ANY_HIGH(config, "irq4").output_handler().set(FUNC(alfaskop4110_state::irq_w<4>));
	INPUT_MERGER_ANY_HIGH(config, "irq5").output_handler().set(FUNC(alfaskop4110_state::irq_w<5>));
	INPUT_MERGER_ANY_HIGH(config, "irq6").output_handler().set(FUNC(alfaskop4110_state::irq_w<6>));
	INPUT_MERGER_ANY_HIGH(config, "irq7").output_handler().set(FUNC(alfaskop4110_state::irq_w<7>));
}

void alfaskop4110_state::machine_start()
{
	save_item(NAME(m_irq));
	save_item(NAME(m_imsk));

	m_poll_start_timer = timer_alloc(FUNC(alfaskop4110_state::poll_start), this);
	m_poll_start_timer->adjust(attotime::from_msec(5000));

	m_poll_bit_timer = timer_alloc(FUNC(alfaskop4110_state::poll_bit), this);
	m_poll_bit_timer->adjust(attotime::never);
}

// Debug - inserts a poll SDLC frame through the ADLC, it ends up at address 0x140 in RAM through DMA
TIMER_CALLBACK_MEMBER(alfaskop4110_state::poll_start)
{
	/* The serial transfer of 8 bits is complete. Now trigger INT7. */
	LOGADLC("Starting poll message\n");
	m_tia_adlc->set_rx(0);
	m_poll_bit_timer->adjust(attotime::from_hz(300000));
}

TIMER_CALLBACK_MEMBER(alfaskop4110_state::poll_bit)
{
	if (flank)
	{
		if (index != 0 && index != 7 && BIT(txBuf[index], (pos % 8)) && ones == 5)
		{
			LOGADLC("%d%c", 2, (pos % 8) == 7 ? '\n' : ' ');
			m_tia_adlc->set_rx(0);
			ones = 0;
		}
		else
		{
			LOGADLC("%d%c", BIT(txBuf[index], (pos % 8)), (pos % 8) == 7 ? '\n' : ' ');
			m_tia_adlc->set_rx(BIT(txBuf[index], (pos % 8)));
			if (index != 0 && index != 7 && BIT(txBuf[index], (pos % 8)))
				ones++;
			else
				ones = 0;
			pos++;
			index = pos / 8;
		}
	}
	m_tia_adlc->rxc_w(flank ? 1 : 0);
	if (index < 8)
		m_poll_bit_timer->adjust(attotime::from_hz(300000) / 2);
	flank = !flank;
}

void alfaskop4110_state::machine_reset()
{
	m_irq = 0x00;
}

/* FD4120 interrupt logic, see the Flexible Disk Unit chapter, Fig. 6 */
uint8_t alfaskop4120_state::pending_level() const
{
	for (int level = 7; level >= 0; level--)
		if (BIT(m_irq, level) && level >= m_imsk)
			return uint8_t(level);
	return NO_IRQ;
}

void alfaskop4120_state::update_irq()
{
	m_maincpu->set_input_line(M6800_IRQ_LINE, pending_level() != NO_IRQ ? ASSERT_LINE : CLEAR_LINE);
}

void alfaskop4120_state::set_imsk(uint8_t level)
{
	if (m_imsk != (level & 7))
	{
		m_imsk = level & 7;
		LOGIRQ("4120 IRQ mask set to %d\n", m_imsk);
		update_irq();
	}
}

template <unsigned N> void alfaskop4120_state::irq_w(int state)
{
	m_irq = (m_irq & ~(1 << N)) | ((state ? 1 : 0) << N);
	LOGIRQ("%f 4120 IRQ %d: %d ==> %02x\n", machine().time().as_double(), N, state, m_irq);
	update_irq();
}

/*
 * FDA PIA, Flexible Disk Unit chapter pages 18-19
 *
 * Port A                                     Port B
 *  0 in  Strap 0: 0 = IPL from the diskette   0 out ERROR 1 LED
 *  1 in  Strap 1: not used                    1 out ERROR 2 LED
 *  2 out Ready LED                            2 out ERROR 3 LED
 *  3 out Door unlock, drive 0                 3 out Door unlock, drive 1
 *  4 out Write fault reset, drive 0           4 out Write fault reset, drive 1
 *  5 out Head select, drive 0                 5 out Head select, drive 1
 *  6 in  Drive 0 connected (0 = connected)    6 in  Drive 1 connected
 *  7 in  Drive 0 disk type (0 = single sided) 7 in  Drive 1 disk type
 */
uint8_t alfaskop4120_state::fda_pa_r()
{
	uint8_t data = m_strap_ipl_source ? 0x01 : 0x00; // PA0: 1 = IPL over the two-wire, 0 = from the diskette
	if (!m_floppy[0]->get_device()) data |= (1U << 6); // no drive connected
	LOGIO("FDA PIA: port A read %02x\n", data);
	return data;
}

uint8_t alfaskop4120_state::fda_pb_r()
{
	uint8_t data = 0;
	if (!m_floppy[1]->get_device()) data |= (1U << 6);
	LOGIO("FDA PIA: port B read %02x\n", data);
	return data;
}

void alfaskop4120_state::fda_pa_w(uint8_t data)
{
	LOGIO("FDA PIA: port A write %02x - Ready LED %d, door unlock %d, head %d\n",
		  data, BIT(data, 2), !BIT(data, 3), BIT(data, 5));
	if (floppy_image_device *f = m_floppy[0]->get_device())
	{
		// the 8" drives spin continuously, there is no motor control on the FDA
		// board; done here rather than in machine_reset() because the floppy
		// device resets after the driver and would clear it again
		f->mon_w(0);
		f->ss_w(BIT(data, 5) ? 1 : 0);
	}
	m_fda_pa = data;
}

void alfaskop4120_state::fda_pb_w(uint8_t data)
{
	LOGIO("FDA PIA: port B write %02x - error LEDs %d%d%d, door unlock %d, head %d\n",
		  data, BIT(data, 2), BIT(data, 1), BIT(data, 0), !BIT(data, 3), BIT(data, 5));

	// Poor man's backtrace: when an error LED is lit, dump where we are and
	// what is on the 6800 stack, which holds the JSR return addresses.
	if ((data & 0x07) && !(m_fda_pb & 0x07))
	{
		uint16_t const sp = m_maincpu->state_int(M6800_S);
		LOGIO("*** ERROR LED %d%d%d lit at PC=%04X SP=%04X\n",
			  BIT(data, 2), BIT(data, 1), BIT(data, 0), m_maincpu->pc(), sp);
		for (int i = 1; i <= 24; i += 2)
		{
			uint16_t const w = (m_ram[(sp + i) & 0x7fff] << 8) | m_ram[(sp + i + 1) & 0x7fff];
			LOGIO("***   stack %04X: %04X%s\n", (sp + i) & 0xffff, w,
				  (w >= 0xe800 && w <= 0xefff) || (w >= 0xf800) ? "  <- return address" : "");
		}
	}
	if (floppy_image_device *f = m_floppy[1]->get_device())
	{
		f->mon_w(0);
		f->ss_w(BIT(data, 5) ? 1 : 0);
	}
	m_fda_pb = data;
}

/*
 * Synthetic SS3 configuration master.
 *
 * The flexible disk unit does not read its diskette on its own: it runs its
 * self tests, sets up both floppy controllers, arms the ADLC and waits to be
 * polled.  Until the CP 4101 is emulated this stands in for it, using the
 * frame level interface of the MC6854 so neither FFSK, bit stuffing nor CRC
 * are involved - exactly as the manual says the hardware handles all three.
 *
 * Poll message (Appendix 2):  Todev | Dsa | Frdev | Status
 *   Todev  FD = system flexible disk unit (Appendix 1)
 *   Frdev  C0 = communication processor, old operating systems (FE on late
 *          ones, in which case Dsa is 02 for an FD instead of FF)
 *   Status bit 7 set marks the byte as a Status rather than a Msgtyp; bit 3
 *          marks the leading poll of a series.
 */
TIMER_CALLBACK_MEMBER(alfaskop4120_state::ss3_poll)
{
	// Sweep the header fields until the unit answers: which physical address it
	// believes it has, and whether it expects an old (C0/FF) or late (FE/02)
	// operating system's addressing, is not established yet.
	// What the unit's own frame parser at FD27/FDAB accepts, read off the PROM:
	//   Frdev  must be FE (this PROM is a late operating system one, C0 is
	//          rejected outright)
	//   Status must have bits 7 and 3 set (poll, leading poll)
	//   Todev  must be FD, the system flexible disk unit
	//   Dsa    FF sets "polled" (bit 0 of the status word at 0208); 02 sets
	//          the word to 10, which is the branch that reads the header
	m_ss3_last[0] = 0xfd;
	m_ss3_last[1] = (m_ss3_polls & 1) ? 0x02 : 0xff;
	m_ss3_last[2] = 0xfe;
	m_ss3_last[3] = 0x88;

	int const rc = m_adlc->send_frame(m_ss3_last, sizeof(m_ss3_last));
	if (rc != 0 || m_ss3_polls < 6)
		LOGADLC("SS3 poll %u -> %02X %02X %02X %02X (%s)\n", m_ss3_polls,
				m_ss3_last[0], m_ss3_last[1], m_ss3_last[2], m_ss3_last[3],
				rc == 0 ? "aceptada" : rc == -2 ? "receptor en reset" : "ADLC ocupado");
	if ((m_ss3_polls % 100) == 0)
		LOGADLC("SS3 t=%.1f s: PC=%04X (%s)\n", machine().time().as_double(),
				m_maincpu->pc(), m_maincpu->pc() < 0x8000 ? "RWM - programa cargado" : "PROM");
	m_ss3_polls++;
	// the configuration master retransmits a poll once after 10 ms when there
	// is no answer, and then polls that unit far less often
	m_ss3_timer->adjust(attotime::from_msec(10));
}

void alfaskop4120_state::ss3_frame_from_unit(uint8_t *data, int length)
{
	std::string s;
	for (int i = 0; i < length && i < 32; i++)
		s += util::string_format("%02X ", data[i]);
	logerror("*** SS3 RESPUESTA de %d bytes: %s  (al poll %02X %02X %02X %02X)\n",
			 length, s, m_ss3_last[0], m_ss3_last[1], m_ss3_last[2], m_ss3_last[3]);

	// The FDA modem drops CTS when the line goes idle after the closing flag;
	// the unit's transmit interrupt handler polls exactly that bit (SR1.CTS) to
	// learn that its frame is out, and only then re-arms the receiver
	m_adlc->set_cts(1);

	// (the synthetic configuration master of the M4/M5 era stays disarmed: the
	// CP is the real master now, and a second poll source poisons the FD's
	// session bookkeeping)

	// keep a copy of everything the FD serves: the data frames are the CP's
	// operating software coming off the diskette
	if (length >= 4 && (data[3] & 0x8f) == 0x02)
	{
		m_ss3_data_frames++;
		logerror("*** SS3 data block %u: %d bytes from the FD to the CP\n", m_ss3_data_frames, length - 4);
	}

	// Half-duplex: the FD's frame now owns the two-wire, so frames still
	// waiting to be delivered to it were lost in the collision (their senders
	// heard the line busy and retry at protocol level).
	m_q_to_fd.clear();
	m_ss3_to_fd_timer->adjust(attotime::never);
	// and the carrier of the FD's answer is on the line before the drain of
	// anyone else's frame completes: no loss-of-carrier edges
	cp_cancel_tails();
	m_du_dcd_timer->adjust(attotime::never);

	// hand the frame to the other stations on the two-wire
	m_q_to_cp.push(data, length);
	m_ss3_to_cp_timer->adjust(ss3_wire_time(length));
	m_q_to_du.push(data, length);
	m_ss3_to_du_timer->adjust(ss3_wire_time(length));
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::ss3_to_cp)
{
	if (m_q_to_cp.count == 0)
		return;
	// every CP channel sits on the shared pair: whichever receivers are armed
	// hear the frame; the others simply do not see it
	bool delivered = false;
	for (auto &adlc : m_cp_adlc)
		if (adlc->send_frame(m_q_to_cp.front(), m_q_to_cp.front_len()) == 0)
		{
			adlc->set_cts(0); // carrier present at the receiving end
			delivered = true;
		}
	if (!delivered)
	{
		static unsigned fails = 0;
		if ((fails++ % 500) == 0)
			logerror("SS3 -> CP sin entregar (intento %u, %02X %02X %02X %02X)\n", fails,
					 m_q_to_cp.front()[0], m_q_to_cp.front()[1], m_q_to_cp.front()[2], m_q_to_cp.front()[3]);
		m_ss3_to_cp_timer->adjust(attotime::from_msec(2));
		return;
	}
	// the line went active again before the carrier of the CP's own frame had
	// fully drained: no loss-of-carrier edge, the answer window stays open
	cp_cancel_tails();
	LOGADLC("SS3 -> CP: %02X %02X %02X %02X (%d bytes)\n", m_q_to_cp.front()[0],
			m_q_to_cp.front()[1], m_q_to_cp.front()[2], m_q_to_cp.front()[3], m_q_to_cp.front_len());
	m_q_to_cp.pop();
	if (m_q_to_cp.count > 0)
		m_ss3_to_cp_timer->adjust(ss3_wire_time(m_q_to_cp.front_len()));
}

void alfaskop4120_state::ss3_frame_from_cp_ch(int ch, uint8_t *data, int length)
{
	std::string s;
	for (int i = 0; i < length && i < 40; i++)
		s += util::string_format("%02X ", data[i]);
	logerror("*** SS3 CP transmite (canal %d) %d bytes: %s\n", ch, length, s);

	// the CP's modem drops CTS when the line goes idle after its closing flag
	m_cp_adlc[ch]->set_cts(1);

	// half-duplex: frames still heading for the CP were lost in the collision
	m_q_to_cp.clear();
	m_ss3_to_cp_timer->adjust(attotime::never);
	m_du_dcd_timer->adjust(attotime::never);

	// the carrier of this channel's transmission drains a moment after its
	// receiver has been re-armed; that loss-of-carrier edge closes the answer
	// window and the polling code retransmits.  Only the IPL PROM relies on
	// it (the FA7C wait); the loaded operating software paces its timeouts
	// with the PTM and closes its session receivers if the edge is injected
	// while a peer is busy fetching from disk.
	cp_cancel_tails();
	if (m_cpcpu->pcbase() >= 0xf800)
		m_cp_dcd_timer[ch]->adjust(attotime::from_msec(60), ch);

	m_q_to_fd.push(data, length);
	m_ss3_to_fd_timer->adjust(ss3_wire_time(length));

	// EXPERIMENT - the two-wire connection number of the display unit.
	//
	// On the real cluster this is physical: a display only ever sees the
	// frames of the pair it is plugged into, and that is what ties the
	// diskette's LOGICAL ADDRESSES form to a particular unit.  The chain in
	// the firmware is
	//     port 8  ->  3270 address $21  ->  [$0C21] = SS3 station $01
	// and the communication processor polls station $01 for it.  Our bus was
	// collapsed to a broadcast, so the display answered whatever address it
	// was called with, settled on station 00 - which no configured port maps
	// to - and every selection ended in RVI.  Delivering only the frames of
	// its own pair is the missing piece of hardware.
	if (!m_ss3_du_port_filter)
	{
		m_q_to_du.push(data, length);
		m_ss3_to_du_timer->adjust(ss3_wire_time(length));
	}
	else if (length >= 1 && data[0] == m_ss3_du_port)
	{
		// PROBE - the poll of a configured station carries Dsa = the station
		// number ("01 01 FE E9"), and the display's IPL PROM only latches its
		// own address off a frame whose Dsa is 00 ($F946: LDAB $0141 ; BNE).
		// Forcing that one byte answers the question the trace left open: if
		// the display could come up on station 01, does the 3270 selection
		// close?  It is a probe, not a model of anything.
		uint8_t buf[64];
		int const n = std::min<int>(length, int(sizeof(buf)));
		std::copy_n(data, n, buf);
		if (!m_ss3_du_latched && n >= 4 && buf[1] == m_ss3_du_port && (buf[3] & 0x80) != 0)
			buf[1] = 0x00;
		m_q_to_du.push(buf, n);
		m_ss3_to_du_timer->adjust(ss3_wire_time(n));
	}
}


void alfaskop4120_state::ss3_frame_from_du(uint8_t *data, int length)
{
	std::string s;
	for (int i = 0; i < length && i < 40; i++)
		s += util::string_format("%02X ", data[i]);
	logerror("%f *** SS3 DU transmite %d bytes: %s\n", machine().time().as_double(), length, s);

	m_ss3_du_latched = true;
	m_du_adlc->set_cts(1);
	m_q_to_du.clear();
	m_ss3_to_du_timer->adjust(attotime::never);
	cp_cancel_tails();
	m_du_dcd_timer->adjust(attotime::from_msec(60));

	m_q_to_fd.push(data, length);
	m_ss3_to_fd_timer->adjust(ss3_wire_time(length));
	m_q_to_cp.push(data, length);
	m_ss3_to_cp_timer->adjust(ss3_wire_time(length));
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::ss3_to_du)
{
	if (m_q_to_du.count == 0)
		return;
	int const rc = m_du_adlc->send_frame(m_q_to_du.front(), m_q_to_du.front_len());
	if (rc != 0)
	{
		static unsigned fails = 0;
		if ((fails++ % 500) == 0)
			logerror("SS3 -> DU sin entregar (rc=%d, intento %u, %02X %02X %02X %02X)\n",
					 rc, fails, m_q_to_du.front()[0], m_q_to_du.front()[1], m_q_to_du.front()[2], m_q_to_du.front()[3]);
		m_ss3_to_du_timer->adjust(attotime::from_msec(2));
		return;
	}
	m_du_adlc->set_cts(0);
	m_du_dcd_timer->adjust(attotime::never);
	LOGADLC("SS3 -> DU: %02X %02X %02X %02X (%d bytes)\n", m_q_to_du.front()[0],
			m_q_to_du.front()[1], m_q_to_du.front()[2], m_q_to_du.front()[3], m_q_to_du.front_len());
	m_q_to_du.pop();
	if (m_q_to_du.count > 0)
		m_ss3_to_du_timer->adjust(ss3_wire_time(m_q_to_du.front_len()));
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::du_carrier_tail)
{
	m_du_adlc->set_dcd(1);
	m_du_adlc->set_dcd(0);
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::cp_carrier_tail)
{
	LOGADLC("%f SS3 CP carrier tail: DCD edge (channel %d)\n", machine().time().as_double(), int(param));
	m_cp_adlc[param]->set_dcd(1); // carrier lost: latches SR2.DCD and interrupts
	m_cp_adlc[param]->set_dcd(0); // and the line is quiet again
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::ss3_to_fd)
{
	if (m_q_to_fd.count == 0)
		return;
	int const rc = m_adlc->send_frame(m_q_to_fd.front(), m_q_to_fd.front_len());
	if (rc != 0)
	{
		static unsigned fails = 0;
		if ((fails++ % 500) == 0)
			logerror("SS3 -> FD sin entregar (rc=%d, intento %u, %02X %02X %02X %02X)\n",
					 rc, fails, m_q_to_fd.front()[0], m_q_to_fd.front()[1], m_q_to_fd.front()[2], m_q_to_fd.front()[3]);
		m_ss3_to_fd_timer->adjust(attotime::from_msec(2));
		return;
	}
	m_adlc->set_cts(0); // carrier present at the receiving end
	LOGADLC("SS3 -> FD: %02X %02X %02X %02X (%d bytes)\n", m_q_to_fd.front()[0],
			m_q_to_fd.front()[1], m_q_to_fd.front()[2], m_q_to_fd.front()[3], m_q_to_fd.front_len());
	m_q_to_fd.pop();
	if (m_q_to_fd.count > 0)
		m_ss3_to_fd_timer->adjust(ss3_wire_time(m_q_to_fd.front_len()));
}

void alfaskop4120_state::machine_start()
{
	save_item(NAME(m_irq));
	save_item(NAME(m_imsk));
	save_item(NAME(m_fda_pa));
	save_item(NAME(m_fda_pb));

	m_q_to_fd.name = "FD";
	m_q_to_cp.name = "CP";
	m_q_to_du.name = "DU";

	m_ss3_timer = timer_alloc(FUNC(alfaskop4120_state::ss3_poll), this);
	m_ss3_to_cp_timer = timer_alloc(FUNC(alfaskop4120_state::ss3_to_cp), this);
	m_ss3_to_fd_timer = timer_alloc(FUNC(alfaskop4120_state::ss3_to_fd), this);
	m_ss3_to_du_timer = timer_alloc(FUNC(alfaskop4120_state::ss3_to_du), this);
	for (auto &t : m_cp_dcd_timer)
		t = timer_alloc(FUNC(alfaskop4120_state::cp_carrier_tail), this);
	m_du_dcd_timer = timer_alloc(FUNC(alfaskop4120_state::du_carrier_tail), this);
	m_tick_timer = timer_alloc(FUNC(alfaskop4120_state::sys_tick), this);
	m_tick_timer->adjust(attotime::from_msec(10), 0, attotime::from_msec(10));
	m_du_acia_clk_timer = timer_alloc(FUNC(alfaskop4120_state::du_acia_clk), this);
	m_fd_rx_pace_timer = timer_alloc(FUNC(alfaskop4120_state::fd_rx_pace), this);
	m_du_rx_pace_timer = timer_alloc(FUNC(alfaskop4120_state::du_rx_pace), this);
	for (int i = 0; i < 4; i++)
		m_cp_rx_pace_timer[i] = timer_alloc(FUNC(alfaskop4120_state::cp_rx_pace), this);
	m_du_kbd_echo_timer = timer_alloc(FUNC(alfaskop4120_state::du_kbd_echo), this);
	// 4800 bps synchronous: ~1667 us per byte on the host line
	m_bsc_timer = timer_alloc(FUNC(alfaskop4120_state::bsc_host_tick), this);
	m_bsc_timer->adjust(attotime::from_usec(1667), 0, attotime::from_usec(1667));
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::fd_rx_pace)
{
	m_dma->dreq_w<1>(1);
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::du_rx_pace)
{
	m_du_dma->dreq_w<1>(1);
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::cp_rx_pace)
{
	switch (param)
	{
	case 0: m_cp_dma->dreq_w<0>(1); break;
	case 1: m_cp_dma->dreq_w<1>(1); break;
	case 2: m_cp_dma->dreq_w<2>(1); break;
	case 3: m_cp_dma->dreq_w<3>(1); break;
	}
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::bsc_host_tick)
{
	// one tick per byte time on a continuously clocked synchronous line

	// cluster -> host: whenever its transmitter runs the line is theirs
	// (half-duplex turnaround: the CP resets its receiver while sending,
	// so this must be checked before anything is gated on the receiver)
	if (!(m_cp_ssda_c1 & 0x02))
	{
		int tuf = 0;
		uint8_t b = m_cp_ssda->get_tx_byte(&tuf);
		if (!tuf && b != 0xff)
		{
			// the line is theirs: drop anything we had queued and
			// answer promptly once they finish (their receiver only
			// listens for a short window after a transmission)
			m_bsc_txq.clear();
			logerror("%f BSC HOST rx %02X\n", machine().time().as_double(), b);
			m_bsc_rx.push_back(b);
			m_bsc_listen = 40;
			m_bsc_settle = 60;
			return;
		}
	}

	// host -> cluster; 0xffff entries are line-idle byte times, 0xfffe is
	// "send SYN until the CP actually drains its rx FIFO" - the real
	// leading-SYN idle of a BSC transmission, which also paces us to the
	// firmware's readiness (its 3-deep FIFO overruns silently otherwise).
	// The bytes are held - not dropped - while the receiver cannot take
	// them (in reset or parked with Clear Sync: receive_byte discards).
	if (!m_bsc_txq.empty())
	{
		if (m_cp_ssda_c1 & 0x09)
			return;
		uint16_t b = m_bsc_txq.front();
		if (b == 0xfffe)
		{
			// pop on a read observed between token ticks: the CP is
			// awake and draining, so the body can follow.  Uses its
			// own bookkeeping - the in-flight tally gets reset by
			// every C1 write and cannot double as the pop detector.
			if (m_bsc_tok_armed && m_ssda_rd_count != m_bsc_tok_seen)
			{
				m_bsc_txq.erase(m_bsc_txq.begin());
				m_bsc_tok_seen = m_ssda_rd_count;
			}
			else if ((++m_bsc_syn_div & 31) == 0)
			{
				// gentle line idle, alternating mark-fill and SYN:
				// the firmware parks its sync register at FF after
				// transmitting (so tx underflow sends pad), meaning
				// its receiver re-syncs on the *idle line* first and
				// scans for SYN SYN in software; after a fresh
				// configuration the register holds 32 instead.
				// Sustained fill is what actually gets the CP to
				// attend the line (with a quiet FIFO it never
				// engages); the initial overrun garbage is fine in
				// an opening window - base $5174 just resyncs.
				// In-transaction responses are sent bare, so the
				// fill never poisons an ack window.
				uint8_t fill = (m_bsc_syn_div & 32) ? 0x32 : 0xff;
				logerror("%f BSC HOST line idle %02X (c1=%02X)\n", machine().time().as_double(), fill, m_cp_ssda_c1);
				m_cp_ssda->receive_byte(fill);
			}
			m_bsc_tok_seen = m_ssda_rd_count;
			m_bsc_tok_armed = true;
			return;
		}
		// bare (in-transaction) envelopes: prime at most three bytes
		// (the FIFO depth) into a window whose reader may engage a
		// second or twenty late, then WAIT for it to start reading -
		// dribbling on a timeout drowns a sleeping FIFO in overrun.
		// Opening envelopes skip this: their token+fill machinery
		// already paces them, and hunt-consumed SYNs would skew the
		// tally anyway.
		if (m_bsc_capped &&
		    m_bsc_sent - int(m_ssda_rd_count - m_bsc_rd_mark) >= 3)
			return;
		// lockstep: at most one un-read byte ahead of the CP; a slow
		// reader still gets one byte every 16 ticks, an engaged one
		// gets full byte rate
		if (m_ssda_rd_count == m_bsc_ls_mark && m_bsc_ls_wait < 16)
		{
			m_bsc_ls_wait++;
			return;
		}
		m_bsc_ls_mark = m_ssda_rd_count;
		m_bsc_ls_wait = 0;
		if (m_bsc_hunting)
		{
			// the line idles in the sync pattern until the receiver latches
			m_cp_ssda->receive_byte(m_cp_ssda_scr);
			m_bsc_hunting = false;
			return;
		}
		m_bsc_txq.erase(m_bsc_txq.begin());
		if (b != 0xffff)
		{
			m_cp_ssda->receive_byte(b & 0xff);
			m_bsc_sent++;
		}
		return;
	}
	if (m_bsc_listen > 0)
	{
		m_bsc_listen--;
		return;
	}

	if (m_bsc_settle > 0)
	{
		m_bsc_settle--;
		return;
	}
	m_bsc_settle = 300;    // ~0.5 s of line idle between polls

	// digest whatever the cluster answered to the previous transmission
	bool got_eot = false, got_ack = false, got_nak = false, got_wack = false;
	bool got_text = false, got_enq = false;
	if (!m_bsc_rx.empty())
	{
		std::string s;
		for (auto v : m_bsc_rx)
			s += util::string_format("%02X ", v);
		logerror("%f BSC HOST frame from cluster: %s\n", machine().time().as_double(), s);
		for (unsigned i = 0; i < m_bsc_rx.size(); i++)
		{
			uint8_t v = m_bsc_rx[i];
			if (v == 0x37) got_eot = true;
			if (v == 0x3d) got_nak = true;
			// only DLE 70/61 arm the CP for host text ($4274: an
			// ack send installs base $5282, whose STX case is the
			// text receiver); DLE 7C is grouped with 6B at $42A6 =
			// "selected but more status pending, poll me first"
			if (v == 0x10 && i + 1 < m_bsc_rx.size() &&
			    (m_bsc_rx[i + 1] == 0x70 || m_bsc_rx[i + 1] == 0x61))
				got_ack = true;
			if (v == 0x10 && i + 1 < m_bsc_rx.size() &&
			    (m_bsc_rx[i + 1] == 0x7c || m_bsc_rx[i + 1] == 0x6b))
				got_wack = true;
			if (v == 0x01 || v == 0x02)
				got_text = true;   // SOH/STX: a status or data block
			if (v == 0x2d)
				got_enq = true;    // ENQ: repeat your last ack
		}
		m_bsc_rx.clear();
	}

	// 3270 BSC control station, CU address 0 (the unit switches at F7FC
	// read zero; the scan confirmed only address 40 is read in full).
	// Each transmission is a proper envelope: pad, four SYN, body, pad.
	auto envelope = [this](std::initializer_list<uint8_t> body)
	{
		m_bsc_txq.clear();
		m_bsc_rd_mark = m_ssda_rd_count;
		m_bsc_sent = 0;
		m_bsc_tok_armed = false;
		m_bsc_capped = false;
		m_bsc_txq.push_back(0xfffe);   // SYN idle until the CP reads
		m_bsc_txq.push_back(0x32);     // one more SYN ahead of the body
		for (uint8_t b : body)
			m_bsc_txq.push_back(b);
		m_bsc_txq.push_back(0xff);
		m_bsc_env = m_bsc_txq;
	};
	// in-transaction response: the CP reopened its receiver for us the
	// moment its transmitter drained ($53FE sets $41FA, the TUF path runs
	// $5043: hunt, sync 32, base $522E) and the window is clean.  No idle
	// fill and no pads: in base $522E any byte outside the control set
	// triggers an immediate ENQ and closes the window - the fills that
	// wake a parked receiver are poison inside a transaction.
	auto bare = [this](std::initializer_list<uint8_t> body)
	{
		m_bsc_txq.clear();
		m_bsc_rd_mark = m_ssda_rd_count;
		m_bsc_sent = 0;
		m_bsc_capped = true;
		m_bsc_txq.push_back(0x32);
		m_bsc_txq.push_back(0x32);
		for (uint8_t b : body)
			m_bsc_txq.push_back(b);
		m_bsc_env = m_bsc_txq;
	};

	switch (m_bsc_phase)
	{
	case 0:
		// specific poll of CU 0 dev 0 until its pending status has been
		// collected and acknowledged; then try the selection.  A fresh
		// 3270 cluster WACKs every selection while it still has unread
		// status, and only a specific poll makes it hand the status over.
		if (got_text || got_enq)
		{
			// first block of a fresh transaction always takes 61:
			// the CP resets its alternation to "expect 61" in every
			// $4704 supervision window (JSR $47CF sets $421F=70)
			if (got_text)
				m_bsc_ack = 0x61;
			bare({ 0x10, m_bsc_ack });
			logerror("%f BSC HOST %s -> ack %02X (bare)\n", machine().time().as_double(),
				 got_text ? "status" : "ENQ", m_bsc_ack);
			m_bsc_cu = 0;               // reuse: ack retry counter
			m_bsc_phase = 3;
			break;
		}
		if (got_eot)
			m_bsc_polls_ok++;
		if (m_bsc_polls_ok >= 2 && !m_bsc_written)
		{
			// a leading EOT resets the rx state machine to base
			// $5174 and reopens the receiver ($522E sees 37 ->
			// $51EF); only then do address bytes parse as a
			// selection instead of bouncing off a stale $522E
			envelope({ 0x37 });
			for (int i = 0; i < 12; i++)
				m_bsc_txq.push_back(0xffff);
			uint8_t const sel[] = { 0x32, 0x32, 0x60, 0x60, 0x40, 0x40, 0x2d };
			m_bsc_txq.insert(m_bsc_txq.end(), std::begin(sel), std::end(sel));
			logerror("%f BSC HOST select CU 0 dev 0 (40)\n", machine().time().as_double());
			m_bsc_phase = 1;
		}
		else
		{
			envelope({ 0x37 });
			for (int i = 0; i < 12; i++)
				m_bsc_txq.push_back(0xffff);
			// CU 0 (40), device 8 (C8): the only configured port
			uint8_t const poll[] = { 0x32, 0x32, 0x40, 0x40, 0x40, 0x40, 0x2d, 0xff };
			m_bsc_txq.insert(m_bsc_txq.end(), std::begin(poll), std::end(poll));
			logerror("%f BSC HOST specific poll CU 0 dev 0 (40)\n", machine().time().as_double());
		}
		break;

	case 1:
		if (got_ack)
		{
			// Erase/Write, WCC = C3 (unlock keyboard, reset MDT), a protected
			// title, and an UNPROTECTED FIELD to type into.
			//
			// The unlock in the WCC is not enough on its own: a 3270 buffer
			// with no unprotected field is protected everywhere, so every key
			// is rejected and the display keeps its input-inhibit marker.
			// Measured that way (qT0: typing changed nothing).  A real host
			// application always defines its input fields, so the synthetic
			// one does too now.
			//
			// Orders: 11 = Set Buffer Address (two 6-bit coded bytes),
			// 1D = Start Field followed by the attribute, also 6-bit coded:
			// 60 = protected, 40 = unprotected.  Address 160 is row 3 col 1
			// (160>>6 = 2 -> C2, 160 & 63 = 32 -> 60).
			static uint8_t const text[] = {
				0x02,                   // STX
				0x27, 0xf5, 0xc3,       // ESC, Erase/Write, WCC
				0x11, 0x40, 0x40,       // SBA to 0
				0x1d, 0x60,             // SF, protected: the title
				0xc1, 0xd3, 0xc6, 0xc1, 0xe2, 0xd2, 0xd6, 0xd7, // ALFASKOP
				0x40, 0xf4, 0xf1,       // " 41"
				0x40, 0xd6, 0xd5, 0xd3, 0xc9, 0xd5, 0xc5,       // " ONLINE"
				0x11, 0xc2, 0x60,       // SBA to 160 (row 3, col 1)
				0x1d, 0x40,             // SF, unprotected: where the operator types
				0x13,                   // IC, put the cursor in that field
				0x03 };                 // ETX
			uint16_t crc = 0;
			for (unsigned i = 1; i < sizeof(text); i++)   // after STX, ETX included
			{
				crc ^= text[i];
				for (int b = 0; b < 8; b++)
					crc = (crc & 1) ? (crc >> 1) ^ 0xa001 : (crc >> 1);
			}
			m_bsc_txq.clear();
			m_bsc_rd_mark = m_ssda_rd_count;
			m_bsc_sent = 0;
			m_bsc_capped = true;
			m_bsc_txq.push_back(0x32);
			m_bsc_txq.push_back(0x32);
			for (uint8_t b : text)
				m_bsc_txq.push_back(b);
			m_bsc_txq.push_back(crc & 0xff);
			m_bsc_txq.push_back(crc >> 8);
			m_bsc_env = m_bsc_txq;
			logerror("%f BSC HOST erase/write + text (crc %04X)\n",
				 machine().time().as_double(), crc);
			m_bsc_phase = 2;
		}
		else
		{
			// RVI or silence: back to polling for status.  ★When the mc6852
			// underflow interrupt is put back (see mc6852.cpp) this retry
			// becomes too eager: the line then turns around in a second
			// instead of twenty, the retries land on top of the display's own
			// file transfers, and one of them arriving while it loaded its
			// printer program wedged the transfer (qL0).  m_bsc_settle here,
			// and a later first poll, were tried and are not enough on their
			// own (qL1-qL3).
			logerror("%f BSC HOST select: %s - poll for status\n",
				 machine().time().as_double(),
				 got_wack ? "WACK" : "no answer");
			m_bsc_polls_ok = 0;
			m_bsc_phase = 0;
		}
		break;

	case 2:
		logerror("%f BSC HOST after text: ack=%d nak=%d eot=%d\n",
			 machine().time().as_double(), got_ack, got_nak, got_eot);
		if (got_ack)
		{
			m_bsc_ack ^= 0x11;
			m_bsc_written = true;   // screen is up: from here on, only poll
		}
		bare({ 0x37 });         // EOT closes the transaction
		m_bsc_phase = 0;
		m_bsc_polls_ok = 0;     // resume polling
		break;

	case 3:
		// the ack stays on offer until the transaction closes
		if (got_eot)
		{
			logerror("%f BSC HOST status transaction closed\n",
				 machine().time().as_double());
			m_bsc_ack ^= 0x11;      // alternation advances on success
			m_bsc_polls_ok = 2;     // straight to the selection
			m_bsc_phase = 0;
		}
		else if (got_text || got_enq || m_bsc_cu < 30)
		{
			m_bsc_cu++;
			if (got_text)
				m_bsc_ack ^= 0x11;   // next block within the transaction
			bare({ 0x10, m_bsc_ack });
			logerror("%f BSC HOST ack %02X on offer (%d)\n",
				 machine().time().as_double(), m_bsc_ack, m_bsc_cu);
		}
		else
		{
			logerror("%f BSC HOST ack never taken, back to polls\n",
				 machine().time().as_double());
			m_bsc_polls_ok = 0;
			m_bsc_phase = 0;
		}
		break;
	}
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::du_kbd_echo)
{
	m_du_kbd_acia->write_rxd(int(param));
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::du_acia_clk)
{
	m_du_acia_clk_level = !m_du_acia_clk_level;
	m_du_kbd_acia->write_txc(m_du_acia_clk_level ? 1 : 0);
	m_du_kbd_acia->write_rxc(m_du_acia_clk_level ? 1 : 0);
}

TIMER_CALLBACK_MEMBER(alfaskop4120_state::sys_tick)
{
	// one active edge on CA1 per cycle whichever polarity the software
	// programs in CRA bit 1.  The DTC board carries the same discrete system
	// timer as the FDP: without it the DU nucleus hangs in its tick wait
	// ($03CA never cleared by the tick ISR at $24E3) and NIP never proceeds.
	m_tick = !m_tick;
	m_mic_pia->ca1_w(m_tick ? 1 : 0);
	m_du_mic_pia->ca1_w(m_tick ? 1 : 0);
}

void alfaskop4120_state::machine_reset()
{
	m_du_acia_clk_on = false;
	m_du_acia_clk_timer->adjust(attotime::never);
	m_irq = 0;
	m_imsk = 0;
	// one controller per drive, so the selection is static
	m_fdc[0]->set_floppy(m_floppy[0]->get_device());
	m_fdc[1]->set_floppy(m_floppy[1]->get_device());
	// 8" drives spin all the time, there is no motor control on the FDA board
	for (auto &conn : m_floppy)
		if (floppy_image_device *f = conn->get_device())
			f->mon_w(0);

	// The CP is the configuration master now, so the synthetic poll source
	// stays quiet; it can be re-armed here for FD-only experiments.
	m_ss3_polls = 0;
	m_ss3_timer->adjust(attotime::never);
	// Careful with the polarity of these two in the MC6854 model: both set a
	// status bit meaning "the bad thing happened", not "the line is asserted".
	// set_cts(1) sets SR1.CTS, which keeps TDRA cleared and silences the
	// transmitter for good; set_dcd(1) sets SR2.DCD, which blocks reception.
	m_adlc->set_cts(0); // the modem on the FDA board is clear to send
	m_adlc->set_dcd(0); // and there is carrier on the two-wire

	// the ACA's line drivers hold both handshakes asserted with nothing
	// plugged into the printer port (these are active low on the 6850)
	m_du_aca_acia->write_cts(0);
	m_du_aca_acia->write_dcd(0);
	for (auto &adlc : m_cp_adlc)
	{
		adlc->set_cts(0);
		adlc->set_dcd(0);
	}
	m_cp_irq = 0;
	m_cp_imsk = 0;
	for (int i = 0; i < 4; i++)
	{
		m_cp_rdsr[i] = false;
		m_cp_tdsr[i] = false;
		m_cp_rx_tail[i] = false;
	}
	m_du_adlc->set_cts(0);
	m_du_adlc->set_dcd(0);
	m_du_irq = 0;
	m_du_imsk = 0;
	m_du_rx_tail = false;
}

static void alfaskop_floppies(device_slot_interface &device)
{
	device.option_add("8sssd", FLOPPY_8_SSSD); // 77 tracks, 26 sectors of 128 bytes, IBM 3740
}

/* Raw 77 x 26 x 128 image of an IBM 3740 diskette, as dumped from the 4016
   system diskette.  The generic IMG format is the same size but is an Intel
   MDS-II diskette, whose tracks carry interleave 6 plus a per-track skew;
   the 3740 medium is 1:1 sequential (verified: the volume label sits in
   track 0 sector 1 and the VTOC chains match the raw offsets).  Through the
   MDS layout a full-track Read Multiple costs almost six revolutions, which
   overruns the disk timeouts of the unit's IPL PROM. */
class alfaskop_format : public floppy_image_format_t
{
public:
	virtual int identify(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants) const override
	{
		uint64_t size;
		if (io.length(size) || size != 77 * 26 * 128)
			return 0;
		if (form_factor != floppy_image::FF_8 && form_factor != floppy_image::FF_UNKNOWN)
			return 0;
		return FIFID_SIZE;
	}

	virtual bool load(util::random_read &io, uint32_t form_factor, const std::vector<uint32_t> &variants, floppy_image &image) const override
	{
		auto const [err, data, actual] = util::read_at(io, 0, 77 * 26 * 128);
		if (err || actual != 77 * 26 * 128)
			return false;
		image.set_variant(floppy_image::SSSD);
		for (int cyl = 0; cyl < 77; cyl++)
		{
			desc_pc_sector sects[26];
			for (int s = 0; s < 26; s++)
			{
				sects[s].track = cyl;
				sects[s].head = 0;
				sects[s].sector = s + 1; // 1:1, no interleave, no skew
				sects[s].size = 0;
				sects[s].actual_size = 128;
				sects[s].data = &data[(cyl * 26 + s) * 128];
				sects[s].deleted = false;
				sects[s].bad_addr_crc = false;
				sects[s].bad_data_crc = false;
			}
			// same cell count and gaps as the generic 8" SD FM layout
			build_pc_track_fm(cyl, 0, image, 83333, 26, sects, 33, 46, 32, 11);
		}
		return true;
	}

	virtual const char *name() const noexcept override { return "alfaskop"; }
	virtual const char *description() const noexcept override { return "Alfaskop 4016 IBM 3740 disk image"; }
	virtual const char *extensions() const noexcept override { return "img"; }
	virtual bool supports_save() const noexcept override { return false; }
};

static const alfaskop_format FLOPPY_ALFASKOP_FORMAT;

static void alfaskop_floppy_formats(format_registration &fr)
{
	fr.add_fm_containers();
	fr.add(FLOPPY_ALFASKOP_FORMAT);
}

void alfaskop4120_state::alfaskop4120(machine_config &config)
{
	/* basic machine hardware */
	M6800(config, m_maincpu, XTAL(19'170'000) / 18); // Verified from service manual
	m_maincpu->set_addrmap(AS_PROGRAM, &alfaskop4120_state::mem_map);

	PIA6821(config, m_mic_pia); // Main Board PIA
	// MIC PIA CA2 output drives the software interrupt I0, active low: the
	// loaded operating software posts an event with CRA=$37 (CA2 low) and its
	// dispatcher, entered through the I0 vector, releases it with CRA=$3F
	m_mic_pia->ca2_handler().set([this](int state) { irq_w<0>(state ? 0 : 1); });
	// the periodic timer arrives on CA1 and leaves through IRQA as interrupt I1
	m_mic_pia->irqa_handler().set(FUNC(alfaskop4120_state::irq_w<1>));
	PIA6821(config, m_fdapia); // Floppy Disk PIA on the FDA board
	m_fdapia->readpa_handler().set(FUNC(alfaskop4120_state::fda_pa_r));
	m_fdapia->readpb_handler().set(FUNC(alfaskop4120_state::fda_pb_r));
	m_fdapia->writepa_handler().set(FUNC(alfaskop4120_state::fda_pa_w));
	m_fdapia->writepb_handler().set(FUNC(alfaskop4120_state::fda_pb_w));

	// ADLC on the FDA board, the unit's connection to the SS3 two-wire bus
	// The two-wire runs at 300 kbit/s, so a byte is 26.67 us.  Telling the
	// ADLC that makes it clock a received frame into its FIFO on a timer
	// instead of refilling it as the reader pops, which is what the line does:
	// a receiver that stops - its DMA count exhausted on a frame that was not
	// for it, or its CPU slow to re-arm at a 64-byte boundary - then overruns
	// the three-byte FIFO by itself, for the right reason and at the right
	// moment, instead of needing the stall detector to guess.
	MC6854(config, m_adlc, XTAL(19'170'000) / 18);
	// DESARMADO: m_adlc->set_wire_rate(attotime::from_nsec(26667));
	m_adlc->out_irq_cb().set(FUNC(alfaskop4120_state::irq_w<4>));
	// the IPL programs DMAC channel 1 with a 64 byte buffer for the ADLC, which
	// is the standard receive buffer the Communication chapter talks about
	m_adlc->out_rdsr_cb().set([this](int state) { m_dma->dreq_w<1>(state); });
	m_adlc->out_tdsr_cb().set([this](int state) { m_dma->dreq_w<0>(state); });
	// the FDA modem grants CTS again as soon as RTS is raised for a new frame
	m_adlc->out_rts_cb().set([this](int state) { if (state) m_adlc->set_cts(0); });
	m_adlc->set_out_frame_callback(FUNC(alfaskop4120_state::ss3_frame_from_unit));

	MC6844(config, m_dma, XTAL(19'170'000) / 18);
	// On the "09" disk timeout, so the next attempt does not redo the work:
	// the FD1771 gives its data request one byte time (32 us at 250 kbit/s)
	// and in a 200 s run exactly FOUR requests out of 123475 go unserved,
	// all of them at 33-34 us against a 2 us median.  Three explanations were
	// tried and all three are refuted by measurement:
	//   * the ADLC transmit burst starving the channel - pacing it, and even
	//     releasing the bus between bytes as mode 1 requires, leaves the four
	//     stalls at exactly the same timestamps (qH3 vs qE7);
	//   * a perfect quantum on this DMAC - far worse, 40 timeouts (qH5);
	//   * a 10 us maximum quantum for the machine - one stall of 34 us
	//     survives it, so it is not a scheduler timeslice either (qH7).
	// During the stall the FD's processor is in its ordinary idle loop
	// ($2942-$295B), so nothing is holding the bus.  What is left to look at
	// is the mc6844 model itself: whether a channel that is mid-sequence can
	// swallow another channel's request.
	m_dma->out_int_callback().set(FUNC(alfaskop4120_state::irq_w<7>));
	m_dma->in_ior_callback<1>().set([this]() -> uint8_t
			{
				uint8_t const data = m_adlc->dma_r();
				// mc6854_device::read() drops the receive DMA request after every
				// data read and refreshes neither status register, so the request
				// has to be driven from here or only the first byte of a frame
				// gets moved.  SR2's FV goes up when the byte now at the head of
				// the FIFO is the last one of the frame, so exactly one more
				// transfer is due the first time we see it.
				bool const fv = BIT(m_adlc->read(1), 1);
				if (!fv || !m_ss3_rx_tail)
				{
					m_ss3_rx_tail = fv;
					// Pace the drain at the line rate, always.  A byte on the
					// two-wire is 26.67 us; re-raising the request instantly
					// both monopolizes the DMAC (the FDC's DRQ window is one
					// byte time, 32 us, so it loses disk bytes during
					// multi-sector reads) and makes a 3328-byte block arrive
					// in a few microseconds instead of the 89 ms the wire
					// takes.  This is the overlapping half of the model: the
					// head of the frame is handed over after one byte time
					// (ss3_wire_time) and the tail is metered out here, so
					// transmission and reception run together.
					m_dma->dreq_w<1>(0);
					m_fd_rx_pace_timer->adjust(attotime::from_nsec(26667));
				}
				else
				{
					m_ss3_rx_tail = false;
					// the frame is complete: drop this channel's request so it
					// stops competing with the disk ones, and force the device
					// to recompute its status so that FV reaches S2RQ and raises
					// the interrupt, which mc6854_device does not do by itself
					m_dma->dreq_w<1>(0);
					m_adlc->set_cts(0);
				}
				return data;
			});
	m_dma->out_drq1_callback().set([this](bool state) { m_dma->dgrnt_w(state); });
	// ADLC transmit.  On the FDA board the DMAC's DEND redirects the last byte
	// of the buffer to the ADLC's last-data register, which closes the frame.
	m_dma->out_iow_callback<0>().set([this](uint8_t data)
			{
				uint16_t const remaining = (m_dma->read(0x02) << 8) | m_dma->read(0x03);
				LOGDMA("DMA ch0 -> ADLC %s: %02X\n", remaining == 1 ? "last-data" : "data", data);
				m_adlc->write(remaining == 1 ? 3 : 2, data);
			});
	// the inverting buffers between the FD1771s and the data bus apply to the
	// DMA path as well, so undo the device model's own inversion here too
	m_dma->in_ior_callback<2>().set([this]() { return m_fdc[0]->data_r() ^ 0xff; });
	m_dma->in_ior_callback<3>().set([this]() { return m_fdc[1]->data_r() ^ 0xff; });
	m_dma->out_iow_callback<2>().set([this](uint8_t data) { m_fdc[0]->data_w(data ^ 0xff); });
	m_dma->out_iow_callback<3>().set([this](uint8_t data) { m_fdc[1]->data_w(data ^ 0xff); });
	m_dma->out_memw_callback().set([this](offs_t offset, uint8_t data) { LOGDMA("DMA -> %04X: %02X\n", offset, data); m_maincpu->space(AS_PROGRAM).write_byte(offset, data); });
	m_dma->in_memr_callback().set([this](offs_t offset) { return m_maincpu->space(AS_PROGRAM).read_byte(offset); });

	// Two FD1771, one per drive.  The data separator is external and the step
	// motor is driven through the STEP/DIRC pins (3PM tied high).
	FD1771(config, m_fdc[0], 4_MHz_XTAL / 2);
	m_fdc[0]->intrq_wr_callback().set([this](int state) { if (state) m_disk_busy = false; irq_w<6>(state); }); // EXPERIMENT
	m_fdc[0]->drq_wr_callback().set([this](int state)
			{
				LOGFDC("%f FDC0 DRQ %d\n", machine().time().as_double(), state);
				m_dma->dreq_w<2>(state);
			});
	m_fdc[0]->set_force_ready(true); // the drives spin from power on, so READY only depends on the media
	FD1771(config, m_fdc[1], 4_MHz_XTAL / 2);
	m_fdc[1]->intrq_wr_callback().set([this](int state) { if (state) m_disk_busy = false; irq_w<5>(state); });
	m_fdc[1]->drq_wr_callback().set([this](int state)
			{
				m_dma->dreq_w<3>(state);
			});
	m_fdc[1]->set_force_ready(true);

	FLOPPY_CONNECTOR(config, m_floppy[0], alfaskop_floppies, "8sssd", alfaskop_floppy_formats).enable_sound(false);
	// the unit takes one or two drives; the second is absent by default, and the
	// operating software waits forever for its index pulses if it is reported
	// as connected with no diskette spinning in it
	FLOPPY_CONNECTOR(config, m_floppy[1], alfaskop_floppies, nullptr, alfaskop_floppy_formats).enable_sound(false);

	/* ---- CP 4101, the communication processor, on the same SS3 two-wire ---- */
	M6800(config, m_cpcpu, XTAL(19'170'000) / 18);
	m_cpcpu->set_addrmap(AS_PROGRAM, &alfaskop4120_state::cp_mem_map);

	PIA6821(config, m_cp_mic_pia);
	// CA2 output is the software interrupt I0, active low, same as on the FD
	m_cp_mic_pia->ca2_handler().set([this](int state) { cp_irq_w<0>(state ? 0 : 1); });

	PIA6821(config, m_cp_cs_pia); // crosspoint selection (TAB board)

	PTM6840(config, m_cp_ptm, XTAL(19'170'000) / 18);
	m_cp_ptm->irq_callback().set(FUNC(alfaskop4120_state::cp_irq_w<1>));

	// the SSDA on the SCA board; its interrupt is I3, latched per MIC PIA PA4
	MC6852(config, m_cp_ssda, XTAL(19'170'000) / 18);
	m_cp_ssda->irq_callback().set(FUNC(alfaskop4120_state::cp_irq_w<3>));
	// the synthetic BSC host pulls the transmitter byte by byte
	m_cp_ssda->set_tx_pull_mode(true);

	// Four ADLCs, one per two-wire channel; channels 0-3 are interrupts I7-I4
	MC6854(config, m_cp_adlc[0], XTAL(19'170'000) / 18);
	// DESARMADO: m_cp_adlc[0]->set_wire_rate(attotime::from_nsec(26667));
	m_cp_adlc[0]->out_irq_cb().set(FUNC(alfaskop4120_state::cp_irq_w<7>));
	MC6854(config, m_cp_adlc[1], XTAL(19'170'000) / 18);
	// DESARMADO: m_cp_adlc[1]->set_wire_rate(attotime::from_nsec(26667));
	m_cp_adlc[1]->out_irq_cb().set(FUNC(alfaskop4120_state::cp_irq_w<6>));
	MC6854(config, m_cp_adlc[2], XTAL(19'170'000) / 18);
	// DESARMADO: m_cp_adlc[2]->set_wire_rate(attotime::from_nsec(26667));
	m_cp_adlc[2]->out_irq_cb().set(FUNC(alfaskop4120_state::cp_irq_w<5>));
	MC6854(config, m_cp_adlc[3], XTAL(19'170'000) / 18);
	// DESARMADO: m_cp_adlc[3]->set_wire_rate(attotime::from_nsec(26667));
	m_cp_adlc[3]->out_irq_cb().set(FUNC(alfaskop4120_state::cp_irq_w<4>));

	// DMAC channel N serves ADLC channel N, both directions.  The crosspoint
	// is collapsed into a broadcast: every channel reaches the single two-wire
	// pair that the FD and the DU hang from, and each receiver's own address
	// filter sorts out what is for it - which is how a shared pair works.
	MC6844(config, m_cp_dma, XTAL(19'170'000) / 18);
	m_cp_dma->out_int_callback().set([this](int state) { LOGDMA("CP DMAC DEND IRQ: %d\n", state); });
	m_cp_dma->out_drq1_callback().set([this](bool state) { m_cp_dma->dgrnt_w(state); });
	m_cp_dma->in_memr_callback().set([this](offs_t offset) { return m_cpcpu->space(AS_PROGRAM).read_byte(offset); });
	m_cp_dma->out_memw_callback().set([this](offs_t offset, uint8_t data) { LOGDMA("CP DMA -> %04X: %02X\n", offset, data); m_cpcpu->space(AS_PROGRAM).write_byte(offset, data); });

	auto wire_cp_channel = [this]<unsigned CH>(std::integral_constant<unsigned, CH>)
	{
		m_cp_adlc[CH]->out_rdsr_cb().set([this](int state) { m_cp_rdsr[CH] = state; m_cp_dma->dreq_w<CH>((m_cp_rdsr[CH] || m_cp_tdsr[CH]) ? 1 : 0); });
		m_cp_adlc[CH]->out_tdsr_cb().set([this](int state) { m_cp_tdsr[CH] = state; m_cp_dma->dreq_w<CH>((m_cp_rdsr[CH] || m_cp_tdsr[CH]) ? 1 : 0); });
		// the TUA modem grants CTS again as soon as RTS is raised
		m_cp_adlc[CH]->out_rts_cb().set([this](int state) { if (state) m_cp_adlc[CH]->set_cts(0); });
		m_cp_adlc[CH]->set_out_frame_callback(FUNC(alfaskop4120_state::ss3_frame_from_cp_t<CH>));
		m_cp_dma->in_ior_callback<CH>().set([this]() -> uint8_t
				{
					uint8_t const data = m_cp_adlc[CH]->dma_r();
					bool const fv = BIT(m_cp_adlc[CH]->read(1), 1);
					if (!fv || !m_cp_rx_tail[CH])
					{
						m_cp_rx_tail[CH] = fv;
						// NOT paced.  The processor loads itself with its IPL
						// PROM, whose answer window is short and re-arms at 63
						// bytes on timeout, so metering its drain at the line
						// rate makes a 3328-byte block outlast the window and
						// the IPL never completes (measured: the display stays
						// at LOAD for 400 s, qU4).
						m_cp_dma->dreq_w<CH>(1);
					}
					else
					{
						m_cp_rx_tail[CH] = false;
						m_cp_rdsr[CH] = false;
						m_cp_dma->dreq_w<CH>(m_cp_tdsr[CH] ? 1 : 0);
						m_cp_adlc[CH]->set_cts(0);
					}
					return data;
				});
		m_cp_dma->out_iow_callback<CH>().set([this](uint8_t data)
				{
					uint16_t const remaining = (m_cp_dma->read(0x02 + (CH << 2)) << 8) | m_cp_dma->read(0x03 + (CH << 2));
					LOGDMA("CP DMA ch%d -> ADLC %s: %02X\n", CH, remaining == 1 ? "last-data" : "data", data);
					m_cp_adlc[CH]->write(remaining == 1 ? 3 : 2, data);
				});
	};
	wire_cp_channel(std::integral_constant<unsigned, 0>());
	wire_cp_channel(std::integral_constant<unsigned, 1>());
	wire_cp_channel(std::integral_constant<unsigned, 2>());
	wire_cp_channel(std::integral_constant<unsigned, 3>());

	/* ---- DU 4110, the display unit, on the same two-wire (up to three
	   units per pair is a documented configuration) ---- */
	M6800(config, m_ducpu, XTAL(19'170'000) / 18);
	m_ducpu->set_addrmap(AS_PROGRAM, &alfaskop4120_state::du_mem_map);

	PIA6821(config, m_du_mic_pia);
	// same soft interrupt convention as the other units: CA2 low = I0
	m_du_mic_pia->ca2_handler().set([this](int state) { du_irq_w<0>(state ? 0 : 1); });
	// the system timer enters on CA1 and interrupts as I1, as on the FDP
	m_du_mic_pia->irqa_handler().set(FUNC(alfaskop4120_state::du_irq_w<1>));
	// PA1 holds the keyboard MCU in reset while high
	m_du_mic_pia->writepa_handler().set([this](uint8_t data)
			{ m_du_kbd->rst_line_w(BIT(data, 1) ? ASSERT_LINE : CLEAR_LINE); });
	PIA6821(config, m_du_dia_pia);
	ACIA6850(config, m_du_kbd_acia, 0);
	m_du_kbd_acia->irq_handler().set(FUNC(alfaskop4120_state::du_irq_w<3>));

	/* ---- ACA, the printer interface (E34193 2000) ---- */
	// The 6840 runs off the processor's own E clock: the loaded software
	// programs CR1 = $82 (bit 1 set = internal clock, bit 7 = output enable,
	// continuous mode) and loads a 16-bit divisor from a baud table - $006E
	// for 300, $0036 for 600, $001B for 1200.  In continuous mode the output
	// toggles every N+1 counts, so with E = 19.17 MHz / 18 = 1.065 MHz the
	// timer gives 1065000 / (2 * 111) = 4797 Hz for the 300 baud entry, and
	// the 6850 divides that by 16: 299.8 baud.  The other entries land just
	// as close, which is what identifies the clock.
	PTM6840(config, m_du_aca_ptm, XTAL(19'170'000) / 18);
	ACIA6850(config, m_du_aca_acia, 0);
	// O1 = Tclock, O2 = Rclock on the schematic, straight into the 6850
	m_du_aca_ptm->o1_callback().set([this](int state) { m_du_aca_acia->write_txc(state); });
	m_du_aca_ptm->o2_callback().set([this](int state) { m_du_aca_acia->write_rxc(state); });
	PIA6821(config, m_du_aca_pia);
	// IRQA and IRQB of the 6821 are tied together with the 6840's and the
	// 6850's onto one IRQ, taken to either IRQ 2 or IRQ 4 of the display by
	// jumper J3.  Nothing is attached to the printer port yet, so the line is
	// deliberately left unwired: an ACIA with no peer would otherwise sit
	// there raising TDRE interrupts nobody asked for.

	// The KBU 4140: its own 6802+6846 running the dumped KBC firmware,
	// bit-banging the 1200 baud line the ACIA terminates on the DTC side.
	// The keyboard link is a SINGLE shared pair (wired-AND, idle mark):
	// every station hears everything INCLUDING ITSELF.  The DTC's driver
	// depends on that: its rx interrupt handler ($1DAD) sends the data byte
	// upon hearing the echo of its own order byte.
	ALFASKOP_S41_KB(config, m_du_kbd, 0);
	m_du_kbd->txd_cb().set([this](int state)
			{
				m_du_kbd_line_kbd = state != 0;
				int const line = (m_du_kbd_line_kbd && m_du_kbd_line_acia) ? 1 : 0;
				m_du_kbd_acia->write_rxd(line);
				m_du_kbd->rxd_w(line);
			});
	m_du_kbd_acia->txd_handler().set([this](int state)
			{
				m_du_kbd_line_acia = state != 0;
				int const line = (m_du_kbd_line_kbd && m_du_kbd_line_acia) ? 1 : 0;
				// NOTE: delaying the ACIA's own echo by one bit time was
				// tried to force inter-character gaps and made things WORSE
				// (21 resets, q31): the DTC's handler chain is sensitive to
				// the exact echo timing.  See NOTES M10.
				m_du_kbd_acia->write_rxd(line);
				m_du_kbd->rxd_w(line);
			});
	// ACIA bit clock disabled for now: a free-running 19200 Hz timer forces a
	// 26 us scheduling quantum on the whole machine and perturbs the FD's
	// disk DMA (multi-sector reads lose bytes and time out).  Re-enable when
	// the DTC software actually drives the keyboard link.
	// clock_device &kbd_clock(CLOCK(config, "du_kbd_acia_clock", 1200 * 16));
	// kbd_clock.signal_handler().set([this](int state)
	//		{ m_du_kbd_acia->write_txc(state); m_du_kbd_acia->write_rxc(state); });

	// video: MC6845 on the DTC board, 80 columns, chargen from the CHGB
	MC6845(config, m_du_crtc, XTAL(19'170'000) / 9);
	m_du_crtc->set_screen(m_du_screen);
	m_du_crtc->set_show_border_area(false);
	m_du_crtc->set_char_width(9);
	m_du_crtc->set_update_row_callback(FUNC(alfaskop4120_state::du_crtc_update_row));
	SCREEN(config, m_du_screen, SCREEN_TYPE_RASTER);
	// 100 character positions of 9 dots = 900 dots per sweep, 426 sweeps per
	// frame: 19.17 MHz / (900 * 426) = 50.000 Hz exactly.  techtext EE360-810C
	// p.3 and osref ch.9 INITAB (all six screen formats give 426 sweeps).
	// Visible: 80 * 9 = 720 by 25 lines * 16 sweeps = 400.
	m_du_screen->set_raw(19'170'000, 900, 0, 720, 426, 0, 400);
	m_du_screen->set_screen_update(m_du_crtc, FUNC(mc6845_device::screen_update));

	// TIA board: ADLC (I7) + DMAC, channel 0 = tx, channel 1 = rx
	MC6854(config, m_du_adlc, XTAL(19'170'000) / 18);
	// DESARMADO: m_du_adlc->set_wire_rate(attotime::from_nsec(26667));  // see the note on m_adlc
	m_du_adlc->out_irq_cb().set(FUNC(alfaskop4120_state::du_irq_w<7>));
	m_du_adlc->out_rdsr_cb().set([this](int state) { m_du_dma->dreq_w<1>(state); });
	m_du_adlc->out_tdsr_cb().set([this](int state) { m_du_dma->dreq_w<0>(state); });
	m_du_adlc->out_rts_cb().set([this](int state) { if (state) m_du_adlc->set_cts(0); });
	m_du_adlc->set_out_frame_callback(FUNC(alfaskop4120_state::ss3_frame_from_du));

	MC6844(config, m_du_dma, XTAL(19'170'000) / 18);
	m_du_dma->out_drq1_callback().set([this](bool state) { m_du_dma->dgrnt_w(state); });
	m_du_dma->in_memr_callback().set([this](offs_t offset) { return m_ducpu->space(AS_PROGRAM).read_byte(offset); });
	m_du_dma->out_memw_callback().set([this](offs_t offset, uint8_t data) { LOGDMA("DU DMA -> %04X: %02X\n", offset, data); m_ducpu->space(AS_PROGRAM).write_byte(offset, data); });
	m_du_dma->in_ior_callback<1>().set([this]() -> uint8_t
			{
				uint8_t const data = m_du_adlc->dma_r();
				bool const fv = BIT(m_du_adlc->read(1), 1);
				if (!fv || !m_du_rx_tail)
				{
					m_du_rx_tail = fv;
					// NOT paced.  Timing the mc6854's stall detector instead of
					// counting offers removed the false overruns at the 64-byte
					// chunk boundary (77 stalls down to 10, qZ0) but is not
					// enough on its own: with the drain metered the display's
					// firmware stops re-arming the DMA controller for the next
					// chunk, the drain dies at 65 of 3328 bytes and the queue to
					// the display then loses 3251 frames.  THAT - why the
					// re-arm never comes - is the one question left before the
					// line can run at its real rate.
					m_du_dma->dreq_w<1>(1);
				}
				else
				{
					m_du_rx_tail = false;
					m_du_dma->dreq_w<1>(0);
					m_du_adlc->set_cts(0);
				}
				return data;
			});
	m_du_dma->out_iow_callback<0>().set([this](uint8_t data)
			{
				uint16_t const remaining = (m_du_dma->read(0x02) << 8) | m_du_dma->read(0x03);
				LOGDMA("DU DMA ch0 -> ADLC %s: %02X\n", remaining == 1 ? "last-data" : "data", data);
				m_du_adlc->write(remaining == 1 ? 3 : 2, data);
			});
}

void alfaskop4101_state::alfaskop4101(machine_config &config)
{
	/* basic machine hardware */
	M6800(config, m_maincpu, XTAL(19'170'000) / 18); // Verified from service manual
	m_maincpu->set_addrmap(AS_PROGRAM, &alfaskop4101_state::mem_map);

	PIA6821(config, m_mic_pia); // Main board PIA
}

/* ROM definitions */
ROM_START( alfaskop4110 ) // Display Unit
	ROM_REGION( 0x800, "roms", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405870205201.bin", 0x0000, 0x0800, CRC(23f20f7f) SHA1(6ed008e309473ab966c6b0d42a4f87c76a7b1d6e))
	ROM_REGION( 0x800, "chargen", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405972067500.bin", 0x0000, 0x0400, CRC(fb12b549) SHA1(53783f62c5e51320a53e053fbcf8b3701d8a805f))
	ROM_LOAD( "e3405972067600.bin", 0x0400, 0x0400, CRC(c7069d65) SHA1(587efcbee036d4c0c5b936cc5d7b1f97b6fe6dba))

	ROM_REGION( 0xff, PLA1_TAG, 0 )
	ROM_LOAD( "dtc_a_e34062_0100_ic50_e3405970303601.bin", 0x00, 0xfa, CRC(16339b7a) SHA1(9b313a7526460dc9bcedfda25bece91c924f0ddc) ) // Signetics_N82S100N.bin DATAIO format
ROM_END

ROM_START( alfaskop4120 ) // Flexible Disk Unit
	// FDP board, two 2716 EPROMs.  IC43 answers at E800-EFFF and IC44 at
	// F800-FFFF; the reset vector at FFFE points at E922, inside IC43.
	ROM_REGION( 0x1000, "roms", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405870080004.bin", 0x0000, 0x0800, CRC(a8956fed) SHA1(c3c1bf160e73dda8e9477b0560989237885cbd1d)) // FDP IC43, Toshiba TMM2716
	ROM_LOAD( "e3405870150103.bin", 0x0800, 0x0800, CRC(95ee6929) SHA1(c4c8f294e5e68cff934d245a330b437cd74d2637)) // FDP IC44, Fujitsu MBM2716

	// CP 4101, second CPU of this machine until the units become bus devices
	ROM_REGION( 0x800, "cproms", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405870034703.bin", 0x0000, 0x0800, CRC(b149868d) SHA1(63272c879bd7d218ac5845332f5e562cbd0524f3)) // CPB IC47, Fujitsu MBM2716

	// DU 4110, third CPU of this machine
	ROM_REGION( 0x800, "duroms", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405870205201.bin", 0x0000, 0x0800, CRC(23f20f7f) SHA1(6ed008e309473ab966c6b0d42a4f87c76a7b1d6e)) // DTC IC52
	ROM_REGION( 0x800, "duchargen", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405972067500.bin", 0x0000, 0x0400, CRC(fb12b549) SHA1(53783f62c5e51320a53e053fbcf8b3701d8a805f))
	ROM_LOAD( "e3405972067600.bin", 0x0400, 0x0400, CRC(c7069d65) SHA1(587efcbee036d4c0c5b936cc5d7b1f97b6fe6dba))
ROM_END

ROM_START( alfaskop4101 ) // Communication Processor Unit
	// CPB board, one 2716 EPROM at F800-FFFF; reset vector points at FB46.
	ROM_REGION( 0x800, "roms", ROMREGION_ERASEFF )
	ROM_LOAD( "e3405870034703.bin", 0x0000, 0x0800, CRC(b149868d) SHA1(63272c879bd7d218ac5845332f5e562cbd0524f3)) // CPB IC47, Fujitsu MBM2716
ROM_END

} // anonymous namespace


/* Driver(S) */

// Only 4101 may exist as a driver in the end making the 4110 and 4120 as slots devices on the SS3 bus, time will tell

//    YEAR  NAME          PARENT  COMPAT  MACHINE       INPUT         CLASS               INIT        COMPANY      FULLNAME       FLAGS
COMP( 1984, alfaskop4110, 0,      0,      alfaskop4110, alfaskop4110, alfaskop4110_state, empty_init, "Ericsson",  "Alfaskop Display Unit 4110", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
COMP( 1984, alfaskop4120, 0,      0,      alfaskop4120, alfaskop4120, alfaskop4120_state, empty_init, "Ericsson",  "Alfaskop Flexible Disk Unit 4120", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
COMP( 1984, alfaskop4101, 0,      0,      alfaskop4101, alfaskop4101, alfaskop4101_state, empty_init, "Ericsson",  "Alfaskop Communication Processor 4101", MACHINE_NO_SOUND | MACHINE_NOT_WORKING)
