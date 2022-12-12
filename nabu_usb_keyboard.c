/*-
 * Copyright (c) 2022 Jason R. Thorpe.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * NABU USB Keyboard
 *
 * This interfaces a NABU PC keyboard to USB has a USB keyboard device
 * with the standard US layout.
 *
 * For the most part, the NABU keyboard reports keys as ASCII characters.
 * There are no key-down or key-up events except for some special keys.
 * Shift, Control, and CapsLock are not themselves reported.  The keyboard
 * itself implements auto-repeat.
 *
 * Joystick data is also reported by the NABU keyboard.
 *
 * The NABU keyboard physical layer is RS422 8N1 @ 6992 baud.
 * (Yes, it's weird, because it's derived from the 3.58MHz NTSC
 * colorburst frequency - 3.58MHz input to 6803 which internally
 * divides by 4 to generate E, and the internal UART is configured
 * to use the /128 clock divisor to get the baud clock.)
 *
 * We use UART1 on the Pico to receive data from the keyboard.  UART0
 * is used as the console port for debugging purposes.
 *
 * TODO:
 * - Report HID gamepad data for the joysticks.
 * - Map SYM to META/GUI/CMD (and track its state).
 * - Figure out something to map to ALT/OPT.
 */

/* Pico SDK headers */
#include "pico/stdlib.h"
#include "pico/printf.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/uart.h"

/* TinyUSB SDK headers */
#include "bsp/board.h"
#include "tusb.h"

/* Standard headers */
#include <string.h>

/* Local headers */

/*
 * GPIO pins 4 and 5 are used for UART1 TX and RX, respectively.
 * This maps to physical pins 6 and 7 on the DIP-40 Pico.
 */
#define	UART1_TX_PIN		4
#define	UART1_RX_PIN		5

/*
 * Circular queue between the the UART receiver and the USB sender.
 */

#define	QUEUE_SIZE		64
#define	QUEUE_MASK		(QUEUE_SIZE - 1)
#define	QUEUE_NEXT(n)		(((n) + 1) & QUEUE_MASK)
#define	QUEUE_EMPTY_P(q)	((q)->cons == (q)->prod)
#define	QUEUE_FULL_P(q)		(QUEUE_NEXT((q)->prod) == (q)->cons)

struct queue {
	mutex_t		mutex;
	unsigned int	prod;
	unsigned int	cons;
	uint8_t		data[QUEUE_SIZE];
};

static void
queue_init(struct queue *q)
{
	memset(q, 0, sizeof(*q));
	mutex_init(&q->mutex);
}

static bool
queue_add(struct queue *q, uint8_t v)
{
	bool rv = true;		/* "OK!" is the common-case. */

	mutex_enter_blocking(&q->mutex);
	if (! QUEUE_FULL_P(q)) {
		q->data[q->prod] = v;
		q->prod = QUEUE_NEXT(q->prod);
	} else {
		rv = false;
	}
	mutex_exit(&q->mutex);

	return rv;
}

static bool
queue_consume(struct queue *q, uint8_t *vp, bool advance)
{
	bool rv = false;

	mutex_enter_blocking(&q->mutex);
	if (! QUEUE_EMPTY_P(q)) {
		*vp = q->data[q->cons];
		printf("DEBUG: prod=%u cons=%u val=0x%02x\n",
		    q->prod, q->cons, *vp);
		if (advance) {
			q->cons = QUEUE_NEXT(q->cons);
		}
		rv = true;
	}
	mutex_exit(&q->mutex);

	return rv;
}

#if 0
static bool
queue_peek(struct queue *q, uint8_t *vp)
{
	return queue_consume(q, vp, false);
}
#endif

static bool
queue_get(struct queue *q, uint8_t *vp)
{
	return queue_consume(q, vp, true);
}

/*
 * Standard TinyUSB example LED blink pattern.
 */
#define	BLINK_NOT_MOUNTED	250
#define	BLINK_MOUNTED		1000
#define	BLINK_SUSPENDED		2500

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

static void
led_task(void)
{
	static uint32_t start_ms = 0;
	static bool led_state = false;

	if (board_millis() - start_ms < blink_interval_ms) {
		return;
	}

	start_ms += blink_interval_ms;

	board_led_write(led_state);
	led_state ^= true;
}

#ifdef SLOW_TWIDDLE
#define	TWIDDLE_DELAY	4
#else
#define	TWIDDLE_DELAY	0
#endif

#define	TWIDDLE_MASK	((1 << TWIDDLE_DELAY) - 1)

static void
twiddle(void)
{
	static unsigned int pos;
	static const char twiddle_chars[] = "|/-\\";

	if ((pos & TWIDDLE_MASK) == 0) {
		uart_putc(uart0, twiddle_chars[(pos >> TWIDDLE_DELAY) & 3]);
		uart_putc(uart0, '\b');
	}
	pos++;
}

static bool want_remote_wakeup = false;

/*
 * Map NABU keycodes to HID key codes.
 *
 * Keys with M_DOWN and M_UP get separate reports for the down
 * and up events, respectively.  Otherwise, we automatically generate
 * 2 reports for the down and up events, since the keyboard doesn't
 * send them.
 *
 * Unassigned entries get 0, which conveniently is HID_KEY_NONE.
 */

#define	M_CTRL		0x0100		/* KEYBOARD_MODIFIER_LEFTCTRL << 8 */
#define	M_SHIFT		0x0200		/* KEYBOARD_MODIFIER_LEFTSHIFT << 8 */
#define	M_ERR		0x0400
#define	M_DOWN		0x1000
#define	M_UP		0x2000
#define	M_JOY		0x4000
#define	M_JOYDAT	0x8000

#define	NABU_CODE_JOY0		0x80
#define	NABU_CODE_JOY1		0x81
#define	NABU_CODE_JOYDAT_FIRST	0xa0
#define	NABU_CODE_JOYDAT_LAST	0xbf

#define	NABU_KBD_BAUDRATE	6992

static const uint16_t nabu_to_hid[256] = {
/*
 * CTRL just lops off the 2 upper bits of the keycode
 * on the NABU keyboard (except for C-'<' ??), but we
 * simplify to C-a, C-c, etc.
 */
[0x00]		=	M_CTRL | M_SHIFT | HID_KEY_2,		/* C-'@' */
[0x01]		=	M_CTRL | HID_KEY_A,
[0x02]		=	M_CTRL | HID_KEY_B,
[0x03]		=	M_CTRL | HID_KEY_C,
[0x04]		=	M_CTRL | HID_KEY_D,
[0x05]		=	M_CTRL | HID_KEY_E,
[0x06]		=	M_CTRL | HID_KEY_F,
[0x07]		=	M_CTRL | HID_KEY_G,
[0x08]		=	HID_KEY_BACKSPACE,			/* Backspace */
[0x09]		=	HID_KEY_TAB,				/* Tab */
[0x0a]		=	HID_KEY_ENTER,				/* LF */
[0x0b]		=	M_CTRL | HID_KEY_K,
[0x0c]		=	M_CTRL | HID_KEY_L,
[0x0d]		=	HID_KEY_ENTER,				/* CR */
[0x0e]		=	M_CTRL | HID_KEY_N,
[0x0f]		=	M_CTRL | HID_KEY_O,
[0x10]		=	M_CTRL | HID_KEY_P,
[0x11]		=	M_CTRL | HID_KEY_Q,
[0x12]		=	M_CTRL | HID_KEY_R,
[0x13]		=	M_CTRL | HID_KEY_S,
[0x14]		=	M_CTRL | HID_KEY_T,
[0x15]		=	M_CTRL | HID_KEY_U,
[0x16]		=	M_CTRL | HID_KEY_V,
[0x17]		=	M_CTRL | HID_KEY_W,
[0x18]		=	M_CTRL | HID_KEY_X,
[0x19]		=	M_CTRL | HID_KEY_Y,
[0x1a]		=	M_CTRL | HID_KEY_Z,
[0x1b]		=	HID_KEY_ESCAPE,				/* ESC */
[0x1c]		=	M_CTRL | M_SHIFT | HID_KEY_COMMA,	/* C-'<' */
[0x1d]		=	M_CTRL | HID_KEY_BRACKET_RIGHT,
[0x1e]		=	M_CTRL | M_SHIFT | HID_KEY_6,		/* C-'^' */
[0x1f]		=	M_CTRL | M_SHIFT | HID_KEY_MINUS,	/* C-'_' */

[0x20]		=	HID_KEY_SPACE,
[0x21]		=	M_SHIFT | HID_KEY_1,			/* ! */
[0x22]		=	M_SHIFT | HID_KEY_APOSTROPHE,		/* " */
[0x23]		=	M_SHIFT | HID_KEY_3,			/* # */
[0x24]		=	M_SHIFT | HID_KEY_4,			/* $ */
[0x25]		=	M_SHIFT | HID_KEY_5,			/* % */
[0x26]		=	M_SHIFT | HID_KEY_7,			/* & */
[0x27]		=	HID_KEY_APOSTROPHE,			/* ' */
[0x28]		=	M_SHIFT | HID_KEY_9,			/* ( */
[0x29]		=	M_SHIFT | HID_KEY_0,			/* ) */
[0x2a]		=	M_SHIFT | HID_KEY_8,			/* * */
[0x2b]		=	M_SHIFT | HID_KEY_EQUAL,		/* + */
[0x2c]		=	HID_KEY_COMMA,				/* , */
[0x2d]		=	HID_KEY_MINUS,				/* - */
[0x2e]		=	HID_KEY_PERIOD,				/* . */
[0x2f]		=	HID_KEY_SLASH,				/* / */
[0x30]		=	HID_KEY_0,
[0x31]		=	HID_KEY_1,
[0x32]		=	HID_KEY_2,
[0x33]		=	HID_KEY_3,
[0x34]		=	HID_KEY_4,
[0x35]		=	HID_KEY_5,
[0x36]		=	HID_KEY_6,
[0x37]		=	HID_KEY_7,
[0x38]		=	HID_KEY_8,
[0x39]		=	HID_KEY_9,
[0x3a]		=	M_SHIFT | HID_KEY_SEMICOLON,		/* : */
[0x3b]		=	HID_KEY_SEMICOLON,
[0x3c]		=	M_SHIFT | HID_KEY_COMMA,		/* < */
[0x3d]		=	HID_KEY_EQUAL,
[0x3e]		=	M_SHIFT | HID_KEY_PERIOD,		/* > */
[0x3f]		=	M_SHIFT | HID_KEY_SLASH,		/* ? */

[0x40]		=	M_SHIFT | HID_KEY_2,			/* @ */
[0x41]		=	M_SHIFT | HID_KEY_A,
[0x42]		=	M_SHIFT | HID_KEY_B,
[0x43]		=	M_SHIFT | HID_KEY_C,
[0x44]		=	M_SHIFT | HID_KEY_D,
[0x45]		=	M_SHIFT | HID_KEY_E,
[0x46]		=	M_SHIFT | HID_KEY_F,
[0x47]		=	M_SHIFT | HID_KEY_G,
[0x48]		=	M_SHIFT | HID_KEY_H,
[0x49]		=	M_SHIFT | HID_KEY_I,
[0x4a]		=	M_SHIFT | HID_KEY_J,
[0x4b]		=	M_SHIFT | HID_KEY_K,
[0x4c]		=	M_SHIFT | HID_KEY_L,
[0x4d]		=	M_SHIFT | HID_KEY_M,
[0x4e]		=	M_SHIFT | HID_KEY_N,
[0x4f]		=	M_SHIFT | HID_KEY_O,
[0x50]		=	M_SHIFT | HID_KEY_P,
[0x51]		=	M_SHIFT | HID_KEY_Q,
[0x52]		=	M_SHIFT | HID_KEY_R,
[0x53]		=	M_SHIFT | HID_KEY_S,
[0x54]		=	M_SHIFT | HID_KEY_T,
[0x55]		=	M_SHIFT | HID_KEY_U,
[0x56]		=	M_SHIFT | HID_KEY_V,
[0x57]		=	M_SHIFT | HID_KEY_W,
[0x58]		=	M_SHIFT | HID_KEY_X,
[0x59]		=	M_SHIFT | HID_KEY_Y,
[0x5a]		=	M_SHIFT | HID_KEY_Z,
[0x5b]		=	HID_KEY_BRACKET_LEFT,			/* [ */
/* 0x5c */
[0x5d]		=	HID_KEY_BRACKET_RIGHT,			/* ] */
[0x5e]		=	M_SHIFT | HID_KEY_6,			/* ^ */
[0x5f]		=	M_SHIFT | HID_KEY_MINUS,		/* _ */

/* 0x60 */
[0x61]		=	HID_KEY_A,
[0x62]		=	HID_KEY_B,
[0x63]		=	HID_KEY_C,
[0x64]		=	HID_KEY_D,
[0x65]		=	HID_KEY_E,
[0x66]		=	HID_KEY_F,
[0x67]		=	HID_KEY_G,
[0x68]		=	HID_KEY_H,
[0x69]		=	HID_KEY_I,
[0x6a]		=	HID_KEY_J,
[0x6b]		=	HID_KEY_K,
[0x6c]		=	HID_KEY_L,
[0x6d]		=	HID_KEY_M,
[0x6e]		=	HID_KEY_N,
[0x6f]		=	HID_KEY_O,
[0x70]		=	HID_KEY_P,
[0x71]		=	HID_KEY_Q,
[0x72]		=	HID_KEY_R,
[0x73]		=	HID_KEY_S,
[0x74]		=	HID_KEY_T,
[0x75]		=	HID_KEY_U,
[0x76]		=	HID_KEY_V,
[0x77]		=	HID_KEY_W,
[0x78]		=	HID_KEY_X,
[0x79]		=	HID_KEY_Y,
[0x7a]		=	HID_KEY_Z,
[0x7b]		=	M_SHIFT | HID_KEY_BRACKET_LEFT,		/* { */
/* 0x7c */
[0x7d]		=	M_SHIFT | HID_KEY_BRACKET_RIGHT,	/* } */
/* 0x7e */
[0x7f]		=	HID_KEY_BACKSPACE,			/* DEL */

/* 0x80 - 0x8f */
[0x90]		=	M_ERR | 1,				/* > 1 key */
[0x91]		=	M_ERR | 2,				/* bad RAM */
[0x92]		=	M_ERR | 3,				/* bad ROM */
[0x93]		=	M_ERR | 4,				/* crash? */
[0x94]		=	M_ERR | 5,				/* heartbeat */
[0x95]		=	M_ERR | 6,				/* reboot */
/* 0x96 - 0x9f */

/* 0xa0 - 0xbf */

/* 0xc0 - 0xdf */

[0xe0]		=	M_DOWN | HID_KEY_ARROW_RIGHT,
[0xe1]		=	M_DOWN | HID_KEY_ARROW_LEFT,
[0xe2]		=	M_DOWN | HID_KEY_ARROW_DOWN,
[0xe3]		=	M_DOWN | HID_KEY_ARROW_UP,
[0xe4]		=	M_DOWN | HID_KEY_PAGE_DOWN,		/* |||> */
[0xe5]		=	M_DOWN | HID_KEY_PAGE_UP,		/* <||| */
[0xe6]		=	0 /* XXX */,				/* NO */
[0xe7]		=	0 /* XXX */,				/* YES */
[0xe8]		=	0 /* XXX */,				/* SYM */
[0xe9]		=	M_DOWN | HID_KEY_PAUSE,			/* PAUSE */
[0xea]		=	0 /* XXX */,				/* TV/NABU */
/* 0xeb - 0xef */
[0xf0]		=	M_UP | HID_KEY_ARROW_RIGHT,
[0xf1]		=	M_UP | HID_KEY_ARROW_LEFT,
[0xf2]		=	M_UP | HID_KEY_ARROW_DOWN,
[0xf3]		=	M_UP | HID_KEY_ARROW_UP,
[0xf4]		=	M_UP | HID_KEY_PAGE_DOWN,		/* |||> */
[0xf5]		=	M_UP | HID_KEY_PAGE_UP,			/* <||| */
[0xf6]		=	0 /* XXX */,				/* NO */
[0xf7]		=	0 /* XXX */,				/* YES */
[0xf8]		=	0 /* XXX */,				/* SYM */
[0xf9]		=	M_UP | HID_KEY_PAUSE,			/* PAUSE */
[0xfa]		=	0 /* XXX */,				/* TV/NABU */
/* 0xfb - 0xff */
};

/*
 * Joystick data packets have the format:
 *
 *	1 0 1 F U R D L
 *	      i p i o e
 *	      r   g w f
 *	      e   h n t
 *	          t
 */
#define	JOY_LEFT	(1U << 0)
#define	JOY_DOWN	(1U << 1)
#define	JOY_RIGHT	(1U << 2)
#define	JOY_UP		(1U << 3)
#define	JOY_FIRE	(1U << 4)
#define	JOY_DIR_MASK	(JOY_LEFT | JOY_DOWN | JOY_RIGHT | JOY_UP)

/*
 * GAMEPAD_HAT_CENTERED is, conveniently, 0.  We'll also use that
 * for physically impossible combinations on a real joystick / dpad.
 */
static const uint8_t joy_to_dpad[JOY_DIR_MASK + 1] = {
[JOY_UP]		=	GAMEPAD_HAT_UP,
[JOY_UP | JOY_RIGHT]	=	GAMEPAD_HAT_UP_RIGHT,
[JOY_RIGHT]		=	GAMEPAD_HAT_RIGHT,
[JOY_DOWN | JOY_RIGHT]	=	GAMEPAD_HAT_DOWN_RIGHT,
[JOY_DOWN]		=	GAMEPAD_HAT_DOWN,
[JOY_DOWN | JOY_LEFT]	=	GAMEPAD_HAT_DOWN_LEFT,
[JOY_LEFT]		=	GAMEPAD_HAT_LEFT,
[JOY_UP | JOY_LEFT]	=	GAMEPAD_HAT_UP_LEFT,
};

/*
 * We keep 2 joystick contexts so we can report "simultaneous" movements
 * on both sticks more accurately, but we still need to have a global for
 * the "instance" we're processing while the data is coming in.
 */
static struct joy_context {
	struct queue queue;
} joy_context[2];

static void
joy_init(int which)
{
	queue_init(&joy_context[which].queue);
}

static void
send_joy_report(int which, uint8_t data)
{
#if 0
	uint8_t dpad = joy_to_dpad[data & JOY_DIR_MASK];
	uint8_t buttons = (data & JOY_FIRE) ? GAMEPAD_BUTTON_A : 0;

	hid_gamepad_report_t report = {
		.hat		=	dpad,
		.buttons	=	buttons,
	};

	tud_hid_n_report(ITF_NUM_JOY0 + which, 0, &report, sizeof(report));
#endif
}

static struct {
	struct queue queue;
	bool want_keyup;
} kbd_context;

static void
kbd_init(void)
{
	queue_init(&kbd_context.queue);
	kbd_context.want_keyup = false;
}

static inline uint8_t
keymod_to_hid(uint16_t code)
{
	return (code >> 8) &
	    (KEYBOARD_MODIFIER_LEFTCTRL | KEYBOARD_MODIFIER_LEFTSHIFT);
}

static void
send_kbd_report(uint16_t code)
{
	uint8_t keymod = keymod_to_hid(code);
	uint8_t keycode = (uint8_t)code;

	hid_keyboard_report_t report = {
		.modifier	=	keymod,
		.keycode	=	{ [0] = keycode },
	};

	tud_hid_n_report(ITF_NUM_KBD, 0, &report, sizeof(report));
}

#define	REPORT_INTERVAL_MS	10

static void
hid_task(void)
{
	uint8_t c;

	/* This is good for ~139 years of uptime. */
	static uint32_t start_ms;

	if (board_millis() - start_ms < REPORT_INTERVAL_MS) {
		return;
	}

	start_ms += REPORT_INTERVAL_MS;

	/*
	 * Quick unlocked queue-empty checks to see if there's
	 * work to do.
	 */
	bool have_work = false;
	if (kbd_context.want_keyup) {
		printf("DEBUG: %s: want_keyup, have_work -> true\n", __func__);
		have_work = true;
	}
	if (! QUEUE_EMPTY_P(&kbd_context.queue)) {
		printf("DEBUG: %s: kbd queue, have_work -> true\n", __func__);
		have_work = true;
	}
	if (! QUEUE_EMPTY_P(&joy_context[0].queue)) {
		printf("DEBUG: %s: joy0 queue, have_work -> true\n", __func__);
		have_work = true;
	}
	if (! QUEUE_EMPTY_P(&joy_context[1].queue)) {
		printf("DEBUG: %s: joy1 queue, have_work -> true\n", __func__);
		have_work = true;
	}

	if (! have_work) {
		/* No data to send. */
		/* XXX Do we need to send reports always? */
		return;
	}

	/*
	 * We have at least one report to send.  If we're suspended,
	 * wake up the host.  We'll send the report the next time
	 * around.
	 */
	if (tud_suspended()) {
		if (want_remote_wakeup) {
			tud_remote_wakeup();
			want_remote_wakeup = false;
		}
		return;
	}

	if (tud_hid_n_ready(ITF_NUM_KBD)) {
		if (kbd_context.want_keyup) {
			kbd_context.want_keyup = false;
			send_kbd_report(HID_KEY_NONE);
		} else if (queue_get(&kbd_context.queue, &c)) {
			uint16_t code = nabu_to_hid[c];

			printf("Got '%c' (code=0x%04x)\n", c, code);
			if (code != 0) {
				kbd_context.want_keyup =
				    (code & (M_UP | M_DOWN)) == 0;
				printf("DEBUG: %s: want_keyup = %d\n",
				    __func__, kbd_context.want_keyup);
				send_kbd_report(code);
			}
		}
	} else {
		twiddle();
	}

#if 0
	/* Now do the joysticks. */
	for (int i = 0; i < 2; i++) {
		if (tud_hid_n_ready(ITF_NUM_JOY0 + i)) {
			if (queue_get(&joy_context[which].queue, &c)) {
				send_joy_report(which, c);
			}
		}
	}
#endif
}

/*
 * Invoked when the device is "mounted".
 */
void
tud_mount_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

/*
 * Invoked when the device is "unmounted".
 */
void
tud_umount_cb(void)
{
	blink_interval_ms = BLINK_NOT_MOUNTED;
}

/*
 * Invoked when the USB bus is suspended.
 *
 * remote_wakeup_en indicates if the host allows us to perform a
 * remote wakeup.
 *
 * Within 7ms, we must drop our current draw to less than 2.5mA from
 * the bus.  Not a problem, since we require an external power source
 * for the keyboard anyway.
 */
void
tud_suspend_cb(bool remote_wakeup_en)
{
	want_remote_wakeup = remote_wakeup_en;
	blink_interval_ms = BLINK_SUSPENDED;
}

/*
 * Invoked when the USB bus is resumed.
 */
void
tud_resume_cb(void)
{
	blink_interval_ms = BLINK_MOUNTED;
}

/*
 * Invoked when received GET_REPORT control request.
 * Application must fill buffer report's content and return its length.
 * Return zero will cause the stack to STALL request.
 */
uint16_t
tud_hid_get_report_cb(uint8_t itf, uint8_t report_id,
    hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
	// TODO not Implemented
	(void) itf;
	(void) report_id; 
	(void) report_type;
	(void) buffer;
	(void) reqlen;

	return 0;
}

/*
 * Invoked when received SET_REPORT control request or
 * received data on OUT endpoint ( Report ID = 0, Type = 0 )
 */
void
tud_hid_set_report_cb(uint8_t itf, uint8_t report_id,
    hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
	// TODO set LED based on CAPLOCK, NUMLOCK etc...
	(void) itf;
	(void) report_id;
	(void) report_type;
	(void) buffer;
	(void) bufsize;
}

/*
 * This function runs on Core 1, sucks down bytes from the UART
 * in a tight loop, and pushes them into the appropriate queue.
 */
static void
nabu_keyboard_reader(void)
{
	int joy_instance = -1;
	uint8_t c;

	static int start_ms = 0;
	static int state = 0;
	static const char str[] = "Oink!\n";
	int idx;

	for (;;) {
#if 0
		c = uart_getc(uart1);
#else
		if (board_millis() - start_ms < 1000) {
			continue;
		}

		start_ms += 1000;
		if (state < 5) {
			state++;
			continue;
		}

		idx = state - 5;
		c = str[idx];
		if (c != '\0') {
			printf("Injecting '%c'\n", c);
			queue_add(&kbd_context.queue, c);
			state++;
			continue;
		} else {
			continue;
		}
#endif

		/* Check for a joystick instance. */
		if (c == NABU_CODE_JOY0 || c == NABU_CODE_JOY1) {
			joy_instance = c & 1;
			/* We expect a joystick data byte next. */
			continue;
		}

		/* Check for joystick data. */
		if (c >= NABU_CODE_JOYDAT_FIRST &&
		    c <= NABU_CODE_JOYDAT_LAST) {
			if (joy_instance < 0) {
				/* Unexpected; discard data. */
				continue;
			}
			queue_add(&joy_context[joy_instance].queue, c);
			joy_instance = -1;
			continue;
		}

		if (joy_instance >= 0) {
			/* Unexpected; reset state. */
			joy_instance = -1;
		}

		/*
		 * The rest is ostensibly keyboard data, but don't
		 * bother to enqueue it if there's no action that
		 * will be taken.
		 */
		if (nabu_to_hid[c] != 0) {
			queue_add(&kbd_context.queue, c);
		}
	}
}

#define	VERSION_MAJOR	0
#define	VERSION_MINOR	1

int
main(void)
{
	uint actual_baud;

	/* TinyUSB SDK board init - initializes LED and console UART (0). */
	board_init();

	printf("NABU Keyboard -> USB HID Adapter %d.%d\n",
	    VERSION_MAJOR, VERSION_MINOR);
	printf("Copyright (c) 2022 Jason R. Thorpe\n\n");

	printf("Initializing UART1 (NABU keyboard).\n");
	gpio_set_function(UART1_TX_PIN, GPIO_FUNC_UART);
	gpio_set_function(UART1_RX_PIN, GPIO_FUNC_UART);
	actual_baud = uart_init(uart1, NABU_KBD_BAUDRATE);
	if (actual_baud != NABU_KBD_BAUDRATE) {
		printf("WARNING: UART1 actual baud rate %u != %u\n",
		    actual_baud, NABU_KBD_BAUDRATE);
	}
	uart_set_fifo_enabled(uart1, true);
	uart_set_format(uart1, 8/*data*/, 1/*stop*/, UART_PARITY_NONE);

	printf("Initializing USB stack.\n");
	tusb_init();

	printf("Initializing keyboard.\n");
	kbd_init();

	printf("Initializing joysticks.\n");
	joy_init(0);
	joy_init(1);

	printf("Resetting Core 1.\n");
	multicore_reset_core1();

	printf("Starting UART reader on Core 1.\n");
	multicore_launch_core1(nabu_keyboard_reader);

	printf("Entering main loop!\n");
	for (;;) {
		tud_task();		/* TinyUSB device task */
		led_task();		/* heartbeat LED */
		hid_task();		/* HID processing */
	}
}
