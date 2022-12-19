# A NABU Keyboard-to-USB Adapter

This is an adapter that lets you connect a [NABU](https://nabu.ca/) PC keyboard to any modern
computer using USB.  In addition to the keyboard, it reports the Atari-style joysticks as USB
HID gamepads (each as a single D-pad / hat switch with a single "A" button).

Most of the NABU keys are supported in the obvious way, but it's helpful to understand how the
NABU keyboard works: The keyboard, for the most part, sends ASCII codes for the keys that are
pressed.  It handles CTRL, SHIFT, and CAPS-LOCK all on its own.  It does not send separate
key-down and key-up events for the standard keys, and it implements key-repeat itself as well.
There are some exceptions, however: the arrow keys (including the **<|||** and **|||>** keys),
**YES**, **NO**, **SYM**, **PAUSE** and **TV/NABU** keys _do_ send separate key-down and key-up
events, and are unaffected by the normal keyboard modifier keys.  These so-called "special keys"
are mapped to various HID key events as described below.

The physical interface to the NABU keyboard is a DIN-6 connector carrying a power rail and an
RS422 differential transmit line.  The RS422 serial line runs at 6992 baud, with 8 data bits,
1 stop bit, and no parity.  The odd-ball bit rate is due to the way the clock is derived; the
main clock frequency in the keyboard is 3.58MHz (the NTSC colorburst frequency), and the MC6803
microcontroller inside the keyboard internally divides the clock by 4 to derive its "E" clock.
The firmware in the keyboard then configures the MC6803's on-board UART to divide E by 128 for
the baud clock.  Communication with the keyboard is unidirectional; it is not possible to send
commands to the keyboard.

The keyboard is powered by a 9V - 12V power supply provided by the NABU.  The available
documentation is somewhat unclear about the requirements of this power supply.  Measured,
the NABU supplies an unregulated 12V.  The documentation says 9V @ 300mA, but my bench
supply tells me the keyboard draws between 310mA and 320mA when supplied with 9V.  The keyboard
contains an on-board regulator to provide its own +5V power rail.  In any case, a 9V - 12V DC
power brick that can supply at least 500mA should be perfect for the job.  The adapter controls
the power to the NABU using a small power MOSFET connected between the 9V return from the
keyboard and and negative terminal of the power supply.

## How the adapter works

The adapter utilizes the built-in UART and GPIO functionality of the Raspberry Pi Pico
microcontroller.  UART0 is used as the adapter's console (115200 @ 8N1), UART1 is used
to receive data from the keyboard, and one GPIO pin is used to control the power to the
keyboard.  Another GPIO is sampled at start-up time to enable debug messages; debugging
is enabled when the jumper is installed.

When the adapter starts up, it performs all of the initialization required and then starts
up the second core on the Pico to pull data in from the keyboard.  This loop running on the
second core determines if the data is for the keyboard or one of the joysticks, and then
adds the data byte to the appropriate input queue.

Meanwhile, the primary core powers up the keyboard and enters its main loop.  This main loop
performs 4 basic tasks:
* Blinking the status LED on the Pico to indicate various states of the adapter.
* Performing a health check of the keyboard.
* Processing incoming data from the keyboard and generating the appropriate USB HID report sequences.
* Calling into the TinyUSB stack to perform device-side USB processing.

### LED task

The LED task is pretty simple and just provides a simple way of reporting the current status of the
adapter at-a-glance.  There are 4 blinking sequences:
* Fast blinking to indicate the adapter is waiting for the host to establish a USB session.
* Medium blinking to indicate the adapter is connected to the host but waiting for the keyboard to show signs of life.
* A heatbeat pattern to indicate everything is operating normally.
* A slow blinking to indicate the adapter has been suspended and the keyboard powered off.

### Keyboard health check task

The health check task (a.k.a. "deadcheck") checks to make sure that we've heard from the keyboard within
the last 5 seconds.  When idle, the keyboard sends a "ping" approximately every 3.7 seconds.  Whenever
the keyboard reader running on the second core receives a byte of data from the keyboard, it updates a
timestamp, which is checked in deadcheck routine.  If the keyboard fails to check in for two deadcheck
intervals, the adapter reboots the keyboard by cycling power.

### Data processing task

The data processing task (called _hid_task()_ in the code) pulls data enqueued by the keyboard reader thread
and performs the processing necessary to generate USB HID report sequences.  In addition to regular keystrokes,
error messages from the keyboard and joystick data are also processed here, as is remote-host-wakeup if the
host system is suspended.

Error processing is actually performed all the time, because the periodic ping from the keyboard comes in
as an error message, as does the notification of a successful power-on-reset.  Additionally, there are
internal hardware errors (bad RAM / ROM, etc.) as well as user errors (too many keys pressed) that are
reported.  Hardware errors result in rebooting the keyboard by cycling power.

Because the NABU keyboard generates only a single byte for most key presses, this task has to generate
HID report sequences to correctly report the key.  For example, if we get "A" from the keyboard, we have
to generate a sequence of 4 HID reports: _SHIFT_, _SHIFT_ + _A_, _SHIFT_, _(none)_.  These sequences are
encoded in a table indexed by NABU keyboard code.  All of the regular keys end with a _(none)_ report.

The special keys are a little different.  They send individual key-down and key-up events to the adapter,
so we are able to simply mirror that behavior to the host.  Additionally, two of the special keys have
been mapped to key modifiers: **SYM** is mapped to **Meta** (a.k.a. Windows, Command, or GUI). and the
**TV/NABU** key is mapped to **Alt** (a.k.a. Option on Macs).  The adapter tracks the state of those
keys and includes them as modifiers in keyboard HID reports.

Much to the chargin of Unix users worldwide, the NABU keyboard lacks a backslash key.  On US keyboards,
this key is also used for the "pipe" character (SHIFT + backslash).  We steal the **YES** and **NO**
keys on the NABU keyboard for these characters: **YES** for pipe, **NO** for backslash (we have to use
two different special keys since special keys are not affected by the SHIFT keys).  Luckily, because
we're using a special key for **Alt**, we can still type guillemets on a Mac by using **TV/NABU** plus
the **YES** and **NO** keys.  _«Absolument merveilleux !», a-t-il déclaré._

### TinyUSB device task

This is just a call into the TinyUSB library that performs device-side processing.  From here, various
callbacks back into the main adapter code can be made to fetch descriptors, reports, and set
suspend/resume state.
