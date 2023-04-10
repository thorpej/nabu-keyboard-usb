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

The keyboard is powered by a nominally-12V-but-actally-9V power supply provided by the NABU.
The keyboard power comes from the NABU's 12V rail, but is fed to the keyboard through a large
power resistor on the NABU's mainboard, which drops the voltage to ~9V at the keyboard's
steady-state current draw (documentation says 300mA, but my bench supply tells me 310-320mA).
The keyboard contains an on-board regulator to provide its own +5V power rail.  In any case,
a 9V DC power brick that can supply at least 500mA should be perfect for the job.  The adapter
controls the power to the NABU using a small power MOSFET connected between the 9V return from
the keyboard and and negative terminal of the power supply.

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

Ok! To summarize the special keys:
* **Up**, **Down**, **Left**, and **Right** arrow keys map to their regular US keyboard equivalents.
* **<|||** maps to **Page Up** and **|||>** maps to **Page Down** (very useful for scrolling in Terminal on macOS).
* **SYM** maps to **Meta**, a.k.a. **Windows**, **Command**, **GUI**, **⌘**, etc.
* **TV/NABU** maps to **Alt**, a.k.a. **Option**, **⌥**, etc.
* **NO** maps to **\\**, a.k.a. backslash.
* **YES** maps to **|**, a.k.a. pipe, vertical bar, etc. 

### TinyUSB device task

This is just a call into the TinyUSB library that performs device-side processing.  From here, various
callbacks back into the main adapter code can be made to fetch descriptors, reports, and set
suspend/resume state.

## The hardware

The hardware is very simple and is centered around the Raspberry Pi Pico microcontroller board,
wich provides a UART for the debug console, a UART for the keyboard interface, the USB interface
to the host, and general-purpose I/O facilities for configuration of the adapter and control of
the keyboard's power supply.

Note that the adapter board has 3 separate power rails: +5V (supplied by the USB host), +3V3 (from
the Pico's on-board regulator), and +9V (from an external DC power brick, for the keyboard).

### Keyboard input

The keyboard uses RS422 differential signalling, so a SN75176A differential bus transciever is used
to convert the keyboard signalling to a TTL-compatible signal.  The transient suppression circuitry
mimics that present in the NABU PC.

The output of the SN75176A is at 5V TTL logic levels, but the Pico's I/O interface operates at 3V3
CMOS logic levels, so a level shifter must be used.  I just picked the 74LVC245 bus transciever for
the job because I had plenty of them handy, even though it's slight overkill.

The 74LVC245's output is connected to the Rx pin of UART1 on the Pico.  That's pretty much it for
the keyboard input.

### Keyboard power supply / control

The board has a 5.5mm x 2.1mm center-positive barrel jack for power input from an external 9V power
brick.  There is a diode in-series with the positive terminal as reverse-polarity protection.
The positive terminal from this supply goes stright to the keyboard.  The power return line
from the keyboard is attached to the drain of an IRLU110PBF power MOSFET, and the MOSFET's source is
then connected to the negative terminal of the supply, which is also connected to the board's
ground plane.  The gate is normally pulled down, but can be driven high by a pin on the Pico to
complete the circuit and power on the keyboard.  The IRLU110PBF was selected for its current
rating, relatively low as far as power MOSFETs go (1V) Vgs threshold, and < 1 Ohm Rds-ON.
There's nothing particularly special about it otherwise, and this application isn't especially
demanding, so feel free to substitute your favorite MOSFET (so long as it is in a TO-251-3
package and has a GDS pin configuration).

### Pin headers

There are 2 pin headers on the board:
* J1 provides access to UART0 on the Pico, so you can see informational or debug messages that are logged by the firmware running on the Pico.
* JP1, when connected, enables those fairly verbose debugging messages.  The pin is sampled when the adapter starts up, so enabling them requires unplugging the adapter from USB and plugging it back in after connecting the jumper.

### The board layout

I am definitely an amateur when it comes to PCB design and layout, but I did try to keep the board
fairly small while still making it easy to assemble (all through-hole parts except for the tab on
the power MOSFET which you don't even _really_ have to solder if that's your preference).  The
footprints for the decoupling caps and suppression caps are fairly forgiving, so if you have 1nF and
100nF caps in your parts bin that fit the footprint, by all means use them.

You will need to put downward-facing pin headers onto your Pico and install female pin header stand-offs
onto the board.  This is because I squeezed the power MOSFET and its pulldown resistor under the Pico.
This isn't really much of a compromise in height since there's a big ol' DIN-6 socket at the back of the
board anyhow.  (The board size is largely dictated by the external connections.)

Space for sockets for the DIP ICs is provided because I like to use them.  Feel free to elide if that's
what floats your boat.

There are a couple of "special" parts on the board:
* The resistors used on the board are fairly small for through-hole resistors, selected to reduce space requirements.  Happily, they're readily available from Mouser.
* The DIN-6 socket is harder to get than it should be (plenty of panel-mount flavors, but not PCB-mount, sigh).  The one I selected was the CLIFF FC680806, available from Newark / element14 / Farnell.  The Hirschmann 930 778-200 may also work, and there's a [no-name version on Amazon](https://www.amazon.com/dp/B015IGPKPK) that looks like a copy of the CLIFF part.

I expect to make small tweaks to the board over time, but mainly just to get alignment of various
externally-facing parts Just Right(tm) in the event that some enterprising individual wants to make
a 3D-printed enclosure for it (**_hey, if you want to do this, please get in contact with me!_**).

In addition to the KiCad design files, a PDF version of the schematic and board images are also provided,
along with the Gerber files needed to have the board made by your favorite PCB fabrication house.

### So what do I need to build one of these things?

Most of the parts requires are bog-standard, available from lots of places.  The Raspberry Pi Pico,
the DIP sockets, and the male and female pin headers can all be acquired from Amazon.  If you want
nice machined DIP sockets instead of the cheap stamped kind, you can get those from
[Phoenix Enterprises](https://www.peconnectors.com/sockets-dip-ic-machined/), but they have a minimum
order.  The ceramic capacitors are basic parts, but I've provided links to specific parts that I know
will fit the board footprints.  I've done the same with the resistors.

* 1 - Raspberry Pi Pico
* 2 - 20x1 2.54mm pin header (for the Pico)
* 1 - 2x1 2.54mm pin header (optional) (for debug jumper)
* 1 - 3x1 2.54mm pin header (optional) (for debug console)
* 2 - 20x1 2.54mm female pin header (to receive the Pico - _cannot be low-profile - space beneath the Pico is required_)
* 1 - DIP-20 socket (optional)
* 1 - DIP-8 socket (optional)
* 1 - [SN74LVC245](https://www.mouser.com/ProductDetail/595-SN74LVC245ANE4) in DIP-20 package
* 1 - [SN75176A](https://www.mouser.com/ProductDetail/595-SN75176AP) in DIP-8 package
* 1 - [IRLU110PBF](https://www.mouser.com/ProductDetail/844-IRLU110PBF) or equivalent power MOSFET in TO-251-3 package with GDS pin configuration
* 2 - [100nF / 0.1µF ceramic capacitor](https://www.mouser.com/ProductDetail/581-SR215E104MAR)
* 2 - [1nF / 1000pF ceramic capacitor](https://www.mouser.com/ProductDetail/581-SR215A102JARTR1)
* 1 - [180 Ohm resistor](https://www.mouser.com/ProductDetail/603-MFR50SFTE52-180R) that fits in a DIN0207 vertical footprint.
* 1 - [10K Ohm resistor](https://www.mouser.com/ProductDetail/603-MFR50SFTE52-10K) that fits in a DIN0207 horizontal footprint.
* 1 - 1N4001 diode in the standard package.  You can actually use whatever diode floats your boat here (it's not involved in
  any switching, merely used for reverse polarity protection), so long as it has > 18V reverse voltage rating and a 500mA current
  rating.
* 1 - [PCB-mount 5.5mm x 2.1mm DC barrel jack](https://www.mouser.com/ProductDetail/163-179PH-EX)
* 1 - [PCB-mount DIN-6 jack, CLIFF FC680806 or equivalent](https://www.newark.com/cliff-electronic-components/fc680806/din-audio-video-conn-jack-6pos/dp/99AC9154).  Also available [here!](https://www.tme.com/us/en-us/details/fc680806/din-connectors/cliff/d6-fc680806/)
* 1 - [9V @ 500mA-or-better DC power brick with 5.5mm x 2.1mm barrel connector](https://www.mouser.com/ProductDetail/490-SWI12-9-N-P5)
* 1 - Micro-USB cable for connecting the adapter to your computer.

### Assembling the board

You might have your own favorite way of putting these things together.  If so, hey, you do you!
But for those of you who don't, let me recommend a few basic pointers that will help making the
going a little easier.  I'm going to assume that you already know the basics of soldering.

First, it's best to assemble the lowest-profile components first, increasing the height of the board
as you go along.  With that in mind, here is the order I would recommend:

1. R1 and Q1 (the parts that sit beneath the Pico)
2. D1
3. U1 and U3 (or their sockets)
4. C1, C2, C3, and C4
5. R2 (the vertically-mounted resistor next to C3 and C4)
6. Female pin heders for U2 (the Pico)
7. J1 and JP1 (the male pin headers)
8. J3 (the DC barrel jack)
9. J2 (the DIN-6 jack)

If you already installed the pin headers into your Pico, or bought the Pico with the pin headers
pre-installed (hopefully with the long end of the pins facing downwards), then cool!  Otherwise,
a trick that makes it easy is to place the pin headers into the Pico (with the plastic holder on
the bottom of the Pico and she short end of the pins poking through the Pico board) and then press
that as a unit into the female headers installed on the adapter board.  That will hold everything
steady while you solder the pin headers to the Pico itself.

### Programming the Pico with the NABU Keyboard to USB firmware

Ok, so you've built the board, now you need to get the firmware onto the Pico.  This is easy!  There's
a small button on the Pico board.  What you need to do is to hold that button down while you connect
the Pico to your computer using the USB cable.  When you do that, the Pico will appear on your computer
as a hard drive with a FAT file system on it named "RPI-RP2" (exactlty how this appears on your computer
is left as an exercise to the reader).  All you need to do is copy the provided **_nabu_keyboard_usb.uf2_**
file onto that file system _et viola!_ your Pico is programmed and has now attached itself as a USB HID
devies with 1 keyboard interface and 2 game pad interfaces!

Once you've programmed the Pico, you're all set.  At this point, I recommend unplugging it from the
computer before attaching the 9V power brick and NABU keyboard.  Once everything is cabled up, plug
the Pico back into your computer and enjoy typing on those sweet, sweet ALPS key switches!

I hope you have fun with your NABU and I hope you enjoy this silly little project of mine!  If you end
up playing with it, I'd love to hear from you!  You can find me on [Mastodon](https://mastodon.sdf.org/@thorpej)
and on [Twitter](https://twitter.com/thorpej) (for as long as it lasts, anyway).
