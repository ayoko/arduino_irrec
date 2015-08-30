/*
 * Learning Infrared (or IR) Remote Controller for Arduino
 * www.flogics.com
 *
 * Copyright (c) 2010-2015, Atsushi Yokoyama, Firmlogics
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * PORTS_IR_LED is, configuration of PORTD GPIO to drive multiple IR LEDs
 * (bit7 = port 7, ..., bit0 = port 0)
 */
#define PORTS_IR_LED	0x7c	/* 01111100 on GPIO PORTD */
#define PORT_RED_LED	13	/* standard Arduino diagnostic LED */
#define PORT_IR_SENSOR	7

#define VERSION		"20150824"
#define TIMEOUT_IR	10000	/* IR pattern timeout in us (microseconds) */
#define ERR_MARGIN	1.3	/* to allow variation of received LED on/off */

#define LEN_TERM_CMD	60	/* max length (byte) of terminal input */
#define LEN_IR_SEQ	15	/* max length (byte) of command bytes */
#define LEN_IR_CHANGES	(LEN_IR_SEQ * 8 * 2 + 5)    /* max IR state changes */

#define diag(s) Serial.println(s)

static int led_patterns[LEN_IR_CHANGES];    // Store received LED state changes
static char term_cmd[LEN_TERM_CMD];	// user interface command string
static int len_ir_changes;		// number of IR state changes
static int pat_len_short;		// short pattern length (us)
static int pat_len_long;		// long pattern length (us)
static int pat_len_leader1;		// leader-1 length (us)
static int pat_len_leader0;		// leader-0 length (us)
static char virtual_leds;		// virtual LEDs VRAM
static char blink_status = 0;		// blinking status (port bit patterns)


/*
 * sense_ir() returns 1 when detecting IR, otherwise returns 0
 */
static int
sense_ir(void) {
	/*
	 * Negating due to IRM-3683N3 output polarity
	 */
	return digitalRead(PORT_IR_SENSOR) == 0;
}

/*
 * get_cmd() retrieves a new command from serial terminal.
 * len is allowable size of *s and it includes entry for the tail '\0'.
 */
static int
get_cmd(char *s, int len)
{
	int c;
	int ct;

	s[len - 1] = '\0';

	ct = 0;
	while (ct < len - 1) {
		while ((c = Serial.read()) == -1)
			delay(1);
		Serial.print((char) c);

		/*
		 * Even a terminal doesn't allow \n or \r, you can terminate
		 * a command by '.'
		 */
		if (c == '.' || c == '\n' || c == '\r')
			break;
		s[ct ++] = c;
	}

	s[ct] = '\0';
	return ct;
}

/*
 * Hold LED status in b (1 = on, 0 = off) for len (microseconds), and return
 * after the busy-wait
 */
static void
hold_led_len(int b, int len)
{
	unsigned long start;
	unsigned long time;

	digitalWrite(PORT_RED_LED, b);
	virtual_leds = b ? PORTS_IR_LED : 0;
	start = micros();
	while ((time = micros()) - start < (unsigned long) len) {
		;
	}
}

/*
 * Send a bit (1 or 0) by IR with already specified modulation pattern
 * pat_len_*
 */
static void
send_led_bit(int b)
{
	hold_led_len(1, pat_len_short);

	if (b == 0) {
		hold_led_len(0, pat_len_short);
	} else {
		hold_led_len(0, pat_len_long);
	}
}

/*
 * Transmit an IR message *cmdpat (its length is len) with already specified
 * modulation pattern pat_len_*
 */
static void
transmit_ir_msg(const unsigned char *cmdpat, int len)
{
	int i;
	int j;

	/*
	 * Transmit a leader
	 */
	hold_led_len(1, pat_len_leader1);
	hold_led_len(0, pat_len_leader0);

	/*
	 * Transmit a main message
	 */
	for (i = 0; i < len; i ++) {
		for (j = 7; j >= 0; j --) {
			send_led_bit((cmdpat[i] & (1 << j)) != 0);
		}
	}

	/*
	 * Transmit a trailer
	 */
	hold_led_len(1, pat_len_short);
	hold_led_len(0, pat_len_short);
}

/*
 * Wait until we detects IR light-on, but abort whenever a key-press received
 * on Serial.  In the former case, returns 1.  Otherwise returns 0
 */
static int
detect_raise(void)
{
	diag("Press a remote controller button toward IR sensor");
	diag("(any key to abort)");

	/*
	 * Wait for IR detection (or sense_ir() returns 1)
	 * (Abort when a key-press detected)
	 */
	while (sense_ir() == 0) {
		if (Serial.available()) {
			diag("Aborted");
			while (Serial.read() != -1)
				;
			return 0;
		}
	}

	return 1;
}

/*
 * Record IR pattern received by IR sensor into tab[], and return length of
 * the pattern.  It aborts whenever IR state (on/off) does not change for
 * TIMEOUT_IR (microseconds).
 *
 * Note: It assumes IR detecting state is 'on' when the function is called
 */
static int
record_ir(int *tab, int maxlen)
{
	unsigned long time;
	int state = 1;
	int tabpos = 0;

	for (;;) {
		time = micros();
		while (sense_ir() == state) {
			if (micros() - time >= TIMEOUT_IR) {
				return tabpos;
			}
		}

		tab[tabpos ++] = micros() - time;
		if (tabpos >= maxlen) {
			diag("record_ir(): too long IR pattern");
			return tabpos;
		}

		state = ! state;
	}

	return -1;	// doesn't reach here
}

/*
 * Analyze received IR modulation parameters and store the result
 * into pat_len_*
 *
 * XXX: It does not recognize Sony format
 */
static int
analyze_pat(const int *tab, int len)
{
	/*
	 * These 4 variables may become large and may not fit into 'int'
	 */
	long len_long0;
	long len_short0;
	long len_short1;
	long sum;

	int calib;
	int ct;
	int ct_short0;
	int ct_long0;
	int i;
	int len_leader0;
	int len_leader1;
	int max;
	int thresh;
	int min;

	/*
	 * Temporary assign leaders length
	 * They will be adjusted later
	 */
	len_leader1 = tab[0];
	len_leader0 = tab[1];

	/*
	 * Calculate average length of the short-on (or short1)
	 */
	sum = 0;
	ct = 0;
	for (i = 2; i < len; i += 2) {
		sum += tab[i];
		ct ++;
	}
	len_short1 = sum / ct;

	/*
	 * Determine judgment threshold between the short-off and long-off
	 * XXX: This algorithm can be improved but working in anyway
	 */
	min = 32767;
	max = 0;
	for (i = 3; i < len; i += 2) {
		if (tab[i] > max)
			max = tab[i];
		if (tab[i] < min)
			min = tab[i];
	}
	thresh = (min + max) / 2;

	/*
	 * Calculate average length of short-off (or short0) and long-off
	 */
	len_short0 = 0;
	len_long0 = 0;
	ct_short0 = 0;
	ct_long0 = 0;
	for (i = 3; i < len; i += 2) {
		if (tab[i] < thresh) {
			len_short0 += tab[i];
			ct_short0 ++;
		}
		if (tab[i] >= thresh) {
			len_long0 += tab[i];
			ct_long0 ++;
		}
	}
	len_short0 /= ct_short0;
	len_long0 /= ct_long0;

	/*
	 * Calculate a calibration length to adjust received patterns.
	 * 'calib' becomes positive value if IR on-time is longer than off-time
	 */
	calib = (len_short1 + len_short0) / 2 - len_short0;

	/*
	 * Finally calibrate the pattern lengths
	 */
	len_leader1 -= calib;
	len_leader0 += calib;
	len_short1 -= calib;
	len_short0 += calib;
	len_long0 += calib;

	/*
	 * Hereafter, len_short1 must be nearly equal to the len_short0
	 */

	/*
	 * Finally show the obtained modulation parameters
	 */
	Serial.print("P ");
	Serial.print(len_leader1, DEC);
	Serial.print(" ");
	Serial.print(len_leader0, DEC);
	Serial.print(" ");
	Serial.print(len_short1, DEC);		// nearly equal to len_short0
	Serial.print(" ");
	Serial.println(len_long0, DEC);

	/*
	 * Remember the parameters for future use
	 */
	pat_len_leader1 = len_leader1;
	pat_len_leader0 = len_leader0;
	pat_len_short = (int) len_short1;	// nearly equal to len_short0
	pat_len_long = (int) len_long0;

	return 0;
}

/*
 * Decode received IR patterns tab[] and show the command message as a result
 */
static int
decode_pat(const int *tab, int len)
{
	char hex[3];
	int b;
	int ct_bit;
	int expected_len;
	int i;
	int thresh;
	unsigned char byte;

	thresh = (pat_len_short + pat_len_long) / 2;

	Serial.print("T ");

	ct_bit = 0;
	byte = 0;
	for (i = 2; i < len; i ++) {
		if (i % 2 == 0) {	// in case of Short-on
			if (tab[i] < pat_len_short / ERR_MARGIN ||
			    tab[i] > pat_len_short * ERR_MARGIN)
				diag("Length error (on)");
		} else {		// in case of Short-off or Long-off
			if (tab[i] > thresh)
				b = 1;
			else
				b = 0;
			expected_len = b ? pat_len_long : pat_len_short;
			if (tab[i] < expected_len / ERR_MARGIN ||
			    tab[i] > expected_len * ERR_MARGIN)
				diag("Length error (off)");
			byte = (byte << 1) | b;
			ct_bit ++;
		}

		/*
		 * Show a result in hexadecimal for each 8-bit sequence
		 */
		if (ct_bit >= 8) {
			sprintf(hex, "%02X", byte);
			Serial.print(hex);
			Serial.print(" ");
			ct_bit = 0;
			byte = 0;
		}
	}
	Serial.println("");

	if (ct_bit != 0) {
		diag("Length error (not multiple of 8 bits)");
		return -1;
	}

	return 0;
}

/*
 * Output debug information to Serial
 */
static int
debug(void)
{
	int i;

	Serial.print("len = ");
	Serial.println(len_ir_changes, DEC);

	for (i = 0; i < len_ir_changes; i ++) {
		Serial.print(i, DEC);
		Serial.print(" ");
		Serial.println(led_patterns[i], DEC);
	}

	return 0;
}

/*
 * Parse an user interface command by communicating on Serial
 */
static int
parse_cmd(void)
{
	unsigned char cmdpat[LEN_IR_SEQ];
	int ct;
	int i;
	char *next_pt;
	char *pt;
	char op;			// Op-code (or command)
	unsigned char pat;

	get_cmd(term_cmd, LEN_TERM_CMD);
	Serial.println("");

	if (strlen(term_cmd) < 1)
		return -1;		// No command found

	/*
	 * Convert the command string to upper cases
	 */
	for (i = 0; i < (int) strlen(term_cmd); i ++)
		term_cmd[i] = toupper(term_cmd[i]);

	op = term_cmd[0];
	pt = &term_cmd[1];
	switch (op) {
	case 'P':
		/*
		 * Specify a modulation pattern by command
		 */
		pat_len_leader1 = (int) strtol(pt, &pt, 0);
		pat_len_leader0 = (int) strtol(pt, &pt, 0);
		pat_len_short = (int) strtol(pt, &pt, 0);
		pat_len_long = (int) strtol(pt, &pt, 0);
		Serial.println(pat_len_leader1, DEC);
		Serial.println(pat_len_leader0, DEC);
		Serial.println(pat_len_short, DEC);
		Serial.println(pat_len_long, DEC);

		if (pat_len_leader1 <= 0 ||
		    pat_len_leader0 <= 0 ||
		    pat_len_short <= 0 ||
		    pat_len_long <= 0) {
			diag("Illegal modulation pattern specified.");
			return -1;
		}
		break;
	case 'T':
		/*
		 * Transmit an IR message sequence with already specified
		 * modulation pattern
		 */
		ct = 0;
		next_pt = NULL;
		while (ct < LEN_IR_SEQ) {
			pat = (unsigned char) strtol(pt, &next_pt, 16);
			if (pt == next_pt)
				break;
			cmdpat[ct] = pat;
			pt = next_pt;
			ct ++;
		}

		if (ct == 0) {
			diag("At least one message byte is required.");
			break;
		}

		transmit_ir_msg(cmdpat, ct);
		break;
	case 'R':
		/*
		 * Receive an IR pattern by an IR sensor and show the decoded
		 * results
		 */
		if (detect_raise() == 0)
			break;
		len_ir_changes = record_ir(led_patterns, LEN_IR_CHANGES);
		analyze_pat(led_patterns, len_ir_changes);
		decode_pat(led_patterns, len_ir_changes);
		break;
	case 'D':
		/*
		 * Show debug information
		 */
		debug();
		break;
	case 'H':
		/*
		 * Command help
		 */
		diag("Learning Infrared Remote Controller command help:");
		diag("  P pat_len_leader1 pat_len_leader0 pat_len_short "
			"pat_len_long");
		diag("    Specify modulation pattern (len in micro seconds).");
		diag("  T hh hh hh ...");
		diag("    Transmit command message.  'hh' is in hexadecimal.");
		diag("  R");
		diag("    Decode command message.");
		diag("  D");
		diag("    Show debug information.");
		break;
	default:
		diag("Illegal command.  Try H for help.");
		break;
	}

	return 0;
}

/*
 * Interrupt service routine for a periodical timer.  It turns on/off Infrared
 * LED to modulate virtual_leds with sub-carrier frequency
 */
ISR(TIMER2_COMPA_vect)
{
	PORTD = blink_status & virtual_leds;

	/*
	 * Inverse all bits of blink_status
	 */
	blink_status = ~blink_status;
}

/*
 * Arduino setup() code
 */
void
setup()
{
	int i;

	/*
	 * Configure modulation pattern as AEHA defines (a common format)
	 */
	pat_len_short = 435;
	pat_len_long = pat_len_short * 3;
	pat_len_leader1 = pat_len_short * 8;
	pat_len_leader0 = pat_len_short * 4;
	len_ir_changes = 0;

	/*
	 * Configure ATmega internal timer to generate sub-carrier on/off
	 * interrupt
	 */
        TCCR2A = 2;             // WGM2[1:0] = 2 (for CTC mode)
        TCCR2B = 1;             // WGM22 = 0 (for CTC mode)
                                // CS2[1:0] = 1 (as No prescaling)

	/*
	 * Please refer section 17.7.2 Clear Timer on Compare Match (CTC) Mode
	 * of ATmega 328P User Manual.
	 * The following value is obtained by the formula
	 * 16e6 (system clock) / 76e3 - 1 = 210
	 */
        OCR2A = 211 - 1;        // 76 kHz

	/*
	 * Enable the timer
	 */
        TIMSK2 = 1 << OCIE2A;

	/*
	 * Initialize Serial
	 */
	Serial.begin(9600);

	/*
	 * Initialize GPIO pins
	 */
	pinMode(PORT_RED_LED, OUTPUT);

	/*
	 * Configure PORTD for IR-LED output
	 */
	for (i = 0; i < 8; i ++) {
		if (PORTS_IR_LED & (1 << i))
			pinMode(i, OUTPUT);
	}

	pinMode(PORT_IR_SENSOR, INPUT);
}

/*
 * Arduino loop() code
 */
void
loop()
{
	Serial.print("\r\n\nLearning Infrared Remote Controller version ");
	Serial.println(VERSION);
 	Serial.println("Copyright (c) 2010-2015, Atsushi Yokoyama, "
			"Firmlogics");

	Serial.println("\nh<CR> to show help\n");

	for (;;) {
		Serial.print("> ");
		parse_cmd();
	}

	// Never return
}
