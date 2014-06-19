#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <stdio.h>
#include "xmega_lib/clksys_driver.h"

/*
 * Pins:
 *
 * Port D: (uses SPI)
 *  0: LED GSCLK
 *  1: LED SCLK
 *  2: LED XLAT
 *  3: LED SIN
 *  4: LED Blank
 */

/**
 * Configure all the pins.
 */
void
setup_pins(void) {
  /* For direction: 0 = input, 1 = output. */

  PORTA.DIR = 0xff;
  PORTA.OUT = 0x00;

  PORTB.DIR = 0xff;
  PORTB.OUT = 0x00;

  PORTC.DIR = 0xff;
  PORTC.OUT = 0x00;

  /* Port D: all outputs. */
  PORTD.DIR = 0xff;
  PORTD.OUT = 0x00;
  /* Make pin3 inverted. */
  PORTD.PIN3CTRL = PORT_INVEN_bm;

  PORTE.DIR = 0xff;
  PORTE.OUT = 0x00;
}

/**
 * Debugging routine to blink out state.
 *
 * @param[in] count  Number of times to blink the debug LED.
 */
void
blink(int count) {
  int i;

  for (i = 0; i < count; i++) {
    PORTA.OUTSET = PIN1_bm;
    _delay_ms(125);
    PORTA.OUTCLR = PIN1_bm;
    _delay_ms(250);
    PORTA.OUTSET = PIN1_bm;
    _delay_ms(125);
  }

  _delay_ms(400);
}

/*
 * Globals.
 */
enum {
  NUM_PIXELS = 1,

  /* Each pixel takes 3 bytes * 4 in bits per out bit = 12 bytes. */
  BYTES_PER_PIXEL = 3 * 4,

  /* 3 bytes per pixel, 4 data bits per output bit. */
  NUM_DATA_BYTES = NUM_PIXELS * BYTES_PER_PIXEL,
};

static uint8_t led_data[NUM_DATA_BYTES];

/*
 * Output is inverted.
 * 0 bit = 0b0111
 * 1 bit = 0b1110
 * Note, there are 4 UART bits per per bit to the WS2812.
 */
static uint8_t pair_to_data_byte[4] = {
  /* 0b00 */
  0b01110111,
  /* 0b01 */
  0b01111110,
  /* 0b10 */
  0b11100111,
  /* 0b11 */
  0b11101110
};

void
set_pixel_color(uint8_t pixel, uint8_t red, uint8_t green, uint8_t blue) {
  uint8_t pair;
  uint8_t offset;

  offset = pixel * BYTES_PER_PIXEL;

  /* Green */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[green & 0b11];
    green >>= 2;
    offset++;
  }

  /* Red */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[red & 0b11];
    red >>= 2;
    offset++;
  }

  /* Blue */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[blue & 0b11];
    blue >>= 2;
    offset++;
  }
}

/**
 * Set up all the processor clock inputs.
 */
void
setup_processor_clocks(void) {
  // Enable the 32 MHz internal oscillator.
  CLKSYS_Enable(OSC_RC32MEN_bm);

  // Wait for the 32 MHz clock to be ready.
  do {
  } while (CLKSYS_IsReady(OSC_RC32MRDY_bm) == 0);

  // Switch to the 32 MHz clock for the main system clock.
  // XXX Check the return value!
  CLKSYS_Main_ClockSource_Select(CLK_SCLKSEL_RC32M_gc);

  // Turn off all the other clocks.
  // XXX What about the external clock?
  CLKSYS_Disable(OSC_PLLEN_bm | OSC_RC32KEN_bm | OSC_RC2MEN_bm);

  // Enable automatic calibration of the 32MHz clock.
  OSC.DFLLCTRL = OSC_RC32MCREF_XOSC32K_gc;
  DFLLRC32M.CTRL = DFLL_ENABLE_bm;
}

/**
 * Configure the USART.
 */
static void
setup_usart(void) {
  /* Only enable the transmitter. */
  USARTD0.CTRLB = USART_TXEN_bm;

  /*
   * Master SPI mode,
   * MSB transmitted first,
   * leading edge sample (SPI Mode 0).
   */
  USARTD0.CTRLC = USART_CMODE_MSPI_gc;

  /* Setting BSEL to 4 and BSCALE to 0 should get a frequency of 3.2 MHz. */
  USARTD0.BAUDCTRLA = 4;
  USARTD0.BAUDCTRLB = 0;
}

/**
 * Main set up routine.
 */
void
setup(void) {
  setup_processor_clocks();
  setup_usart();
  setup_pins();

  // Enable low and medium level interrupts.
  PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;

  /* Enable interrupts. */
  sei();
}

/**
 * Main run time processing loop.
 */
void
loop(void) {
  static uint8_t red = 0, green = 0, blue = 0;
  blink(1);

  set_pixel_color(0, red, green, blue);

  PORTD.OUTSET = PIN5_bm;
  USARTD0.DATA = led_data[0];
  asm("nop");
  USARTD0.DATA = led_data[1];
  while ((USARTD0.STATUS & USART_TXCIF_bm) == 0) {}
  USARTD0.STATUS = USART_TXCIF_bm;
  PORTD.OUTCLR = PIN5_bm;

  _delay_ms(1000);
}

int
main(void) {
  setup();

  blink(3);

  while(1) {
    loop();
  }
}
