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
 *  1: SPI clock (don't connect)
 *  3: WS2811 Data in.
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
 * 1 bit = 0b0001
 * Note, there are 4 UART bits per per bit to the WS2812.
 */
static uint8_t pair_to_data_byte[4] = {
  /* 0b00 */
  0b01110111,
  /* 0b01 */
  0b01110001,
  /* 0b10 */
  0b00010111,
  /* 0b11 */
  0b00010001
};

/**
 * Set color values for a given pixel.
 *
 * Ordering is the same as the driver chip expects.
 * Most of the devices are green, red, blue.
 * Some are red, green, blue.
 *
 * @param[in] pixel  The index of the pixel.
 * @param[in] first  The first color value (normally green).
 * @param[in] second  The second color value (normally red).
 * @param[in] third  The third color value (normally blue).
 */
void
set_pixel_color(uint8_t pixel, uint8_t first, uint8_t second, uint8_t third) {
  uint8_t pair;
  uint8_t offset;

  /* Start at the end and work forward. */
  offset = (pixel + 1) * BYTES_PER_PIXEL - 1;

  /* Third */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[third & 0b11];
    third >>= 2;
    offset--;
  }

  /* Second */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[second & 0b11];
    second >>= 2;
    offset--;
  }

  /* First */
  for (pair = 0; pair < 4; pair++) {
    led_data[offset] = pair_to_data_byte[first & 0b11];
    first >>= 2;
    offset--;
  }
}

/**
 * Interrupt routine for a completed DMA transfer.
 */
ISR(DMA_CH0_vect) {
  PORTA.OUTTGL = PIN1_bm;
  /* Clear the interrupt flag so we stop getting called. */
  DMA.INTFLAGS |= DMA_CH0TRNIF_bm;
  // PORTA.OUTCLR = PIN1_bm;
}

/**
 * Start a DMA transfer to send data to the chip.
 */
static void
start_dma(void) {
  /* Enable the DMA transaction complete interrupt at low. */
  /* XXX Maybe move this into the setup. */
  DMA.CH0.CTRLB = DMA_CH_TRNINTLVL_LO_gc;

  /* Transfer a all the LED data. */
  DMA.CH0.TRFCNT = NUM_DATA_BYTES;

  /* Transfer 1 block. */
  DMA.CH0.REPCNT = 1;

  /*
   * Start the transaction.
   * Note: The data register is empty, so it will automatically start the
   *   transfer.
   */
  DMA.CH0.CTRLA |= DMA_CH_ENABLE_bm;
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
 * Set up the DMA controller.
 *
 * Used by the LED driver to copy the LED bits to the USART.
 */
void
setup_dma(void) {
  uint16_t temp;

  // LED controller uses DMA 0.

  // Enable DMA, double buffer disabled, round robin priority.
  DMA.CTRL = DMA_RESET_bm;
  DMA.CTRL = DMA_ENABLE_bm;

  /* Set up DMA0 to transfer the data into the USART. */

  /* Reset, just to be safe. */
  DMA.CH0.CTRLA = DMA_CH_RESET_bm;

  /*
   * DMA_CH_SINGLE_bm:
   *   Just do a single burst transfer each time the trigger fires.
   * DMA_CH_BURSTLEN_1BYTE_gc:
   *   Each burst transfers a single byte.
   */
  DMA.CH0.CTRLA = DMA_CH_SINGLE_bm | DMA_CH_BURSTLEN_1BYTE_gc;

  /*
   * DMA_CH_SRCRELOAD_TRANSACTION_gc:
   *   Reload the original src address after every transaction.
   * DMA_CH_SRCDIR_INC_gc:
   *   Increment the src address after every burst.
   * DMA_CH_DESTRELOAD_NONE_gc:
   *   Don't reload the destination address after the transaction.
   * DMA_CH_DESTDIR_FIXED_gc:
   *   Keep the destination address fixed.
   */
  DMA.CH0.ADDRCTRL = (DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc |
                      DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc);

  /* Transfers are triggered when the USART D0 data register is empty. */
  DMA.CH0.TRIGSRC = DMA_CH_TRIGSRC_USARTD0_DRE_gc;

  /* Set the source address to the beginning of the led_data array. */
  temp = (uint16_t)led_data;
  DMA.CH0.SRCADDR0 = temp;
  DMA.CH0.SRCADDR1 = temp >> 8;
  DMA.CH0.SRCADDR2 = 0;

  /* Set the destination address to the USART data register. */
  temp = (uint16_t)(&(USARTD0.DATA));
  DMA.CH0.DESTADDR0 = temp;
  DMA.CH0.DESTADDR1 = temp >> 8;
  DMA.CH0.DESTADDR2 = 0;
}

/**
 * Main set up routine.
 */
void
setup(void) {
  setup_processor_clocks();
  setup_pins();
  setup_usart();
  setup_dma();

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
  static int8_t increment[3] = {5, 0, 0};
  static uint8_t red = 0, green = 0, blue = 0;

  /* 5mm through-hole LEDs are RGB, not GRB. */
  set_pixel_color(0, red, green, blue);
  PORTD.OUTSET = PIN5_bm;
  start_dma();
  PORTD.OUTCLR = PIN5_bm;
  if (red == 255) {
    increment[0] = -5;
  } else if (green == 255) {
    increment[1] = -5;
  } else if (blue == 255) {
    increment[2] = -5;
  } else if (red == 0 && increment[0] == -5) {
    increment[0] = 0;
    increment[1] = 5;
  } else if (green == 0 && increment[1] == -5) {
    increment[1] = 0;
    increment[2] = 5;
  } else if (blue == 0 && increment[2] == -5) {
    increment[2] = 0;
    increment[0] = 5;
  }
  red += increment[0];
  green += increment[1];
  blue += increment[2];

  _delay_ms(20);
}

int
main(void) {
  setup();

  blink(3);

  while(1) {
    loop();
  }
}
