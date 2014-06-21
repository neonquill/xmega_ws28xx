# XMega WS28xx demo

Basic demo of WS28xx (800 MHz) communication protocol using the Atmel
XMega USART in master SPI mode.  Also uses DMA to automatically
transfer the data to the USART.

## Timing

The XMega is running at 32 MHz, and the USART is running at 3.2 MHz.
This means that a single bit has a width of (1 / 3.2 MHz) = 0.312
microseconds.  Based on the [timing
diagram](https://learn.adafruit.com/adafruit-neopixel-uberguide/advanced-coding)
a single pulse will work for both the T0H pulse and the T1L pulse.
Three bits equal 0.9375 microseconds, which satisfies the timing for
the T0L and T1H pulse widths.  So, a single bit for the WS28xx is
created using 4 bits on the USART.

## Logic levels

By default, the SPI defaults to a high logic level when high.  The
WS28xx needs to be logic level low at idle.  To fix this, we just
invert the output of the data pin and invert the logic levels going
into the USART.

## Testing

This has been tested using an XMega32a4u and a single [Adafruit
NeoPixel Clear 5mm Through-Hole
LED](http://www.adafruit.com/products/1837).

## License

Unless otherwise noted, all files are released into the public as
described in the UNLICENSE file.
