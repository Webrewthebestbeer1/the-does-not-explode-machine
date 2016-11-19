## SPI bus

Connecting the TFT screen and SD card to the same SPI bus did not work for some reason. Therefor, connect the TFT to SPI bus 1 and the SD card to the SPI bus 2.

SPI bus 1

    CS PA4
    SCK PA5
    MISO PA6 (not needed for TFT screen)
    MOSI PA7

SPI bus 2

    CS PB12
    SCK PB13
    MISO PB14
    MOSI PB15

## Libraries

Arduino STM32 from https://github.com/rogerclarkmelbourne/Arduino_STM32.git (into /hardware directory)

The Adafruit GFX library for the ST7355, ported to the STM32

    https://github.com/KenjutsuGH/Adafruit-ST7735-Library

SdFat from

    https://github.com/greiman/SdFat-beta

From the "Manage Libraries..." tool in the Arduino IDE

    - OneWire
    - DS1820
