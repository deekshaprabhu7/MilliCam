# MilliCam

## Base Code

This project is built on the base code from the [BeetleCam Repository](https://github.com/uw-x/insect-robot-cam)

## Configuration

The following configuration was used for this project:

- **Development Board**: nRF5340-DK
- **Development Environment**: nRF Connect SDK
- **Camera Module**: Himax HM01B0 on a custom board

## Pin Configuration

Below is the table with pin details for connecting the camera module to the nRF53:

| Pin            | Camera Pin       | nRF53 Pin      | Comments                     |
|----------------|------------------|----------------|------------------------------|
| I2C SCL        | SCL (4 Top)      | P1.03          |                              |
| I2C SDA        | SDA (3 Top)      | P1.02          |                              |
| SPI MISO       |                  | P1.07          |                              |
| SPI MOSI(D0)   | D0 (11)          | P1.06          |                              |
| SPI SCK (PCLK) | PCLK (9)         | P1.05          |                              |
| SPI CS         |                  | P1.04          | SPI CS is shorted to GPIO CS |
| GPIO CS        |                  | P1.11          | SPI CS is shorted to GPIO CS |
| MLCK           | MCLK (2 Top)     | P0.26          |                              |
| Frame Valid    | FVLD (8)         | P0.27          |                              |
| Line Valid     | NetP42_1 (1 Top) | P0.25          |                              |
| Cam Pow (Vin)  | Vin (1)          | VDD            |                              |
| Ground         | Gnd (2)          | GND            |                              |

