# ILI9341 Project ESP-IDF

# Overview

This project demonstrates how to read temperature and humidity data from a DHT22 sensor and display it using LVGL (Light and Versatile Graphics Library). The temperature and humidity data are updated in real-time on a graphical user interface (GUI) running on an embedded system.

# Hardware Requirements

- ESP32 development board
- DHT22 sensor
- ILI9341 TFT LCD display

# Wiring

| ILI9341 Pin | ESP32 Pin | Description |
| --- | --- | --- |
| VCC | 3.3V | Power supply |
| GND | GND | Ground |
| CS | GPIO 13 | Chip Select |
| RST | GPIO 12 | Reset |
| DC (RS) | GPIO 14 | Data/Command |
| SDI (MOSI) | GPIO 27 | SPI Master Out Slave In |
| SCK(CLK) | GPIO 26 | SPI Clock |
| LED | 3.3V | Backlight |
| SDO (MISO) | GPIO 25 | SPI Mater In Slave Out |

# Add Library

## LVGL (8.3.7)

From your project's root directory:

1. Download here: https://github.com/lvgl/lvgl/archive/refs/tags/v8.3.7.zip
2. Create a directory named `components` (if you don't have one already) with `mkdir -p components`
3. Move/copy `lvgl/` to `/components`
4. Run `idf.py menuconfig`, go to `Component config` then `LVGL configuration` to configure LVGL.

## lvgl_esp32_drivers

From your project's root directory:

1. Clone the lvgl_esp32_drivers
    
    ```bash
    git clone https://github.com/hiruna/lvgl_esp32_drivers/tree/develop/lvgl_8.3.7_idf_5.2
    ```
    
2. Create a directory named `components` (if you don't have one already) with `mkdir -p components`.
3. Move/copy `lvgl_esp32_drivers` to `/components`
4. Run `idf.py menuconfig`, go to `Component config` then `LVGL TFT configuration` and `LVGL TFT Display configuration` to configure lvgl_esp32_drivers.

## am2302_rmt

From your project's root directory:

1. Download Here: https://components.espressif.com/api/download/?object_type=component&object_id=2446f0b5-72f7-406f-bc51-20baafcde756
2. Create a directory named `components` (if you don't have one already) with `mkdir -p components`.
3. Move/copy am2302_rmt to `/components`

# How to use

1. Build/compile the project
    
    ```bash
    idf.py build
    ```
    
2. If no error exist, flash the program
    
    ```bash
    idf.py flash
    ```
    

# References

https://github.com/xpress-embedo/ESP32/tree/master/ESP-IDF/LVGL_HelloWorld

https://docs.lvgl.io/8.3/index.html