# hello-lvgl-esp-idf
Painfully over-commented, single sourcefile example of a simple LVGL menu system on ESP32 using ESP-IDF

## What is this
An ESP-IDF project with a single main.c source file that connects the ESP LCD driver with LVGL, with detailed commentary on the implementation and decision making process. This project is based on Espressif's `spi_lcd_touch_example_main`, but has been re-written, simplified, and documented for educational purposes.

I created this project while learning how to integrate LVGL with ESP-IDF for my own audio player project. I found lots of official documentation on how to do this integration, several demos that leaned heavily on Arduino infrastructure, and also a few helper libraries, But I didn't find any end-to-end tutorials focused on IDF that would help me (an LCD/LVGL layperson) actually understand what was going on. So, I made one.

This tutorial project follows the complete system narrative from hardware setup through LVGL integration to setting up a working menu, with explanatory comments along the way. 

[Documented main.c](/main/main.c)

![Demo Menu GUI](assets/demo-run.gif "Demo Menu GUI")

## Hardware
- ESP-WROOM-32D dev board
- Waveshare 2" LCD (240x320) with ST7789V controller
- UART/Serial for keyboard input

## Project Setup
This is a fairly standard ESP IDF project setup with one main component. LVGL is **included as a managed component**, so no manual download/clone/etc. is required -- this idf_component.yml will make idf.py grab LVGL for you. 

## Helpful Documentation
I leaned heavily on these resources to make this project!
- [Espressif LCD SPI Example](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/lcd/spi_lcd_touch) (original basis)
- [ESP-IDF LCD Driver Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/lcd.html)
- [LVGL ESP32 Integration Docs](https://docs.lvgl.io/master/integration/chip/espressif.html)
- [LVGL Menu Widget Examples](https://docs.lvgl.io/master/widgets/menu.html)

## CORRECTIONS WANTED AND APPRECIATED
As mentioned, I'm new to LVGL. I got this project working and did my best to make sure the technical commentary is accurate, but if you notice any issues or discrepencies, please let me know by opening an issue!