# linux-wmt
This repository is a Linux kernel fork dedicated to bringing modern support to the **WonderMedia WM8505**, specifically targeting the early-2010s ARM netbooks built around this SoC.

This project modernizes WM8505 support by migrating it to current kernel frameworks and writing new drivers for the hardware found in these devices.

> **⚠️ Note:** This branch is frequently rebased to track upstream Linux releases and maintain a clean patch stack.

## Supported Hardware & Features

*   **Display & Graphics (DRM/KMS)**
    *   Atomic KMS display driver for the built-in LCD panel, with vblank-timed page flips.
    *   Panel driver reading each board's LCD timings from the bootloader, falling back to the device tree.
    *   Video DMA (VDMA) support for copies that convert between 16 and 32 bpp, as used by a 16 bpp desktop.
    *   2D Graphics Engine (GE) support for asynchronous solid fills and blits.
    *   Framebuffer console (fbcon) acceleration on the GE.
*   **Video (V4L2)**
    *   JPEG Decoder (JDEC) support for baseline JPEG and Motion JPEG.
    *   Scaler (SCL) support for image scaling and NV12 to RGB color conversion.
*   **Audio (ASoC)**
    *   AC97 controller driver with DAPM, driving the board's AC97 codec (VT1613, VT1612A, or WM9715L).
    *   DMA-backed PCM playback and capture, including internal speaker and headphone support.
*   **Input**
    *   PS/2 driver for the built-in keyboard and touchpad.
*   **Storage**
    *   SD/MMC controller support for the SD card slot, with burst DMA transfers.
    *   Serial Flash Controller (SFC) support for the internal SPI flash.
*   **Power**
    *   Battery monitor for the VT1613 and VT1612A boards, reporting charge, status, and a low-battery alarm.
    *   Voltage gauged without an ADC by measuring an RC circuit's rise time on a GPIO.
    *   Calibrated at the low-battery alarm, with the result exposed for userspace to save and restore.
*   **Core SoC**
    *   Clock controller driver on the Common Clock Framework (CCF), covering the full SoC clock tree.
    *   DMA controller support for scatter-gather peripheral and memory-to-memory transfers.
