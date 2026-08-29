# nRF52840 Pro Micro Wireless Transmitter

This firmware runs on the **SuperMini / Pro Micro nRF52840** module located inside the custom wireless keyboard housing. It acts as the ultra-fast SPI Slave bridge connected to the RP2040 Host Controller, forwarding keyboard inputs, multimedia keys, and battery telemetry over a dedicated 2.4 GHz Enhanced ShockBurst (ESB) wireless link to the USB Receiver Dongle.

---

## Ecosystem & Repositories (v1.0 Stable)

This firmware is part of the 3-tier custom wireless keyboard project:
- 🧠 **[RP2040 Keyboard Controller](https://github.com/Monard2033/RaspberryPicoUSBHost)**: USB Host, Battery ADC & 8 MHz SPI Master.
- 📻 **[nRF52840 Transmitter](https://github.com/Monard2033/nRF52840-Transmitter)** (this repository): Pro Micro SPI Slave & 2.4 GHz ESB PTX (+8 dBm).
- 📡 **[nRF52840 Receiver](https://github.com/Monard2033/nRF52840-Receiver)**: USB Dongle 2.4 GHz ESB PRX & 1000 Hz USB HID Bridge.

### Primary Release Artifacts
- **Firmware Binary**: [`firmware/transmitter.uf2`](firmware/transmitter.uf2)
- **SHA-256 Checksum**: `52996D63E5B6F7BC4225576D4B5427EA804444F583E5B19AC592C0DDAD9C7295`

---

## Key Features & Architecture

1. **High-Speed Hardware SPIS (EasyDMA)**:
   - Configured as a hardware SPI Slave on instance 1 (`SPIS1`) running up to 8 MHz.
   - 12-byte fixed frame protocol (`Link Protocol 0x03`) with hardware EasyDMA transfers completed in $\approx 12\ \mu s$.
2. **High-Power 2.4 GHz ESB Radio (+8 dBm)**:
   - Configured at maximum hardware output power (`ESB_TX_POWER_8DBM` / $+8\text{ dBm} \approx 6.3\text{ mW}$) and 2 Mbps bitrate on **Channel 90 ($2490\text{ MHz}$)**.
   - Operates above Wi-Fi Channel 11 (+17 MHz) and Bluetooth (+10 MHz) for clean RF transmission and immunity to household wireless noise.
3. **Zero Dropped Keys Guarantee**:
   - Radio transmission thread uses a dedicated retry loop (`while (esb_send_once(&frame) != 0)`), guaranteeing that no key presses or releases are ever dropped in RF congestion.
4. **Bidirectional Reverse-ACK Synchronization**:
   - Reads incoming radio ACK payloads carrying the latest Windows NumLock/CapsLock state and prepares a fresh MISO snapshot before each SPI transaction, ensuring zero-delay LED state delivery to RP2040.
5. **Ultra-Low Power Deep Sleep (System OFF @ 0.5 µA)**:
   - Enters deep sleep when the keyboard is idle via SPI control frame (`LINK_CONTROL_SYSTEM_OFF`).
   - Hardware wake-up trigger on `P0.22` (CSN active-low sense) instantly wakes the transmitter upon the first physical keypress.

---

## Hardware Pinout & Wiring (RP2040 $\leftrightarrow$ nRF52840 Transmitter)

| RP2040 Pin | nRF52840 ProMicro Pin | Signal Name | Description |
| :---: | :---: | :---: | :--- |
| **`GP6`** | **`P0.17`** | **`SPI SCK`** | 8 MHz SPI Serial Clock from RP2040 Master |
| **`GP7`** | **`P0.20`** | **`SPI MOSI`** | Serial Data from RP2040 to Transmitter |
| **`GP8`** | **`P0.08`** | **`SPI MISO`** | Reverse ACK / Status snapshot back to RP2040 |
| **`GP9`** | **`P0.22`** | **`SPI CSN`** | Chip Select (Active Low) & Hardware Wake Sense |
| **`3V3 (OUT)`** | **`VCC / 3V3`** | **`Power (3.3V)`** | Regulated 3.3V power rail |
| **`GND`** | **`GND`** | **`Ground`** | Common Ground Reference |

---

## Recommended Hardware & Procurement

- **Recommended Hardware**: **SuperMini nRF52840** / **Pro Micro nRF52840** (nice!nano v2 pinout compatible).
- **Microcontroller**: Nordic nRF52840 (ARM Cortex-M4F @ 64 MHz with FPU, 1 MB Flash, 256 KB RAM).
- **Bootloader**: Pre-installed Adafruit / UF2 Bootloader for drag-and-drop flashing without external programmers.
- **Form Factor**: Compact Pro Micro footprint with Type-C USB for standalone updating.

---

## Flashing & Programming Guide

The Pro Micro nRF52840 board features an onboard UF2 bootloader. No external J-Link / SWD debugger is required.

### Flashing via UF2 Bootloader (Drag-and-Drop):

1. Connect the Pro Micro nRF52840 to your PC via a USB Type-C cable.
2. **Double-click the small RST button** on the board (or briefly short `RST` to `GND` twice).
3. A mass storage drive named **`NICENANO`** or **`NRF52BOOT`** will appear in Windows Explorer.
4. Drag and drop the file [`firmware/transmitter.uf2`](firmware/transmitter.uf2) onto the drive.
5. The drive will automatically disconnect and the transmitter firmware will boot immediately!

---

## Critical `prj.conf` Configuration Directives

The following Zephyr / Nordic Connect SDK (NCS v3.4.0) options must be strictly preserved:

```ini
# Silent release build: zero UART/logging CPU overhead
CONFIG_LOG=n
CONFIG_CONSOLE=n
CONFIG_BOOT_BANNER=n
CONFIG_NCS_BOOT_BANNER=n
CONFIG_PRINTK=n
CONFIG_UART_CONSOLE=n
CONFIG_SERIAL=n

# ESB High-Priority Radio Subsystem
CONFIG_ESB=y
CONFIG_CLOCK_CONTROL=y
CONFIG_ESB_CLOCK_INIT=y
CONFIG_ESB_PIPE_COUNT=8
CONFIG_ESB_RADIO_IRQ_PRIORITY=0

# Zephyr SPIS Hardware Slave Driver (instance 1)
CONFIG_SPI=y
CONFIG_NRFX_SPIS=n

# Power Management (System OFF support)
CONFIG_PM_DEVICE=y
CONFIG_POWEROFF=y
CONFIG_SIZE_OPTIMIZATIONS=y
CONFIG_BUILD_OUTPUT_UF2=y
```
