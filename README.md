# Daft's Emulator for RP2040
This is mostly an _extremely hacky_ PicoSystem emulator.

```
./DERP_SDL file.uf2
```

(To run some uf2s built with the PicoSystem SDK add `--board pimoroni_picosystem` before the file to override the board value)

## Bootrom
A copy of the RP2040 bootrom is required in bootrom.bin.

This can be created from the elf files in [the official releases](https://github.com/raspberrypi/pico-bootrom/releases):
```
arm-none-eabi-objcopy -O binary b2.elf bootrom.bin
```
... or dumped from a real RP2040 with picotool:
```
picotool save -r 0 4000 bootrom.bin
```

## PicoSystem status
| Feature | Status | Details 
|---------|--------|---------
| Display | Hack   | No SPI, hook in PIO code. Enough display regs implemented to guess 32blit vs picosystem SDK
| Buttons | Works  |
| Sound   | Works  |
| LED     | TODO   |       

## RP2040 status

|  Feature   | Status  | Details 
|------------|---------|---------
| Cortex-M0+ | Partial | Copied from GBA emu, so some details are still an ARM7
| DMA        | Minimal | Implemented just enough for PicoSystem display updates
| Clocks     | Partial |
| XOSC       | Minimal | Fixed 12MHz
| ROSC       | Minimal | Fixed 6MHz
| PLL        | Partial |
| GPIO       | Partial | Missing overrides, partial interrupt handling
| PIO        | Minimal | Some status flags, fakes TX progress
| USB        | Partial | Optional device enumeration and USBIP server
| UART       | Minimal | Prints TX data
| I2C        | Minimal | Some registers stubbed
| SPI        | Minimal | SPI0 TNF always set
| PWM        | Partial | Enough for PicoSystem beeps
| Timer      | Partial |
| Watchdog   | Partial | Tick generation and reset timer
| RTC        | Minimal | Enough to get through init
| ADC        | None    |
| SSI        | Minimal | Just enough to boot

## Other Boards
Additional boards with some support:

|     Name     |         --board         | Details
|--------------|-------------------------|--------
| Pico         | `pico` or default       | No extra peripherals, so no output
| Interstate75 | `pimoroni_interstate75` | Emulates a single 32x32 panel.
| Tufty2040    | `pimoroni_tufty2040`    | Similar display hacks to PicoSystem

## GDB Server
A GDB server can be enabled on port 3333 with:
```
./DERP_SDL --gdb --board [board name]
```

To use this with Cortex-Debug, replace `"servertype": "openocd"` with
```json
"servertype": "external",
"gdbTarget": "localhost:3333",
```

## USB

Optional USB support can be enabled with:

```
./DERP_SDL --usb [...]
```

This will attempt to enumerate the device and print the output from a CDC interface if found.

Alternatively:
```
./DERP_SDL --usbip [...]
```

Will enable a USBIP server. Which can be connected to using something like:
(May require some extra `sudo`)

```shell
modprobe vhci-hcd

# check for devices
usbip list -r localhost

# attach device
usbip attach -r localhost -b 1-1
```

Launching without a .uf2 file does allow using the bootrom's loader. Though this is probably not a good idea...

(Low level transfers/timings are not implemented so the transfer rate for USB data is... unpredictable)
