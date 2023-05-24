# Daft's Emulator for RP2040
This is mostly an _extremely hacky_ PicoSystem emulator.

```
./DERP_SDL file.uf2
```

(To run uf2s built with the PicoSystem SDK add `--picosystem-sdk` before the file to apply some workarounds)

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
| Display | Hack   | No PIO/SPI, hack in DMA code. (Assumes 32blit-sdk setup)
| Buttons | Works  |
| Sound   | Works  |
| LED     | TODO   |       

## RP2040 status

| Feature | Status | Details 
|---------|--------|---------
| Cortex-M0+ | Partial   | Copied from GBA emu, so some details are still an ARM7
| DMA | Minimal | Implemented just enough for PicoSystem display updates
| Clocks | Partial |
| XOSC | Minimal | Fixed 12MHz
| ROSC | Minimal | Fixed 6MHz
| PLL | Partial |
| GPIO | Partial | Missing overrides, partial interrupt handling
| PIO | Minimal | PIO0 always returns all FIFOs empty + TXSTALL
| USB | Partial | Optional device enumeration and USBIP server
| UART | Minimal | Prints TX data
| I2C | None |
| SPI | Minimal | SPI0 TNF always set
| PWM | Partial | Enough for PicoSystem beeps
| Timer | Partial |
| Watchdog | Partial | Tick generation and reset timer
| RTC | Minimal | Enough to get through init
| ADC | None |
| SSI | Minimal | Just enough to boot

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