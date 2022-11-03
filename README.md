# Daft's Emulator for RP2040
This is mostly an _extremely hacky_ PicoSystem emulator.

```
./DERP_SDL file.uf2
```

## PicoSystem status
| Feature | Status | Details 
|---------|--------|---------
| Display | Hack   | No PIO/SPI, hack in DMA code. (Assumes 32blit-sdk setup)
| Buttons | Works  |
| Sound   | TODO   |
| LED     | TODO   |       

## RP2040 status

| Feature | Status | Details 
|---------|--------|---------
| Cortex-M0+ | Partial   | One core. Copied from GBA emu, so some details are still an ARM7
| DMA | Minimal | Implemented just enough for PicoSystem display updates
| Clocks | Partial |
| XOSC | Minimal | Fixed 12MHz
| ROSC | Minimal | Fixed 6MHz
| PLL | Partial |
| GPIO | Partial | Some input and interrupt handling
| PIO | Minimal | PIO0 always returns all FIFOs empty + TXSTALL
| USB | Minimal | RAM exists
| UART | None |
| I2C | None |
| SPI | Minimal | SPI0 TNF always set
| PWM | Minimal | Register read/write handled
| Timer | Partial |
| Watchdog | Partial | Ticks for timer
| RTC | None |
| ADC | None |
| SSI | Minimal | Just enough to boot