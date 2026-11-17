# dsPIC33 Embedded system Project

Embedded firmware for a differential-drive robot buggy based on the **dsPIC33EP** microcontroller. Developed as part of an Embedded Systems course project at UniGe.

---

## Overview

The firmware implements a complete control loop running at **500 Hz**, handling motor PWM generation, infrared distance sensing, magnetometer data acquisition via SPI, battery monitoring, and a full bidirectional UART command interface with interrupt-driven circular buffers.

---

## Hardware

| Component | Details |
|---|---|
| MCU | dsPIC33EP (Buggy + Clicker Mikrobus) |
| Motor driver | 4× Output Compare peripherals (OC1–OC4), 10 kHz PWM |
| Magnetometer | BMX055 via SPI1 |
| IR distance sensor | Analog (AN14/AN15), enable on RB9 |
| Battery sensing | Voltage divider on AN11 (200 kΩ / 100 kΩ) |
| UART/RS232 | Clicker Mikrobus 2 — TX: RD0/RP64, RX: RD11/RPI75 |

---

## Features

### Finite State Machine

```
         [RE8 press]           [distance < 20 cm]
WAIT ─────────────────► MOVING ──────────────────► EMERGENCY
 ◄─────────────────────         ◄──────────────────
         [RE8 press]           [clear for ≥ 5 s]
```

| State | Behaviour |
|---|---|
| `STATE_WAIT` | Motors stopped, LED A0 blinks at 1 Hz |
| `STATE_MOVING` | PWM driven by UART speed/yawrate commands |
| `STATE_EMERGENCY` | Motors stopped, side lights blink at 1 Hz, sends `$MEMRG,1*` |

- RE8 button uses hardware debounce via Timer1 (50 ms).
- RE8 is ignored in `STATE_EMERGENCY`.

### Motor Control

Speed/yawrate mixing:
```
left_pwm  = speed − yawrate
right_pwm = speed + yawrate
```
Both clamped to [−100, 100] % before PWM generation.

| Condition | RD2 (LEFTF) | RD1 (LEFTB) | RD4 (RIGHTF) | RD3 (RIGHTB) |
|---|---|---|---|---|
| left_pwm > 0 | left_pwm | 0 | — | — |
| left_pwm < 0 | 0 | −left_pwm | — | — |
| right_pwm > 0 | — | — | right_pwm | 0 |
| right_pwm < 0 | — | — | 0 | −right_pwm |

### UART Protocol

**Outgoing messages (MCU → PC)**

| Message | Rate | Description |
|---|---|---|
| `$MBATT,X.YZ*` | 1 Hz | Battery voltage in Volts |
| `$MDIST,N*` | 10 Hz | IR distance in cm (integer) |
| `$MACC,x,y,z*` | 10 Hz | Magnetometer x, y, z in mg |
| `$MEMRG,1*` | on event | Emergency state entered |
| `$MEMRG,0*` | on event | Emergency state cleared |
| `$MACK,1*` | on command | Command accepted |
| `$MACK,0*` | on command | Command rejected (emergency active or buffer full) |

**Incoming commands (PC → MCU)**

| Command | Action |
|---|---|
| `$PCREF,speed,yawrate*` | Set speed (−100…100) and yawrate (−100…100) |
| `$PCSTP,*` | Switch to WAIT state (rejected in EMERGENCY) |
| `$PCSTT,*` | Switch to MOVING state (rejected in EMERGENCY) |

### Buffers

- **TX**: interrupt-driven `CircularBuffer` (char, size `BUFFER_SIZE`). TX ISR pops one byte at a time.
- **RX command FIFO**: `CMDCircularBuffer` holds up to 10 parsed `parser_state` frames. Overflow returns `$MACK,0*`.
- `enqueue_buffer()` disables the TX interrupt while writing to prevent race conditions.

---

## File Structure

```
dsPIC33-embedded-project/
├── src/
│   ├── main.c          # Main loop, ISRs, FSM
│   ├── drivers.c       # Peripheral drivers (timers, UART, SPI, ADC, PWM, buffers, parser)
│   └── drivers.h       # Shared types, macros, function declarations
├── docs/
│   └── project_spec.pdf
└── README.md
```

---

## Pin Mapping

| Pin | Function |
|---|---|
| RA0 | LED (main loop heartbeat) |
| RB8 | Left side light |
| RF1 | Right side light |
| RD1/RP65 | Left PWM — backward (OC1) |
| RD2/RP66 | Left PWM — forward (OC2) |
| RD3/RP67 | Right PWM — backward (OC3) |
| RD4/RP68 | Right PWM — forward (OC4) |
| AN11 | Battery voltage sense |
| AN14 | IR sensor analog input |
| RB9 | IR sensor enable |
| RD0/RP64 | UART TX |
| RD11/RPI75 | UART RX |
| RE8 | Start/stop button |
| RA1/RPI17 | SPI1 MISO (magnetometer) |
| RF12/RP108 | SPI1 SCK |
| RF13/RP109 | SPI1 MOSI |

---

## Timing Budget (500 Hz loop, 2 ms period)

| Task | Rate |
|---|---|
| IR distance + battery ADC read | 500 Hz |
| PWM update | 500 Hz |
| `$MDIST` + `$MACC` messages | 10 Hz (every 50 iterations) |
| `$MBATT` message + LED toggle | 1 Hz (every 500 iterations) |
| Emergency light blink | 1 Hz (every 500 iterations) |

---
## Authors
- AGUENAROUS Mohamed
- MOKRANI Lisa
