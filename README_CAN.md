# CANFD Motor Control

This project accepts CANFD commands to control the motor in position mode or
speed mode.

On power-up, the motor PWM is disabled. The motor does not move until a valid
CAN command with `enable = 1` is received.

## Bus Settings

Configure your CANFD tool like this:

```text
CAN FD: enabled
BRS: enabled
Nominal bitrate: 500 kbps
Data bitrate: 5 Mbps
ID type: Standard ID
Command ID: 0x201
Status ID: 0x181
```

The board uses FDCAN1:

```text
PB10: FDCAN1_TX
PB12: FDCAN1_RX
```

Use a CANFD transceiver between the MCU pins and the CAN bus.

## Command Frame

Send commands to:

```text
Standard ID: 0x201
DLC: 8
```

Data format:

```text
Byte0: mode
       0 = idle
       1 = position mode
       2 = speed mode

Byte1: enable
       0 = disable CAN control
       1 = enable command

Byte2-3: reserved, set to 0

Byte4-7: target value, int32 little-endian
```

Little-endian means the lowest byte is sent first.

## Position Mode

Position unit is `mdeg`.

```text
1 degree = 1000 mdeg
10 degrees = 10000 mdeg
90 degrees = 90000 mdeg
360 degrees = 360000 mdeg
```

Command format:

```text
Byte0 = 01
Byte1 = 01
Byte2 = 00
Byte3 = 00
Byte4-7 = target position in mdeg
```

Examples:

```text
Go to 10 degrees:
ID:   0x201
Data: 01 01 00 00 10 27 00 00

Go to 90 degrees:
ID:   0x201
Data: 01 01 00 00 90 5F 01 00

Go to 360 degrees:
ID:   0x201
Data: 01 01 00 00 40 7E 05 00
```

## Speed Mode

Speed unit is mechanical `mHz`.

```text
1000 mHz = 1 revolution per second
500 mHz = 0.5 revolution per second
-500 mHz = -0.5 revolution per second
```

Command format:

```text
Byte0 = 02
Byte1 = 01
Byte2 = 00
Byte3 = 00
Byte4-7 = target mechanical speed in mHz
```

Examples:

```text
Run at 1 revolution per second:
ID:   0x201
Data: 02 01 00 00 E8 03 00 00

Run at 0.5 revolution per second:
ID:   0x201
Data: 02 01 00 00 F4 01 00 00

Run at -0.5 revolution per second:
ID:   0x201
Data: 02 01 00 00 0C FE FF FF
```

## Disable CAN Control

```text
ID:   0x201
Data: 00 00 00 00 00 00 00 00
```

## Status Frame

The board sends status every 50 ms:

```text
Standard ID: 0x181
DLC: 16
CANFD + BRS
```

Data format:

```text
Byte0: current mode
       0 = idle
       1 = position mode
       2 = speed mode

Byte1: active
       0 = CAN control inactive
       1 = CAN control active

Byte2-3: received command counter, uint16 little-endian
Byte4-7: current mechanical position, int32 mdeg little-endian
Byte8-11: current target value, int32 little-endian
Byte12-15: current mechanical speed, int32 mHz little-endian
```
