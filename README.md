# H723 BootLoader Communication Protocol

## Communication

- Physical interface: UART3 / RS485.
- The BootLoader loads the slave address and baud rate from EEPROM during startup.
- Modbus RTU CRC16 uses polynomial `0xA001`, initial value `0xFFFF`, and is sent low byte first (`CRC_L`, then `CRC_H`).
- A host must use the device's current slave address and baud rate. A Modbus broadcast read has no response.

### Baud-rate codes

| Code | Baud rate |
| --- | --- |
| `1` | 9600 |
| `2` | 19200 |
| `3` | 38400 |
| `4` | 57600 |
| `5` | 115200 |
| `6` | 230400 |
| `7` | 460800 |
| `8` | 921600 |

## Read communication parameters

Use standard Modbus RTU function `0x03` (Read Holding Registers). Values are read directly from EEPROM.

| Register | Value | Encoding |
| --- | --- | --- |
| `0x0000` | Slave address | `uint16`, high byte is `0x00` |
| `0x0001` | Baud-rate code | `uint16`, high byte is `0x00` |

Read one register or both contiguous registers. The requested quantity must be `1` or `2`.

```text
Request
[Slave][03][Start_H][Start_L][Qty_H][Qty_L][CRC_L][CRC_H]

Success response
[Slave][03][ByteCount][Data_H][Data_L]...[CRC_L][CRC_H]
```

Example: the device address is `0xC8`, and EEPROM stores `115200`, whose baud-rate code is `5`.

```text
Request : C8 03 00 00 00 02 D5 92
Response: C8 03 04 00 C8 00 05 E2 C2
```

Modbus exception responses use `[Slave][83][Exception][CRC_L][CRC_H]`.

| Exception | Meaning |
| --- | --- |
| `0x02` | Unsupported register address or range |
| `0x03` | Quantity is zero or greater than two |
| `0x04` | EEPROM data cannot be read or is invalid |

## OTA frame format

After entering the upgrade state, file transfer uses the project's custom OTA frame rather than Modbus register frames.

```text
[AA][Command][Length][Payload...][CRC16_L][CRC16_H][55]
```

- `Length` is the number of payload bytes.
- CRC16 covers bytes from `AA` through the final payload byte. The trailing `55` is not included in the CRC.
- Total frame length is `Length + 6` bytes.
- Multi-byte values in OTA payloads are big-endian unless noted otherwise.

## Enter upgrade mode

### Modbus handshake

The preferred BootLoader entry request is function `0x10` with the fixed payload `12 34 56 78`.

```text
[Slave][10][01][00][00][02][04][12][34][56][78][CRC_L][CRC_H]
```

For slave address `0xC8`:

```text
Host request : C8 10 01 00 00 02 04 12 34 56 78 57 94
Boot reply 1 : C8 10 01 00 00 02 04 12 34 56 78 57 94
Boot reply 2 : C8 10 01 00 00 02 04 87 65 43 21 E5 23
```

This is a project-defined handshake. Its 13-byte replies are not the standard Modbus `0x10` write-multiple-registers response. After the replies, BootLoader enters `UPG_WAIT_FILE_INFO`; send `F4` file information next.

### Custom OTA handshake

The legacy custom OTA request `F1` is also supported while BootLoader is idle.

```text
Host request : AA F1 00 14 70 55
Boot reply 1 : AA F2 04 12 34 56 78 8E EC 55
Boot reply 2 : AA F3 04 87 65 43 21 3D 8A 55
```

After the two replies, send `F4` file information.

## Upgrade commands and replies

| Command | Host payload | Successful BootLoader reply | Notes |
| --- | --- | --- | --- |
| `F1` | None | `F2` then `F3`, both with four-byte fixed flags | Custom OTA entry handshake only |
| `F4` | Firmware length, 4 bytes | Echoes the four-byte length with command `F4` | Erases APP Flash before replying |
| `F5` | Frame ID, firmware data, CRC32 | `A5 A5 ID_H ID_L` with command `F5` | One data frame |
| `F6` | None | `12 34 56 78` with command `F6` | Verifies all written firmware, writes EEPROM state, then resets |
| `F0` | Not accepted as a host command | `FF FF FF FF` with command `F0` | BootLoader notification after a fatal upgrade failure |

### File information: `F4`

Payload is the firmware length in bytes, encoded as a big-endian `uint32_t`.

```text
Request : AA F4 04 [Length_3][Length_2][Length_1][Length_0] [CRC_L][CRC_H] 55
Response: AA F4 04 [Length_3][Length_2][Length_1][Length_0] [CRC_L][CRC_H] 55
```

Example for a 4096-byte image:

```text
AA F4 04 00 00 10 00 F8 7E 55
```

The image must not exceed 896 KiB.

### Firmware data: `F5`

```text
Request
AA F5 Length ID_H ID_L FirmwareData... CRC32_3 CRC32_2 CRC32_1 CRC32_0 CRC16_L CRC16_H 55

Success response
AA F5 04 A5 A5 ID_H ID_L CRC16_L CRC16_H 55
```

- `Length = 2 + firmware-data length + 4`.
- Frame IDs start at `1` and must be strictly consecutive.
- Firmware-data length must be `32` to `128` bytes and a multiple of `32` bytes. The firmware length sent by `F4`, including the final frame, must also be a multiple of `32` bytes.
- The final four payload bytes are the running CRC32 from the first firmware byte through the current `F5` data. The first frame starts with `0xFFFFFFFF`; each later frame uses the previous frame's CRC32 as its initial value. CRC32 uses the STM32 hardware CRC configuration: polynomial `0x04C11DB7`, no input/output inversion, byte input format; the CRC32 value is sent most-significant byte first.

Example success reply for frame ID `1`:

```text
AA F5 04 A5 A5 00 01 07 40 55
```

### Final verification: `F6`

Send `F6` only after the final `F5` acknowledgement. The current implementation does not consume an `F6` payload; use zero payload length.

```text
Request : AA F6 00 16 40 55
Response: AA F6 04 12 34 56 78 8F 68 55
```

On success, BootLoader recalculates the CRC32 over all written firmware bytes and compares it with the final `F5` running CRC32. It then stores the written length and CRC in EEPROM, marks the application ready, and performs a software reset.

## Upgrade sequence

1. Use Modbus `0x03` to confirm the slave address and baud-rate code when communication is already established.
2. Send the Modbus upgrade handshake, or send the custom `F1` handshake.
3. Wait for the expected handshake replies.
4. Send one `F4` file-information frame and wait for its echo reply.
5. Send `F5` frames in ascending frame-ID order, waiting for each acknowledgement.
6. Send `F6` and wait for its reply or for the device reset.

## OTA error replies

For custom OTA frames, errors retain the custom OTA envelope, not the Modbus envelope.

| Command | Payload format | Meaning |
| --- | --- | --- |
| `F9` | Usually `A5 A5 OriginalCommand ErrorCode` | Frame, length, or file-size error; parser errors without a decoded command use `A5 A5 A5 ErrorCode` |
| `FA` | Error payload | Flash erase, write, or CRC error |
| `FB` | `A5 A5 OriginalCommand UpgradeState` | Command received in the wrong upgrade state |
| `FC` | `LastID_H LastID_L F5 ErrorCode` | Duplicate, discontinuous, or out-of-range data frame ID |
| `F0` | `FF FF FF FF` | Upgrade was stopped after a fatal failure; restart from `F4` |

The upgrade-state values are `0` Idle, `1` Wait File Info, `2` Erase Flash, `3` Write Data, `4` Verify CRC, `5` Done, and `6` Error.
