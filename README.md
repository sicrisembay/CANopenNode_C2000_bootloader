# CANopenNode_C2000_bootloader
This is a CANopen bootloader for C2000 microcontroller using CANopenNode.

| Tool                     | Version         | Note                    |
|--------------------------|-----------------|-------------------------|
| Code Composer Studio     | v10.2.0         |                         |
| TI Compiler              | v22.6.0.LTS     |                         |
| Object Dictionary Editor | v4.1.2-gff637a7 | Link: [CANopenEditor](https://github.com/CANopenNode/CANopenEditor) |

## Feature
- Does not use interrupts
- Entirely polled communication
- Executed after device reset or power-up.  Execution is independent on loaded application.
- Minimal hardware initialization.
- Flash programming and verification.
- Overwrite protection of bootloader code area (Flash Sector A).
- Application code verification using CRC.  List of supported algorithms is found in Section 8.9 of TMS320C28xx Assembly Language Tool document. Example of supported CRC algorithms:
    - CRC32_PRIME
    - CRC16_802_15_4
- Backdoor access with waiting window (default is 500ms)
- LSS slave supported.
- LSS slave node Id and bitrate is stored into EEPROM, if available.  Refer to [EEPROM Utilization](#eeprom-utilization)
- see bootloader device [Object Dictionary](c2000_bootloader.md)

## Execution Flow
```mermaid
flowchart TD
    A(Reset) --> B{Check App CRC}
    B -->|valid|J{Check Node ID}
    J -->|valid|C{Check Boot flag}
    C -->|BOOTLOADER_RUN_VAL|D(Run Bootloader)
    C -->|APPLICATION_RUN_VAL|E(Run Application)
    C -->|other value|F[Timer on]
    F -->G{Has expired?}
    G -->|yes|E(Run Application)
    G -->|no|H{SDO received}
    H -->|no|G
    H -->|yes|I[Timer off]
    I -->D
    B -->|invalid|D
    J -->|invalid|D
```

## EEPROM Utilization

| Address       |  Usage           |
|---------------| -----------------|
| 0x0000-0x0003 | <table><thead>Entry Signature</thead> <tbody><tr><td>0x0000-0x0001</td><td>Length of Entry(0x0004)</td></tr><tr><td>0x0002-0x0003</td><td>CRC16 of Entry</td></tr></tbody></table>|
| 0x0040-0x0043 | <table><thead>Entry: LSS Configuration</thead><tbody><tr><td>0x0040-0x0041</td><td>LSS bit rate</td></tr><tr><td>0x0042-0x0043</td><td>LSS node ID</td></tr></tbody></table> |
