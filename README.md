# CANopenNode_C2000_bootloader
This is a CANopen bootloader for C2000 microcontroller using CANopenNode.

| Tool                     | Version         | Note                    |
|--------------------------|-----------------|-------------------------|
| Code Composer Studio     | v10.2.0         |                         |
| TI Compiler              | v22.6.0.LTS     |                         |
| Object Dictionary Editor | v4.1.2-gff637a7 | Link: [CANopenEditor](https://github.com/CANopenNode/CANopenEditor) |

```mermaid
flowchart TD
    A(Reset) --> B{Check App CRC}
    B -->|valid|C{Check Boot flag}
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
```