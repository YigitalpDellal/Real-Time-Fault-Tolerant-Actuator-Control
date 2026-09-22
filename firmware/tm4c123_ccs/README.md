# TM4C123 Code Composer Studio Project

This directory contains the Code Composer Studio project metadata for the TM4C123GXL firmware used by the dual-axis actuator controller.

The CCS project links the canonical firmware source from `../../src/tm4c123/main.c`, so the repository does not maintain two different copies of the MCU application.

## Target

- Board: EK-TM4C123GXL LaunchPad
- MCU: TM4C123GH6PM
- Debug probe: Stellaris ICDI
- UART: UART1 on PB0/PB1, 115200 8-N-1
- PWM: PB6/M0PWM0 for PAN, PB7/M0PWM1 for TILT
- System clock: 80 MHz
- TivaWare DriverLib

## Import into CCS

1. Install Code Composer Studio and TivaWare.
2. In CCS, use **File > Import > Code Composer Studio > CCS Projects**.
3. Select this `firmware/tm4c123_ccs` directory.
4. The project defaults to `C:/ti/TivaWare_C_Series-2.2.0.295`.
5. If TivaWare is installed elsewhere, update `SW_ROOT` in both **Project Properties > Resource > Linked Resources > Path Variables** and **Project Properties > Build > Variables**.
6. Confirm that DriverLib resolves to `${SW_ROOT}/driverlib/ccs/Debug/driverlib.lib`.
7. Build the **Debug** configuration and flash with the Stellaris ICDI connection.

## Included project metadata

- `.project` — Eclipse/CCS project definition
- `.cproject` — compiler and linker configuration
- `.ccsproject` — TI device/toolchain metadata
- `tm4c123gh6pm.cmd` — TM4C123GH6PM memory map and section placement
- `targetConfigs/Tiva TM4C123GH6PM.ccxml` — LaunchPad debug target configuration

The standard TivaWare `startup_ccs.c` file is linked from the installed TivaWare tree rather than duplicated here. The application source remains `src/tm4c123/main.c`.
