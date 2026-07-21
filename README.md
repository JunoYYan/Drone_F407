# Drone_F407

STM32F4 drone firmware project.

## Prerequisites

- USB/IP tools for WSL USB passthrough
- OpenOCD for flashing
- STM32 toolchain (arm-none-eabi-gcc, etc.)

## Compile
make -j$(nproc)

## Flash
sudo openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/Drone_F407.elf verify reset exit"

## USB Setup (WSL)

List available USB devices and bind the ST-Link:

```bash
usbipd list
usbipd bind --busid 4-4
usbipd attach --wsl --busid 4-4