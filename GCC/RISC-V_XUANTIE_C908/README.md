# FreeRTOS Port for XuanTie C908/C908V

This directory contains the FreeRTOS port for Alibaba XuanTie C908/C908V RISC-V processors.

This is the minimum configuration to boot FreeRTOS, including FPU, VPU and SMP features.

For more advanced features of this CPU,
please refer to [this page](https://www.xrvm.com/soft-tools/os/RTOS?spm=xrvm.27140568.0.0.78f19b29Tt40XI)
and get the latest RTOS-SDK.

## Files

| File | Description |
|------|-------------|
| `port.c` | FreeRTOS port implementation
| `cpu_task_sw.S` | Context switch assembly code |
| `portmacro.h` | Port-specific macros and definitions |

## Requirements
- GCC:  [link](https://www.xrvm.cn/community/download?versionId=4460156621967921152)

- QEMU: [link](https://www.xrvm.cn/community/download?versionId=4468398114511851520)

## Building and Running

- For build instructions and a complete example, please refer to the `FreeRTOS-Community-Supported-Demos/RISC-V_XUANTIE_C908_GCC/README.md`
