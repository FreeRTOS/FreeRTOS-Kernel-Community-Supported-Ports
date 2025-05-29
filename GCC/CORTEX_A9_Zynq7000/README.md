## FreeRTOS 11.1.0 for Zynq7000 platforms

This port supports Symmetric MultiProcessing and has been developed starting from the last available port officially released by Xilinx, based on FreeRTOS 10.6.4. __The port supports both single core and multi core configuration__ as the code to include is managed with some preprocessor macros. However, the SDT flow and the use of XILTIMER are not supported in SMP due to the lack of access to the Unified IDE during the development.

### Usage

#### Steps to create an application
1. Import the Xilinx [embeddedsw](https://github.com/Xilinx/embeddedsw/tree/master) software library into the workspace
    1. Download the code tagged `xilinx_v2024.1`
    2. Add it as a software library under `Xilinx -> Software repositories`
    3. Fix the `uartps` driver bug as described [here](https://adaptivesupport.amd.com/s/question/0D54U00008lF74QSAS/cannot-build-with-xuartps-driver?language=en_US)

    __NOTE:__ New _Hardware Platform_ (HP) projects automatically use the latest BSP and drivers versions while already created HP need to manually change them to the latest available version.

2. Create an _Application Project_ (AP) in Vitis classic with a Standalone OS running on Core #0
3. Include the FreeRTOS source and headers and the port files to the project
    1. Right-click on the AP (not the top-level _System Project_)
    2. Select `Properties -> C/C++ General -> Paths and Symbols`
    3. Add the include paths in the `Includes` tab
    4. Add the source paths in the `Source Location` tab with `Link Folder`
    
    __NOTE:__ Sources need to be properly filtered to avoid the recursive inclusion of undesired subdirectories. To do it, select a linked folder, click on `Edit Filter` and add one or multiple filters.

4. Substitute the linker file automatically generated with the [provided linker file](build/lscript.ld) as it contains an additional memory section to allocate the processor modes stacks of Core #1
5. Use the [provided `FreeRTOSConfig.h`](FreeRTOSConfig.h) template to configure FreeRTOS as it contains some additional configuration macros
6. Write some code, compile and start the application

The port has been developed with __Vitis 2021.1__ so some steps may differ on a different Vitis version. 

### Testing

The code has been tested with a demo still not available on the [FreeRTOS-Community-Supported-Demos](https://github.com/Matth9814/FreeRTOS-Community-Supported-Demos) repository. The demo is similar to the [ZC702 Full_Demo](https://github.com/FreeRTOS/FreeRTOS/tree/main/FreeRTOS/Demo/CORTEX_A9_Zynq_ZC702) but has been slightly modified to support SMP.

### Application debug

An application can be debugged using the classic trace macros or [__Percepio View__](https://forums.freertos.org/t/new-free-trace-tool-from-percepio/22678). Both approaches are supported in the files provided in the [utility](utility/) directory:
* [trace.h](utility/trace.h) should be included inside the _C defines_ section delimited by the preprocessor directive 
```
#ifndef __ASSEMBLER__
    // C code defines
#else
    // Asm defines
```
* [gtimer.h](utility/gtimer.h) implements some macros to work with the Cortex-A9 Global Timer (for Percepio View)

While debugging with the FreeRTOS trace macros does not require any particular effort, using Percepio View requires to patch the trace recoder files. To do so, it is sufficient to substitute the files provided in [this folder](Percepio%20View%204.10.3%20patch) to the original ones.

__NOTE:__ In order to correctly setup the timer used to generate the timestamps in multi core, call the function _vTraceSetupGlobalTimer()_ defined in [trace.c](utility/trace.c) before _xTraceInitialize()_ or _xTraceEnable(TRC_START)_.

### License
Distributed under the MIT License. See `LICENSE` for more information.