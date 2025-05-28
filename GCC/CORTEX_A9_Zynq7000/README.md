## FreeRTOS 11.1.0 for Zynq7000 platforms

This port supports Symmetric MultiProcessing and has been developed starting from the last available port officially released by Xilinx, based on FreeRTOS 10.6.4. The port files support both single core and multi core configuration as the code to include is managed with some preprocessor macros.   

Steps to setup create a project:
* Create an Application project in Vitis classic with a Standalone OS running on Core #0
* Include the FreeRTOS source and headers and the port files to the project
* Substitute the linker file automatically generated with the [provided linker file](build/lscript.ld) as it contains additional memory section for the correct initialization of Core #1
* Use the [provided FreeRTOSConfig.h](FreeRTOSConfig.h) as template to configure FreeRTOS as it contains some additional configuration macros
* Compile the code and start your application

### Testing

The code has been tested with a demo still not available on the [FreeRTOS-Community-Supported-Demos](https://github.com/Matth9814/FreeRTOS-Community-Supported-Demos) repository. 

### Application debug

The application can be debugged using the classic trace macros or [Percepio View](https://forums.freertos.org/t/new-free-trace-tool-from-percepio/22678). Both approaches are supported in the files provided in the [Utility](Utility/) directory:
* [trace.h](Utility/trace.h) should be included inside the _C defines_ section delimited by the preprocessor directive 
```
#ifndef __ASSEMBLER__
    // C code defines
#else
    // Asm defines
```
* [gtimer.h](Utility/gtimer.h) implements some macros to work with the Cortex-A9 Global Timer (for Percepio View)

While debugging with the FreeRTOS trace macros does not require any particular effort, using Percepio View requires to patch the trace recoder files. To do so, it is sufficient to substitute the files provided in [this folder](Utility/Percepio View 4.10.3 patch) to the original ones.

__NOTE:__ In order to correctly setup the Global Timer for timestamping, call the function _vTraceSetupGlobalTimer_ in [trace.c](Utility/trace.c)

