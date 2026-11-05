# FreeRTOS port for Renesas RH850, GCC

Community supported port for RH850 G4 cores with GCC v850-elf
(`-mv850e3v5 -mrh850-abi`). Supported core: G4KH (RH850/U2C).
HW = RH850/U2C HW UM R01UH1018EJ0100.

## Using this port

- Compile `port.c` and `portasm.S` with the kernel sources.
- Add two include paths: this directory and `core_specific/<core>`, for
  example `core_specific/G4KH`. The core is chosen only by that include
  path: `core.h` has the facts of one core (FPU, SYNCE, mask register,
  priority levels), and the other files are the same for every core.
- `core_specific/<core>/core.inc` has assembler macros for the reset-time
  register init; start-up code may include it.
- Float: hard by default. With `-msoft-float` define `configENABLE_FPU 0`.
- `FreeRTOSConfig.h` sets `configKERNEL_INTERRUPT_PRIORITY`,
  `configMAX_SYSCALL_INTERRUPT_PRIORITY` (a PLM value, see below),
  `configISR_STACK_SIZE_WORDS` and `configENABLE_FPU`.
- The vector table sends TRAP0 to `rh850_trap0_entry` and every EIINT
  priority (direct vectors) to `rh850_eiint_entry`.
- The application or board supplies the hooks declared in `portmacro.h`:
  - `vPortSetupTimerInterrupt()`: start the tick; its ISR calls
    `vPortTickHandler()`;
  - `vApplicationIRQHandler(ulEIIC)`: dispatch an EIINT by its EIIC value;
  - for `configUSE_TICKLESS_IDLE 1`: `ulPortTickTimerCount()`,
    `vPortTickTimerSetCompare()`, `vPortTickTimerClearPending()` and
    `ulPortTickTimerCyclesPerTick`.

Test project: `FreeRTOS/Demo/ThirdParty/Community-Supported-Demos/RH850_GCC`
(blinky and the full Template test suite on QEMU, machine `rh850-u2c`).

Tools:
- GCC: https://github.com/linumiz/rh850-gcc-toolchain/releases/tag/v1.0.0
- QEMU: https://github.com/linumiz/qemu-rh850/releases/tag/v1.0.0

## Task frame

- `portframe.h` defines the frame. The task frame is 144 bytes,
  lowest address first:
  - critical nesting count;
  - FPEPC and FPSR;
  - EIPSW and EIPC;
  - r1, r2 and r4-r31;
  - the interrupt mask (PLMR) of the task.
- sp (r3) is kept in the TCB. The frame size is a multiple of 8, so sp
  stays 8-byte aligned (`portBYTE_ALIGNMENT`).
- A new task starts through EIRET with PSW.ID = 0 and no mask; PSW.CU0
  and FPSR.FS are set only when tasks use the FPU. Its return address is
  `prvTaskExitError()`, which asserts: a task function must not return.

## Yield

- `portYIELD()` is TRAP0. The TRAP0 entry saves the whole frame with
  PSW.ID = 1, calls `vTaskSwitchContext()`, and restores the next task
  with EIRET.
- With the FPU in use, SYNCE comes before the trap while PSW.ID = 0. A
  pending imprecise FPU exception is then taken in the task that caused
  it, not after the switch (G4KH SW UM 2.1.2.1 p7; HW 3.2.4.7 p322,
  3.2.6.1 (11) p361).
- From an ISR, `portYIELD_FROM_ISR()` only sets `ulPortYieldPending`. The
  switch happens at the outermost interrupt exit.

## Interrupts and nesting

- Each EIINT priority has its own direct vector. Acceptance sets
  PSW.ID = 1 (HW Table 3.95 p288).
- The entry saves EIPC and EIPSW and reads EIIC before EI, because a
  nested interrupt overwrites them (HW 3.2.4.1 (7) p298). Then it calls
  `vApplicationIRQHandler(EIIC)` with interrupts enabled.
- Nesting depth 0 (the outermost interrupt): the full task frame goes on
  the task stack, then sp moves to the ISR stack
  (`configISR_STACK_SIZE_WORDS`).
- A nested interrupt saves a 96-byte frame on the ISR stack: the return
  state, FPSR and FPEPC, and the registers a GCC call may change.
- ISPR (INTCFG.ISPC = 0) holds off equal and lower priorities until
  EIRET (HW Table 3.44 p237-238).
- At the outermost exit the port switches tasks if one was asked for, and
  EIRET with EP = 0 clears the ISPR bit (HW 3.2.4.3 p302-303).

## Masking: configMAX_SYSCALL_INTERRUPT_PRIORITY

- Critical sections set the core mask register (PLMR on G4), not PSW.ID.
  PSW.ID is used only inside `portasm.S` and around the tickless sleep.
- `configMAX_SYSCALL_INTERRUPT_PRIORITY` is a PLM value n. PLM = n masks
  every EIINT with priority number >= n (HW Table 3.47 p242). Priority
  number 0 is the highest.
- Priorities n to 15 are masked in critical sections and may call the
  FromISR APIs.
- Priorities 0 to n-1 are never masked by the kernel and must not call
  any FreeRTOS API. `vPortValidateInterruptPriority()` checks the lowest
  set ISPR bit and asserts.
- `configKERNEL_INTERRUPT_PRIORITY` (the tick) must be >= n. The default
  is n = 4 with the tick at 15.
- The critical nesting count and PLMR are part of the task frame, so each
  task keeps its own.

## FPU

- The build sets `configENABLE_FPU` from `RH850_FLOAT`: 1 for hard,
  0 for soft (`-msoft-float`). `portUSE_FPU` is 1 when the core has an
  FPU and `configENABLE_FPU` is 1.
- With `portUSE_FPU`, FPSR and FPEPC are saved in every task frame and
  in every nested ISR frame. The FPU uses the general registers and has
  no register file of its own (HW 3.2.3.4 (a) p243), so they are saved
  anyway.
- With `portUSE_FPU` = 0 these slots are never accessed. With PSW.CU0 = 0
  such an access raises UCPOP (HW p185).
- An FPU invalid-operation report (FPEC.FPIVD) may be pending at the
  outermost interrupt exit. The port then keeps the interrupted task, so
  the exception is taken there, and switches at a later exit (HW 3.2.6.1
  (11) p361, FPEC p250). While it is pending, both restore paths keep
  FPEPC and the FPSR cause bits for that exception (HW Table 3.50 p247,
  Table 3.49 p245).

## Tickless idle

Enabled with `configUSE_TICKLESS_IDLE 1` in the `app_config.h` of an
application.

- The board provides a free-running up-counter with a compare (OSTM0 in
  free-run compare mode on U2C, HW 32.4.3 p4342-4343). It supplies
  `ulPortTickTimerCount()`, `vPortTickTimerSetCompare()`,
  `vPortTickTimerClearPending()` and `ulPortTickTimerCyclesPerTick`.
- A compare the counter has already passed would wait for a full wrap
  (HW Table 32.19 p4343). `vPortTickTimerSetCompare()` therefore raises
  the request itself when the counter has reached the compare.
- The port keeps the counter value of the last counted tick. The tick
  ISR counts every whole period since then and sets the next compare.
- `vPortSuppressTicksAndSleep()`:
  1. Sets PSW.ID and checks `eTaskConfirmSleepModeStatus()`.
  2. Sets the compare to the expected wake time and halts. An EIINT
     request ends HALT even with PSW.ID = 1 (G4MH SW UM p80).
  3. After the wake, steps the tick count by the whole periods slept, at
     most the expected time.
  4. Clears the tick request, sets the next compare, and clears PSW.ID.
- The time below one period stays in the counter, so the error does not
  grow. A sleep ended early by another interrupt counts only the whole
  periods slept.
- The longest sleep is 0x7fffffff counter clocks: 107 s at 20 MHz.
- On QEMU with `-icount sleep=on` (and without icount) the wake comes
  about 1 ms of host time late. With `sleep=off` it is exact. The
  tickless tests run in both modes.

## Limits

- errno is one global: picolibc is built without TLS (GCC v850 has only
  emulated TLS), so tasks share it.
- An ISR starts with the FPSR of the task it interrupted (rounding mode,
  flags). The ISR frame restores FPSR on exit, so an ISR that needs its
  own mode must set it.
- No per-task FPU choice. The FPU context is in every task frame or in
  none (`configENABLE_FPU` for the whole build).
- Single core only: no SMP and no cross-core locks.
- The MPU is not used (no `portUSING_MPU_WRAPPERS`).
