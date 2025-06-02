#include "FreeRTOSConfig.h"

#if ( configNUMBER_OF_CORES > 1)
    .eabi_attribute Tag_ABI_align_preserved, 1
    .text
    .arm
/*  The asm code in this file is interleaved with C code
    so we need to keep the stack 8-byte aligned to be EABI compliant.
    I do not know if i have to do it explicitly since nothing is pushed/pulled on/from the stack.
*/

/* PRIMARY_CORE must be equal to 0 because it guards code
 * that is executed only by CORE0 during the standalone boot */
#define PRIMARY_CORE 0

	/* Functions */
	.global	_boot_core1
	.global vPortLaunchSecondaryCore
	.global vPortGetLock
	.global vPortReleaseLock
	
	.extern vPortRestoreTaskContext 			/* Defined in portASM.S */
	.extern vPortInitCoreInterruptController	/* Defined in portZynq7000.c */
	.extern test_SGI_core1_to_core0
	/* Data */
	.extern ulSecondaryCoreAwake	/* Defined in port.c */
	
	.set CORE1_START_VECTOR,		0xFFFFFFF0
	.set portLOCK_OWNED_BY_CORE0,	0x0
	.set portLOCK_OWNED_BY_CORE1,	0x1
	.set portLOCK_FREE,				0x2
	.set AFF_MASK,					0x3

	/* boot.s labels */
    .globl  MMUTable                /* Defined in translation_table.S (standalone) */
    .global __stack1                /* Defined in lscript.ld */
    .global __irq_stack1
    .global __supervisor_stack1
    .global __abort_stack1
    .global __fiq_stack1
    .global __undef_stack1
    .global _vector_table           /* Defined in port_asm_vectors.S */

    .set TblBase ,		MMUTable
    
    /* workaround for simulation not working when L1 D and I caches,MMU and  L2 cache enabled - DT568997 */
    .if SIM_MODE == 1
    .set CRValMmuCac,	0b00000000000000	/* Disable IDC, and MMU */
    .else
    .set CRValMmuCac,	0b01000000000101	/* Enable IDC, and MMU */
    .endif

    /* Stack Pointer locations for boot code */
    .set Undef_stack,	__undef_stack1
    .set FIQ_stack, 	__fiq_stack1
    .set Abort_stack,	__abort_stack1
    .set SPV_stack, 	__supervisor_stack1
    .set IRQ_stack, 	__irq_stack1
    .set SYS_stack,     __stack1

    .set vector_base,	_vector_table

    .set FPEXC_EN,		0x40000000		/* FPU enable bit, (1 << 30) */

/* this initializes the various processor modes */
.type _boot_core1, %function
_boot_core1:

	/* set VBAR to the _vector_table address in linker script */
	ldr	r0, =vector_base
	mcr	p15, 0, r0, c12, c0, 0

/* Done by the primary core */
/*invalidate scu*/
#if PRIMARY_CORE == 1
	ldr	r7, =0xf8f0000c
	ldr	r6, =0xffff
	str	r6, [r7]
#endif

    /* NOTE: the following two blocks of code, aside from Dcache invalidation,
     * should have been executed already after CORE1 wakeup from WFE.
	 * They have been left here in case the WFE loop of CORE1 was modified. */
	
	/* Invalidate caches and TLBs */
    mov	r0,#0				        /* r0 = 0  */
	mcr	p15, 0, r0, c8, c7, 0		/* invalidate TLBs */
	mcr	p15, 0, r0, c7, c5, 0		/* invalidate icache */
	mcr	p15, 0, r0, c7, c5, 6		/* Invalidate branch predictor array */
	bl	invalidate_dcache		    /* invalidate dcache */

	/* Disable MMU, if enabled */
	mrc	p15, 0, r0, c1, c0, 0		/* read CP15 register 1 */
	bic	r0, r0, #0x1			    /* clear bit 0 */
	mcr	p15, 0, r0, c1, c0, 0		/* write value back */

#ifdef SHAREABLE_DDR
	/* Mark the entire DDR memory as shareable */
	ldr	r3, =0x3ff			/* 1024 entries to cover 1G DDR */
    ldr	r0, =TblBase		/* MMU Table address in memory */
    ldr	r2, =0x15de6		/* S=b1 TEX=b101 AP=b11, Domain=b1111, C=b0, B=b1 */
shareable_loop:
	str	r2, [r0]			/* write the entry to MMU table */
	add	r0, r0, #0x4		/* next entry in the table */
	add	r2, r2, #0x100000	/* next section */
	subs	r3, r3, #1
	bge	shareable_loop		/* loop till 1G is covered */
#endif

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the irq stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x12		    /* IRQ mode */
	msr	cpsr, r2
	ldr	r13,=IRQ_stack			/* IRQ stack pointer */
	bic r2, r2, #(0x1 << 9)    	/* Set EE bit to little-endian */
	msr spsr_fsxc,r2

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the supervisor stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x13			/* supervisor mode */
	msr	cpsr, r2
	ldr	r13,=SPV_stack			/* Supervisor stack pointer */
	bic r2, r2, #(0x1 << 9)     /* Set EE bit to little-endian */
	msr spsr_fsxc,r2

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the Abort  stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x17			/* Abort mode */
	msr	cpsr, r2
	ldr	r13,=Abort_stack		/* Abort stack pointer */
	bic r2, r2, #(0x1 << 9)     /* Set EE bit to little-endian */
	msr spsr_fsxc,r2

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the FIQ stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x11			/* FIQ mode */
	msr	cpsr, r2
	ldr	r13,=FIQ_stack			/* FIQ stack pointer */
	bic r2, r2, #(0x1 << 9)     /* Set EE bit to little-endian */
	msr spsr_fsxc,r2

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the Undefine stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x1b			/* Undefine mode */
	msr	cpsr, r2
	ldr	r13,=Undef_stack		/* Undefine stack pointer */
	bic r2, r2, #(0x1 << 9)     /* Set EE bit to little-endian */
	msr spsr_fsxc,r2

	mrs	r0, cpsr			/* get the current PSR */
	mvn	r1, #0x1f			/* set up the system stack pointer */
	and	r2, r1, r0
	orr	r2, r2, #0x1F			/* SYS mode */
	msr	cpsr, r2
	ldr	r13,=SYS_stack			/* SYS stack pointer */

/* Done by the primary core */
#if PRIMARY_CORE == 1
	/*set scu enable bit in scu*/
	ldr	r7, =0xf8f00000
	ldr	r0, [r7]
	orr	r0, r0, #0x1
	str	r0, [r7]
#endif

	/* enable MMU and cache */

	ldr	r0,=TblBase			    /* Load MMU translation table base */
	orr	r0, r0, #0x5B			/* Outer-cacheable, WB */
	mcr	15, 0, r0, c2, c0, 0	/* TTB0 */

	mvn	r0,#0				    /* Load MMU domains -- all ones=manager */
	mcr	p15,0,r0,c3,c0,0

	/* Enable mmu, icahce and dcache */
	ldr	r0,=CRValMmuCac
	mcr	p15,0,r0,c1,c0,0		/* Enable cache and MMU */
	dsb					        /* dsb	allow the MMU to start up */
	isb					        /* isb	flush prefetch buffer */

	/* Write to ACTLR */
	mrc	p15, 0, r0, c1, c0, 1		/* Read ACTLR*/
	orr	r0, r0, #(0x01 << 6)		/* set SMP bit */
	orr	r0, r0, #(0x01 )		    /* Cache/TLB maintenance broadcast */
	mcr	p15, 0, r0, c1, c0, 1		/* Write ACTLR*/

/* Done by the primary core */
/* Invalidate L2 Cache and enable L2 Cache*/
#if PRIMARY_CORE == 1
	ldr	r0,=L2CCCrtl			/* Load L2CC base address base + control register */
	mov	r1, #0				/* force the disable bit */
	str	r1, [r0]			/* disable the L2 Caches */

	ldr	r0,=L2CCAuxCrtl			/* Load L2CC base address base + Aux control register */
	ldr	r1,[r0]				/* read the register */
	ldr	r2,=L2CCAuxControl		/* set the default bits */
	orr	r1,r1,r2
	str	r1, [r0]			/* store the Aux Control Register */

	ldr	r0,=L2CCTAGLatReg		/* Load L2CC base address base + TAG Latency address */
	ldr	r1,=L2CCTAGLatency		/* set the latencies for the TAG*/
	str	r1, [r0]			/* store the TAG Latency register Register */

	ldr	r0,=L2CCDataLatReg		/* Load L2CC base address base + Data Latency address */
	ldr	r1,=L2CCDataLatency		/* set the latencies for the Data*/
	str	r1, [r0]			/* store the Data Latency register Register */

	ldr	r0,=L2CCWay			/* Load L2CC base address base + way register*/
	ldr	r2, =0xFFFF
	str	r2, [r0]			/* force invalidate */

	ldr	r0,=L2CCSync			/* need to poll 0x730, PSS_L2CC_CACHE_SYNC_OFFSET */
						/* Load L2CC base address base + sync register*/
	/* poll for completion */
Sync:	ldr	r1, [r0]
	cmp	r1, #0
	bne	Sync

	ldr	r0,=L2CCIntRaw			/* clear pending interrupts */
	ldr	r1,[r0]
	ldr	r0,=L2CCIntClear
	str	r1,[r0]

	ldr	r0,=SLCRUnlockReg		/* Load SLCR base address base + unlock register */
	ldr	r1,=SLCRUnlockKey	    	/* set unlock key */
	str	r1, [r0]		    	/* Unlock SLCR */

	ldr	r0,=SLCRL2cRamReg		/* Load SLCR base address base + l2c Ram Control register */
	ldr	r1,=SLCRL2cRamConfig        	/* set the configuration value */
	str	r1, [r0]	        	/* store the L2c Ram Control Register */

	ldr	r0,=SLCRlockReg         	/* Load SLCR base address base + lock register */
	ldr	r1,=SLCRlockKey	        	/* set lock key */
	str	r1, [r0]	        	/* lock SLCR */

	ldr	r0,=L2CCCrtl			/* Load L2CC base address base + control register */
	ldr	r1,[r0]				/* read the register */
	mov	r2, #L2CCControl		/* set the enable bit */
	orr	r1,r1,r2
	str	r1, [r0]			/* enable the L2 Caches */
#endif

	mov	r0, r0
	mrc	p15, 0, r1, c1, c0, 2		/* read cp access control register (CACR) into r1 */
	orr	r1, r1, #(0xf << 20)		/* enable full access for p10 & p11 */
	mcr	p15, 0, r1, c1, c0, 2		/* write back into CACR */

	/* enable vfp */
	fmrx	r1, FPEXC			/* read the exception register */
	orr	r1,r1, #FPEXC_EN		/* set VFP enable bit, leave the others in orig state */
	fmxr	FPEXC, r1			/* write back the exception register */

	mrc	p15,0,r0,c1,c0,0		/* flow prediction enable */
	orr	r0, r0, #(0x01 << 11)	/* #0x8000 */
	mcr	p15,0,r0,c1,c0,0

	mrc	p15,0,r0,c1,c0,1		/* read Auxiliary Control Register */
	orr	r0, r0, #(0x1 << 2)		/* enable Dside prefetch */
	orr	r0, r0, #(0x1 << 1)		/* enable L2 Prefetch hint */
	mcr	p15,0,r0,c1,c0,1		/* write Auxiliary Control Register */

	mrs	r0, cpsr			    /* get the current PSR */
	bic	r0, r0, #0x100			/* enable asynchronous abort exception */
	msr	cpsr_xsf, r0

	dsb
	isb
	
	/* Initialized the IC CPU interface of CORE1 */
	ldr r3, vPortInitCoreInterruptControllerConst
	blx r3

	/* Notify CORE0 that CORE1 is awake */
	mov	r1, #1
	ldr r2, ulSecondaryCoreAwakeConst
	str r1, [r2]
	dsb

	/* Start first task on CORE1 */
	ldr r0, vPortRestoreTaskContextConst
    bx	r0
	andeq r0, r0 ,r0		/* No op */

.Ldone:	b	.Ldone				/* Paranoia: we should never get here */

/*
 *************************************************************************
 *
 * invalidate_dcache - invalidate the entire d-cache by set/way
 *
 * Note: for Cortex-A9, there is no cp instruction for invalidating
 * the whole D-cache. Need to invalidate each line.
 *
 *************************************************************************
 */
invalidate_dcache:
	mrc	p15, 1, r0, c0, c0, 1		/* read CLIDR */
	ands	r3, r0, #0x7000000
	mov	r3, r3, lsr #23			/* cache level value (naturally aligned) */
	beq	finished
	mov	r10, #0				/* start with level 0 */
loop1:
	add	r2, r10, r10, lsr #1		/* work out 3xcachelevel */
	mov	r1, r0, lsr r2			/* bottom 3 bits are the Cache type for this level */
	and	r1, r1, #7			/* get those 3 bits alone */
	cmp	r1, #2
	blt	skip				/* no cache or only instruction cache at this level */
	mcr	p15, 2, r10, c0, c0, 0		/* write the Cache Size selection register */
	isb					/* isb to sync the change to the CacheSizeID reg */
	mrc	p15, 1, r1, c0, c0, 0		/* reads current Cache Size ID register */
	and	r2, r1, #7			/* extract the line length field */
	add	r2, r2, #4			/* add 4 for the line length offset (log2 16 bytes) */
	ldr	r4, =0x3ff
	ands	r4, r4, r1, lsr #3		/* r4 is the max number on the way size (right aligned) */
	clz	r5, r4				/* r5 is the bit position of the way size increment */
	ldr	r7, =0x7fff
	ands	r7, r7, r1, lsr #13		/* r7 is the max number of the index size (right aligned) */
loop2:
	mov	r9, r4				/* r9 working copy of the max way size (right aligned) */
loop3:
	orr	r11, r10, r9, lsl r5		/* factor in the way number and cache number into r11 */
	orr	r11, r11, r7, lsl r2		/* factor in the index number */
	mcr	p15, 0, r11, c7, c6, 2		/* invalidate by set/way */
	subs	r9, r9, #1			/* decrement the way number */
	bge	loop3
	subs	r7, r7, #1			/* decrement the index */
	bge	loop2
skip:
	add	r10, r10, #2			/* increment the cache number */
	cmp	r3, r10
	bgt	loop1

finished:
	mov	r10, #0				/* switch back to cache level 0 */
	mcr	p15, 2, r10, c0, c0, 0		/* select current cache level in cssr */
	dsb
	isb

	bx	lr

/*
 * 	Function to launch the secondary core from
 * 	the primary core
 */
.type vPortLaunchSecondaryCore, %function
vPortLaunchSecondaryCore:
	/* Write the start address of CORE1 */
	mov	r0, #CORE1_START_VECTOR
	ldr r1, =_boot_core1
	str r1, [r0]
	dsb
	mcr p15, 0, r0, c7, c10, 1 /* Clean address to PoC */
	dsb

	/* Wake-up CORE1 and wait for it to be ready */
	ldr r2, ulSecondaryCoreAwakeConst
pollForCore:
	sev
	ldr r1, [r2]
	cmp r1, #0
	beq pollForCore

	bx lr

/* 
 * Function to claim a recursive spinlock
 */
.type vPortGetLock, %function

#if ( configGET_LOCK_VERSION == 0 )
vPortGetLock:
    /* 	r0: lock address */
	/*  r1: core ID */

	/* Save R4. */
	push { r4 }

    /* Try to claim the lock */
    ldr		r4, =checkOwnership
lockLoop:
    ldrex 	r2, [r0]
    cmp		r2, #portLOCK_FREE 	/* Check if the lock is free */
    bxne	r4
    
    strex 	r3, r1, [r0]		/* Try to claim the lock */
    cmp 	r3, #0				/* Check if the operation updated the memory (r2=0) */
    bne 	lockLoop			/* If not loop until one claimer gets the lock*/
    
    /* Ensures that all subsequent accesses are observed after the
    gaining of the lock is observed. */
    dmb

#if ( configTEST_RECURSIVE_SPINLOCK == 1 )
    ldr		r2, [r0, #4]
    cmp		r2, #0				/* Recursive count == 0 when first acquired */
    bne		.	
#endif

    b		1f

checkOwnership:
    cmp		r2, r1
    ldrne	r4, =lockLoop		/* Continue to loop until the lock is free */
    bxne	r4	
    
#if ( configTEST_RECURSIVE_SPINLOCK == 1 )
    ldr		r2, [r0, #4]
    cmp		r2, #255			/* Recursive count < 255 when reclaimed */
    bhs		.	
#endif

1:
    /* From now on shared data are protected from 
    concurrent accesses. */
    
    /* Increase claim count */
    ldr		r2, [r0, #4]
    add		r2, r2, #1
    str		r2, [r0, #4]
    
	pop { r4 }
	
    bx lr

#elif ( configGET_LOCK_VERSION == 1 )
vPortGetLock:
	/* 	r0: lock address */
	/*	r1: core ID */

	/* Try to claim the lock */
tryLockLoop:
	ldrex 	r2, [r0]
	cmp		r2, #portLOCK_FREE 	/* Check if the lock is free */
	bne		checkOwnership

	strex 	r3, r1, [r0]		/* Try to claim the lock */
	cmp 	r3, #0				/* Check if the operation updated the memory (r2=0) */
	bne 	tryLockLoop		/* If not loop until one claimer gets the lock */
	
	/* Ensures that all subsequent accesses are observed after the
	gaining of the lock is observed. */
	dmb

#if ( configTEST_RECURSIVE_SPINLOCK == 1 )
	ldr		r2, [r0, #4]
	cmp		r2, #0				/* Recursive count == 0 when first acquired */
	bne		.	
#endif

	b		1f

checkOwnership:
	cmp		r2, r1	
	beq		1f			/* If the core owns the lock just reclaim it */

	/* Continue to loop until the lock is free */
waitLockLoop:
	ldrex 	r2, [r0]
	cmp		r2, #portLOCK_FREE
	strexeq r3, r1, [r0]	
	cmpeq 	r3, #0
	bne 	waitLockLoop
	
	/* Ensures that all subsequent accesses are observed after the
	gaining of the lock is observed. */
	dmb	

1:
	/* From now on shared data are protected from 
	concurrent accesses. */
	
#if ( configTEST_RECURSIVE_SPINLOCK == 1 )
	ldr		r2, [r0, #4]
	cmp		r2, #255			/* Recursive count < 255 when reclaimed */
	bhs		.	
#endif

	/* Increase claim count */
	ldr		r2, [r0, #4]
	add		r2, r2, #1
	str		r2, [r0, #4]

	bx lr

#else
	#error "The spinlock version can be either 0 or 1"
#endif

/* 
 * Function to release a recursive spinlock
 */
.type vPortReleaseLock, %function
vPortReleaseLock:
	/* 	r0: lock address */

#if ( configTEST_RECURSIVE_SPINLOCK == 1 )
	/* Get core ID */
	mrc     p15, 0, r1, c0, c0, 5
    and     r1, r1, #AFF_MASK

	ldr		r2, [r0]
	cmp		r1, r2			/* Core ID == Lock owner ID */
	bne		.				/* Error */

	ldr		r2, [r0, #4]	
	cmp		r2, #0			/* Recursion count != 0 */
	beq		.				/* Error */
#endif

	/* Decrease the recursion count */
	ldr		r2, [r0, #4]
	sub		r2, r2, #1
	str		r2, [r0, #4]	

	cmp		r2, #0
	
	/* Allow all the previous memory operations to be observed
	before releasing the lock */
	dmb

	/* Release the lock if all the claims have
	been released */
	moveq	r1, #portLOCK_FREE
	streq	r1, [r0]

	bx lr

vPortRestoreTaskContextConst: 			.word 	vPortRestoreTaskContext
vPortInitCoreInterruptControllerConst: 	.word 	vPortInitCoreInterruptController
ulSecondaryCoreAwakeConst:				.word	ulSecondaryCoreAwake

.end

#endif /* #if ( configNUMBER_OF_CORES > 1) */
