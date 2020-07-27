/*
 * Copyright (c) 2014-2015 Wind River Systems, Inc.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module for fsl_frdm_k64f platform
 *
 * This module provides routines to initialize and support board-level
 * hardware for the fsl_frdm_k64f platform.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <soc.h>
#include <drivers/uart.h>
#include <fsl_common.h>
#include <fsl_clock.h>
#include <fsl_rtc.h> // FLL / RTC setup
#include <arch/cpu.h>
#include <arch/arm/aarch32/cortex_m/cmsis.h>

#define PLLFLLSEL_MCGFLLCLK	(0)
#define PLLFLLSEL_MCGPLLCLK	(1)
#define PLLFLLSEL_IRC48MHZ	(3)

#define ER32KSEL_OSC32KCLK	(0)
#define ER32KSEL_RTC		(2)
#define ER32KSEL_LPO1KHZ	(3)

#define TIMESRC_OSCERCLK        (2)

#define RUNM_HSRUN              (3)

#if CONFIG_OSC_XTAL0_FREQ != 0
#error "Set OSC_XTAL0_FREQ to zero in menuconfig!"
#endif
// #define CONFIG_OSC_XTAL0_FREQ (0) // FLL
#define CONFIG_XTAL32K_FREQ (32768)
#define CONFIG_USE_FLL (1)
#if defined(CONFIG_USE_FLL) && defined(CONFIG_K6X_HSRUN) // we can't support HSRUN from FLL
#undef CONFIG_K6X_HSRUN
#endif

#if !defined(CONFIG_USE_FLL)
static const osc_config_t oscConfig = {
	.freq = CONFIG_OSC_XTAL0_FREQ,
	.capLoad = 0,

#if defined(CONFIG_OSC_EXTERNAL)
	.workMode = kOSC_ModeExt,
#elif defined(CONFIG_OSC_LOW_POWER)
	.workMode = kOSC_ModeOscLowPower,
#elif defined(CONFIG_OSC_HIGH_GAIN)
	.workMode = kOSC_ModeOscHighGain,
#else
#error "An oscillator mode must be defined"
#endif

	.oscerConfig = {
		.enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && \
	FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
		.erclkDiv = 0U,
#endif
	},
};
static const mcg_pll_config_t pll0Config = {
	.enableMode = 0U,
	.prdiv = CONFIG_MCG_PRDIV0,
	.vdiv = CONFIG_MCG_VDIV0,
};
#endif // !CONFIG_USE_FLL

#define SIM_PLLFLLSEL_IRC48MCLK_CLK                       3U  /*!< PLLFLL select: IRC48MCLK clock */
#define SIM_OSC32KSEL_RTC32KCLK_CLK                       2U  /*!< OSC32KSEL select: RTC32KCLK clock (32.768kHz) */

static const sim_clock_config_t simConfig = {
	// .pllFllSel = PLLFLLSEL_MCGPLLCLK, /* PLLFLLSEL select PLL. */
	// we'll need this for USB
	.pllFllSel = SIM_PLLFLLSEL_IRC48MCLK_CLK, //
	// .er32kSrc = ER32KSEL_RTC,         /* ERCLK32K selection, use RTC. */
	.er32kSrc = SIM_OSC32KSEL_RTC32KCLK_CLK, // use RTC32KCLK
	// TODO: review the divisor settings, but yes, we want them
	//       .clkdiv1 = 0x1730000U,                    /* SIM_CLKDIV1 - OUTDIV1: /1, OUTDIV2: /2, OUTDIV3: /8, OUTDIV4: /4 */
	.clkdiv1 = SIM_CLKDIV1_OUTDIV1(CONFIG_K6X_CORE_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV2(CONFIG_K6X_BUS_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV3(CONFIG_K6X_FLEXBUS_CLOCK_DIVIDER - 1) |
		   SIM_CLKDIV1_OUTDIV4(CONFIG_K6X_FLASH_CLOCK_DIVIDER - 1),
};

#ifdef CONFIG_USE_FLL
// FLL support code, stolen from mxuxpresso clock setup tool, which generates
// KSDK2-like setup code
// TODO: Where to set DRX32?

static void CLOCK_CONFIG_FllStableDelay(void)
{
    uint32_t i = 30000U;
    while (i--)
    {
        __NOP();
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : CLOCK_CONFIG_SetRtcClock
 * Description   : This function is used to configuring RTC clock including
 * enabling RTC oscillator.
 * Param capLoad : RTC oscillator capacity load
 * Param enableOutPeriph : Enable (1U)/Disable (0U) clock to peripherals
 *
 *END**************************************************************************/
static void CLOCK_CONFIG_SetRtcClock(uint32_t capLoadBits, uint8_t enableOutPeriph)
{
  /* RTC clock gate enable */
  CLOCK_EnableClock(kCLOCK_Rtc0);
  if ((RTC->CR & RTC_CR_OSCE_MASK) == 0u) { /* Only if the Rtc oscillator is not already enabled */
    /* Set the specified capacitor configuration for the RTC oscillator */
    RTC_SetOscCapLoad(RTC, capLoadBits);
    /* Enable the RTC 32KHz oscillator */
    RTC->CR |= RTC_CR_OSCE_MASK;
  }
  /* Output to other peripherals */
  if (enableOutPeriph) {
    RTC->CR &= ~RTC_CR_CLKO_MASK;
  }
  else {
    RTC->CR |= RTC_CR_CLKO_MASK;
  }
  /* Set the XTAL32/RTC_CLKIN frequency based on board setting. */
  CLOCK_SetXtal32Freq(CONFIG_XTAL32K_FREQ);
  /* Set RTC_TSR if there is fault value in RTC */
  if (RTC->SR & RTC_SR_TIF_MASK) {
    RTC -> TSR = RTC -> TSR;
  }
  /* RTC clock gate disable */
  CLOCK_DisableClock(kCLOCK_Rtc0);
}
#endif // CONFIG_USE_FLL

/**
 *
 * @brief Initialize the system clock
 *
 * This routine will configure the multipurpose clock generator (MCG) to
 * set up the system clock.
 * The MCG has nine possible modes, including Stop mode.  This routine assumes
 * that the current MCG mode is FLL Engaged Internal (FEI), as from reset.
 * It transitions through the FLL Bypassed External (FBE) and
 * PLL Bypassed External (PBE) modes to get to the desired
 * PLL Engaged External (PEE) mode and generate the maximum 120 MHz system
 * clock.
 *
 * @return N/A
 *
 */
static ALWAYS_INLINE void clock_init(void)
{
	CLOCK_SetSimSafeDivs();

#if !defined(CONFIG_USE_FLL)
	CLOCK_InitOsc0(&oscConfig);
	CLOCK_SetXtal0Freq(CONFIG_OSC_XTAL0_FREQ);

	CLOCK_BootToPeeMode(kMCG_OscselOsc, kMCG_PllClkSelPll0, &pll0Config);

	CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow,
				      CONFIG_MCG_FCRDIV);

	CLOCK_SetSimConfig(&simConfig);

#if CONFIG_ETH_MCUX
	CLOCK_SetEnetTime0Clock(TIMESRC_OSCERCLK);
#endif
#if CONFIG_ETH_MCUX_RMII_EXT_CLK
	CLOCK_SetRmii0Clock(1);
#endif
#if CONFIG_USB_KINETIS
	CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcPll0,
				DT_PROP(DT_PATH(cpus, cpu_0), clock_frequency));
#endif
#else // CONFIG_USE_FLL

	/* Configure RTC clock including enabling RTC oscillator. */
	// 0pf extra caps, enable 32K to other peripherals out of generator
	CLOCK_CONFIG_SetRtcClock(0, 1);
	/* Configure the Internal Reference clock (MCGIRCLK). */
	// CLOCK_SetInternalRefClkConfig(mcgConfig_BOARD_BootClockRUN.irclkEnableMode,
	// 															mcgConfig_BOARD_BootClockRUN.ircs,
	// 															mcgConfig_BOARD_BootClockRUN.fcrdiv);
	// TODO: Do we need the IRCLK even? we now select slow, but could it be
	//       disabled?
	CLOCK_SetInternalRefClkConfig(kMCG_IrclkEnable, kMCG_IrcSlow,
				      CONFIG_MCG_FCRDIV);
  /* Set MCG to FEE mode. */
  CLOCK_BootToFeeMode(/*mcgConfig_BOARD_BootClockRUN.oscsel*/ kMCG_OscselRtc, // rtcosc as the source to FLL
                      /*mcgConfig_BOARD_BootClockRUN.frdiv*/ 0 /* div by 1 */,
                      /*mcgConfig_BOARD_BootClockRUN.dmx32*/ kMCG_Dmx32Fine /* since exactly 32768 */,
                      /*mcgConfig_BOARD_BootClockRUN.drs*/ kMCG_DrsHigh /* 96M target range */,
                      CLOCK_CONFIG_FllStableDelay);
  /* Set the clock configuration in SIM module. */
  CLOCK_SetSimConfig(&simConfig);

// #define BOARD_BOOTCLOCKRUN_CORE_CLOCK              95977472U  /*!< Core clock frequency: 95977472Hz */

  // /* Set SystemCoreClock variable. */
  // This is an internal variable for clock_config it seems, or common CMSIS
  // SystemCoreClock = BOARD_BOOTCLOCKRUN_CORE_CLOCK;

  /* Set RTC_CLKOUT source. */
  // 0: 1Hz, 1: 32768Hz
  CLOCK_SetRtcClkOutClock(1);
  /* Enable USB FS clock. Note that it's never disabled anywhere. */
  CLOCK_EnableUsbfs0Clock(kCLOCK_UsbSrcIrc48M, 48000000U);
  /* Set CLKOUT source. (FlexBusClk = 0) */
  CLOCK_SetClkOutClock(0);
  /* Set debug trace clock source. */
  // CLOCK_SetTraceClock(SIM_TRACE_CLK_SEL_CORE_SYSTEM_CLK);

  // TODO: getting SD to work will require switching the USB clock to something
  //       else when there's no USB.
  //       There's no SDHC support for kinetis anyway. except there's an NXP USDHC driver
  //       for imx stuff. does it look the same?
#endif // CONFIG_USE_FLL
}

/**
 *
 * @brief Perform basic hardware initialization
 *
 * Initialize the interrupt controller device drivers.
 * Also initialize the timer device driver, if required.
 *
 * @return 0
 */

static int k6x_init(struct device *arg)
{
	ARG_UNUSED(arg);

	unsigned int oldLevel; /* old interrupt lock level */
#if !defined(CONFIG_ARM_MPU)
	uint32_t temp_reg;
#endif /* !CONFIG_ARM_MPU */

	/* disable interrupts */
	oldLevel = irq_lock();

	/* release I/O power hold to allow normal run state */
	PMC->REGSC |= PMC_REGSC_ACKISO_MASK;

#ifdef CONFIG_TEMP_KINETIS
	/* enable bandgap buffer */
	PMC->REGSC |= PMC_REGSC_BGBE_MASK;
#endif /* CONFIG_TEMP_KINETIS */

#if !defined(CONFIG_ARM_MPU)
	/*
	 * Disable memory protection and clear slave port errors.
	 * Note that the K64F does not implement the optional ARMv7-M memory
	 * protection unit (MPU), specified by the architecture (PMSAv7), in the
	 * Cortex-M4 core.  Instead, the processor includes its own MPU module.
	 */
	temp_reg = SYSMPU->CESR;
	temp_reg &= ~SYSMPU_CESR_VLD_MASK;
	temp_reg |= SYSMPU_CESR_SPERR_MASK;
	SYSMPU->CESR = temp_reg;
#endif /* !CONFIG_ARM_MPU */

#if 0
#ifdef CONFIG_K6X_HSRUN
	/* Switch to HSRUN mode */
	SMC->PMPROT |= SMC_PMPROT_AHSRUN_MASK;
	SMC->PMCTRL = (SMC->PMCTRL & ~SMC_PMCTRL_RUNM_MASK) |
		SMC_PMCTRL_RUNM(RUNM_HSRUN);
#endif
#endif
	/* Initialize PLL/system clock up to 180 MHz */
	clock_init();

	/*
	 * install default handler that simply resets the CPU
	 * if configured in the kernel, NOP otherwise
	 */
	NMI_INIT();

	/* restore interrupt state */
	irq_unlock(oldLevel);
	return 0;
}

SYS_INIT(k6x_init, PRE_KERNEL_1, 0);
