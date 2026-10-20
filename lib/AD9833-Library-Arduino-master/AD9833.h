/*
 * AD9833.h
 *
 * Copyright 2016 Bill Williams <wlwilliams1952@gmail.com, github/BillWilliams1952>
 * Patched for STM32 32-bit pin support, double-precision frequency resolution,
 * and high-speed SPI transactions.
 */

#ifndef __AD9833__
#define __AD9833__

#include <Arduino.h>
#include <SPI.h>

#if defined(__AVR__) && defined(FNC_PIN)
	#include "digitalWriteFast.h"
	#define WRITE_FNCPIN(Val) digitalWriteFast2(FNC_PIN, (Val))
#else
	#define WRITE_FNCPIN(Val) digitalWrite(FNCpin, (Val))
#endif

#define pow2_28				268435456.0	// 2^28 used in frequency word calculation
#define BITS_PER_DEG		11.3777777777778	// 4096 / 360

#define RESET_CMD			0x0100		// Reset enabled (also CMD RESET)
#define SLEEP_MODE			0x00C0		// Both DAC and Internal Clock
#define DISABLE_DAC			0x0040
#define	DISABLE_INT_CLK		0x0080

#define PHASE_WRITE_CMD		0xC000		// Setup for Phase write
#define PHASE1_WRITE_REG	0x2000		// Which phase register
#define FREQ0_WRITE_REG		0x4000
#define FREQ1_WRITE_REG		0x8000
#define PHASE1_OUTPUT_REG	0x0400		// Output is based off REG0/REG1
#define FREQ1_OUTPUT_REG	0x0800

typedef enum {
	SINE_WAVE = 0x2000,
	TRIANGLE_WAVE = 0x2002,
	SQUARE_WAVE = 0x2028,
	HALF_SQUARE_WAVE = 0x2020
} WaveformType;

typedef enum { REG0, REG1, SAME_AS_REG0 } Registers;

class AD9833 {

public:

	AD9833 ( uint32_t FNCpin, uint32_t referenceFrequency = 25000000UL );

	// Must be the first command after creating the AD9833 object.
	// If nothing is given (i.e. NULL) default SPI is used.
	void Begin ( SPIClass *spiBus = NULL );

	// Setup and apply a signal.
	void ApplySignal ( WaveformType waveType, Registers freqReg,
		double frequencyInHz,
		Registers phaseReg = SAME_AS_REG0, float phaseInDeg = 0.0 );

	// Resets internal registers to 0.
	void Reset ( void );

	// Update just the frequency in REG0 or REG1 with 64-bit double precision.
	void SetFrequency ( Registers freqReg, double frequency );

	// Increment the selected frequency register by freqIncHz.
	void IncrementFrequency ( Registers freqReg, double freqIncHz );

	// Update just the phase in REG0 or REG1.
	void SetPhase ( Registers phaseReg, float phaseInDeg );

	// Increment the selected phase register by phaseIncDeg.
	void IncrementPhase ( Registers phaseReg, float phaseIncDeg );

	// Set the output waveform for the selected frequency register.
	void SetWaveform ( Registers waveFormReg, WaveformType waveType );

	// Output based on the contents of REG0 or REG1.
	void SetOutputSource ( Registers freqReg, Registers phaseReg = SAME_AS_REG0 );

	// Turn ON / OFF output using the RESET command.
	void EnableOutput ( bool enable );

	// Enable/disable Sleep mode.
	void SleepMode ( bool enable );

	// Enable / Disable DAC.
	void DisableDAC ( bool enable );

	// Enable / Disable Internal Clock.
	void DisableInternalClock ( bool enable );

	// Return actual frequency programmed in register.
	double GetActualProgrammedFrequency ( Registers reg );

	// Return actual phase programmed in register.
	float GetActualProgrammedPhase ( Registers reg );

	// Return frequency resolution.
	float GetResolution ( void );

	// Update reference clock frequency (MCLK) and recalculate active frequency registers
	void SetRefClock ( uint32_t referenceFrequency );

	// Return active reference clock frequency in Hz
	uint32_t GetRefClock ( void );

private:

	void 			WriteRegister ( uint16_t dat );
	void 			WriteControlRegister ( void );
	uint16_t		waveForm0, waveForm1;
#ifndef FNC_PIN
	uint32_t		FNCpin;
#endif
	uint8_t			outputEnabled, DacDisabled, IntClkDisabled;
	uint32_t		refFrequency;
	double			frequency0, frequency1;
	float			phase0, phase1;
	Registers		activeFreq, activePhase;
	SPIClass		*_spiBus;
};

#endif
