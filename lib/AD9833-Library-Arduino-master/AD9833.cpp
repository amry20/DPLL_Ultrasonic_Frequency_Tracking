/*
 * AD9833.cpp
 * 
 * Copyright 2016 Bill Williams <wlwilliams1952@gmail.com, github/BillWilliams1952>
 * Patched for STM32, 64-bit double precision frequency calculations,
 * and high-speed SPI transactions. Square wave full-rate: 0x2028 (DIV2=1, direct MSB).
 */

#include "AD9833.h"
#include <math.h>

/*
 * Create an AD9833 object
 */
AD9833 :: AD9833 ( uint32_t FNCpin, uint32_t referenceFrequency ) {
#ifndef FNC_PIN
	this->FNCpin = FNCpin;
#endif
	refFrequency = referenceFrequency;
	
	// Setup defaults
	DacDisabled = false;
	IntClkDisabled = false;
	outputEnabled = false;
	waveForm0 = waveForm1 = SINE_WAVE;
	frequency0 = frequency1 = 1000.0;		// 1 KHz sine wave to start
	phase0 = phase1 = 0.0f;					// 0 phase
	activeFreq = REG0;
	activePhase = REG0;
	_spiBus = &SPI;
}

/*
 * This MUST be the first command after declaring the AD9833 object
 * Start SPI and place the AD9833 in the RESET state
 */
void AD9833 :: Begin ( SPIClass* spiBus ) {
#ifdef FNC_PIN
	pinMode(FNC_PIN, OUTPUT);
#else
	pinMode(FNCpin, OUTPUT);
#endif
	WRITE_FNCPIN(HIGH);

    if ( spiBus != NULL ) {
        this->_spiBus = spiBus;
    } else {
        this->_spiBus = &SPI;
    }
    _spiBus->begin();
	delay(10);
	Reset();	// Hold in RESET until first WriteRegister command
}

/*
 * Setup and apply a signal. phaseInDeg defaults to 0.0 if not supplied.
 * phaseReg defaults to value of freqReg if not supplied.
 */
void AD9833 :: ApplySignal ( WaveformType waveType,
		Registers freqReg, double frequencyInHz,
		Registers phaseReg, float phaseInDeg ) {
	SetFrequency ( freqReg, frequencyInHz );
	SetPhase ( phaseReg, phaseInDeg );
	SetWaveform ( freqReg, waveType );
	SetOutputSource ( freqReg, phaseReg );
}

void AD9833 :: Reset ( void ) {
	WriteRegister(RESET_CMD);
	delay(5);
}

/*
 *  Set the specified frequency register with the frequency (in Hz)
 *  Using 64-bit double precision to maintain sub-Hz (e.g. 0.0075 Hz) step resolution.
 */
void AD9833 :: SetFrequency ( Registers freqReg, double frequency ) {
	double maxFreq = (double)refFrequency / 2.0;
	if ( frequency > maxFreq )
		frequency = maxFreq;
	if ( frequency < 0.0 )
		frequency = 0.0;
	
	if ( freqReg == REG0 ) frequency0 = frequency;
	else frequency1 = frequency;
	
	uint32_t freqWord = (uint32_t)llround((frequency * 268435456.0) / (double)refFrequency);
	uint16_t upper14 = (uint16_t)((freqWord >> 14) & 0x3FFF);
	uint16_t lower14 = (uint16_t)(freqWord & 0x3FFF);

	uint16_t reg = (freqReg == REG0) ? FREQ0_WRITE_REG : FREQ1_WRITE_REG;
	lower14 |= reg;
	upper14 |= reg;   

	WriteControlRegister();
	WriteRegister(lower14);			// Write lower 14 bits to AD9833
	WriteRegister(upper14);			// Write upper 14 bits to AD9833
}

void AD9833 :: IncrementFrequency ( Registers freqReg, double freqIncHz ) {
	double frequency = (freqReg == REG0) ? frequency0 : frequency1;
	SetFrequency(freqReg, frequency + freqIncHz);
}

void AD9833 :: SetPhase ( Registers phaseReg, float phaseInDeg ) {
	phaseInDeg = fmod(phaseInDeg, 360.0f);
	if ( phaseInDeg < 0 ) phaseInDeg += 360.0f;
	
	uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phaseInDeg) & 0x0FFF;
	phaseVal |= PHASE_WRITE_CMD;
	
	if ( phaseReg == REG0 )	{
		phase0 = phaseInDeg;
	}
	else {
		phase1 = phaseInDeg;
		phaseVal |= PHASE1_WRITE_REG;
	}
	WriteRegister(phaseVal);
}

void AD9833 :: IncrementPhase ( Registers phaseReg, float phaseIncDeg ) {
	float phase = (phaseReg == REG0) ? phase0 : phase1;
	SetPhase(phaseReg, phase + phaseIncDeg);
}

void AD9833 :: SetWaveform ( Registers waveFormReg, WaveformType waveType ) {
	if ( waveFormReg == REG0 )
		waveForm0 = waveType;
	else
		waveForm1 = waveType;
	WriteControlRegister();
}

void AD9833 :: SetOutputSource ( Registers freqReg, Registers phaseReg ) {
	activeFreq = freqReg;
	if ( phaseReg == SAME_AS_REG0 )
		activePhase = activeFreq;
	else
		activePhase = phaseReg;
	WriteControlRegister();
}

void AD9833 :: EnableOutput ( bool enable ) {
	outputEnabled = enable;
	WriteControlRegister();
}

void AD9833 :: SleepMode ( bool enable ) {
	DisableDAC(enable);
	DisableInternalClock(enable);
}

void AD9833 :: DisableDAC ( bool enable ) {
	DacDisabled = enable;
	WriteControlRegister();
}

void AD9833 :: DisableInternalClock ( bool enable ) {
	IntClkDisabled = enable;
	WriteControlRegister();
}

double AD9833 :: GetActualProgrammedFrequency ( Registers reg ) {
	double frequency = (reg == REG0) ? frequency0 : frequency1;
	uint32_t freqWord = (uint32_t)llround((frequency * 268435456.0) / (double)refFrequency) & 0x0FFFFFFFUL;
	return ((double)freqWord * (double)refFrequency) / 268435456.0;
}

float AD9833 :: GetActualProgrammedPhase ( Registers reg ) {
	float phase = (reg == REG0) ? phase0 : phase1;
	uint16_t phaseVal = (uint16_t)(BITS_PER_DEG * phase) & 0x0FFF;
	return (float)phaseVal / BITS_PER_DEG;
}

float AD9833 :: GetResolution ( void ) {
	return (float)refFrequency / 268435456.0f;
}

void AD9833 :: SetRefClock ( uint32_t referenceFrequency ) {
	if ( referenceFrequency == 0 ) return;
	refFrequency = referenceFrequency;
	// Recalculate and rewrite registers for both frequencies with new reference clock
	SetFrequency(REG0, frequency0);
	SetFrequency(REG1, frequency1);
}

uint32_t AD9833 :: GetRefClock ( void ) {
	return refFrequency;
}

// --------------------- PRIVATE FUNCTIONS --------------------------

void AD9833 :: WriteControlRegister ( void ) {
	uint16_t waveForm;
	if ( activeFreq == REG0 ) {
		waveForm = waveForm0;
		waveForm &= ~FREQ1_OUTPUT_REG;
	}
	else {
		waveForm = waveForm1;
		waveForm |= FREQ1_OUTPUT_REG;
	}
	if ( activePhase == REG0 )
		waveForm &= ~PHASE1_OUTPUT_REG;
	else
		waveForm |= PHASE1_OUTPUT_REG;
	if ( outputEnabled )
		waveForm &= ~RESET_CMD;
	else
		waveForm |= RESET_CMD;
	if ( DacDisabled )
		waveForm |= DISABLE_DAC;
	else
		waveForm &= ~DISABLE_DAC;
	if ( IntClkDisabled )
		waveForm |= DISABLE_INT_CLK;
	else
		waveForm &= ~DISABLE_INT_CLK;

	WriteRegister ( waveForm );
}

void AD9833 :: WriteRegister ( uint16_t dat ) {
    const SPISettings settings{2000000, MSBFIRST, SPI_MODE2};
    _spiBus->beginTransaction(settings);

	WRITE_FNCPIN(LOW);		// FNCpin low to write to AD9833

	_spiBus->transfer((uint8_t)(dat >> 8));	// Transmit MSB first
	_spiBus->transfer((uint8_t)(dat & 0xFF));

	WRITE_FNCPIN(HIGH);		// Write done
    _spiBus->endTransaction();
}
