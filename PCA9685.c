#include <stdint.h>
#include <stdbool.h>
#include "../../utils/utils.h"
#include "PCA9685.h"


#if defined( PCA8695_I2C_MODULE ) 
	#if PCA8695_I2C_MODULE == 1 

    #elif PCA8695_I2C_MODULE == 2 

	#elif PCA8695_I2C_MODULE == 0
	
	#endif
#else
	#error No está definido el módulo i2c. Verificar.
#endif   

/**
 */
void PCA9685_init(uint8_t add, uint8_t mode) {
}

void PCA9685_resetDevices(void) {
}


PCA9685_OutputDriverMode getOutputDriverMode(void) {
	return _driverMode;
}

PCA9685_OutputEnabledMode getOutputEnabledMode(void) {
	return _enabledMode;
}

PCA9685_OutputDisabledMode getOutputDisabledMode(void) {
	return _disabledMode;
}

PCA9685_ChannelUpdateMode getChannelUpdateMode(void) {
	return _updateMode;
}

PCA9685_PhaseBalancer getPhaseBalancer(void) {
	return _phaseBalancer;
}


void PCA9685_setPWMFrequency(float pwm_freq) {
	if(pwm_freq < 0)	return;// Valor incorrecto

	uint32_t divisor = 4096 * (uint16_t)(pwm_freq);
	uint32_t preescala = (uint16_t)division_entera_sin_signo(PCA8695_OSC_CLOCK,divisor)-1;
	preescala = (preescala > 255)? 255:((preescala < 3)? 3: preescala );	// Acotación [3,255]

	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::setPWMFrequency pwmFrequency: ");
    Serial.print(pwmFrequency);
    Serial.print(", preScalerVal: 0x");
    Serial.println(preScalerVal, HEX);
	#endif

	// El registro PRE_SCALE solo se puede establecer cuando el bit SLEEP del registro MODE1 se establece en 1 lógico.
    uint8_t mode1Reg = readRegister(PCA9685_MODE1_REG);
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP));
    writeRegister(PCA9685_PRESCALE_REG, (uint8_t)preescala);

    // Se necesitan 500 [μs] como máximo para que el oscilador esté en funcionamiento una vez que el bit SLEEP se haya establecido en 0 lógico.
    writeRegister(PCA9685_MODE1_REG, (mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART));
    __delay_us(500);



}

void PCA9685_setChannelOn(uint16_t channel) {
	if (channel < 0 || channel > 15) return;
	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setChannelOn");
	#endif

    writeChannelBegin(channel);
    writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    writeChannelEnd();
}

void PCA9685_setChannelOff(uint16_t channel) {
	if (channel < 0 || channel > 15) return;

	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
	Serial.println("PCA9685::setChannelOff");
	#endif

    writeChannelBegin(channel);
    writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    writeChannelEnd();
}
    
void PCA9685_setChannelPWM(uint16_t channel, uint16_t duty_cycle) {
	if (channel < 0 || channel > 15) return;

	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setChannelPWM");
	#endif

    writeChannelBegin(channel);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(channel, duty_cycle, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

void PCA9685_setChannelsPWM(uint16_t starting_channel, uint16_t num_channels, uint16_t *duty_cycles) {
	if (starting_channel < 0 || starting_channel > 15 || num_channels < 0) return;
    if (starting_channel + num_channels > 16) num_channels -= (starting_channel + num_channels) - 16;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::setChannelsPWM numChannels: ");
    Serial.println(numChannels);
#endif

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

    while (num_channels > 0) {
        writeChannelBegin(starting_channel);

#ifndef PCA9685_USE_SOFTWARE_I2C
        int maxChannels = 0; // PENDIENTE: ////// min(num_channels, (PCA9685_I2C_BUFFER_LENGTH - 1) / 4);
#else // TODO: Software I2C doesn't have buffer length restrictions? -NR
        int maxChannels = numChannels;
#endif
        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            getPhaseCycle(starting_channel++, *duty_cycles++, &phaseBegin, &phaseEnd);

            writeChannelPWM(phaseBegin, phaseEnd);
            --num_channels;
        }

        writeChannelEnd();
        if (_lastI2CError) return;
    }
}

void PCA9685_setAllChannelsPWM(uint16_t duty_cycle) {
	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.println("PCA9685::setAllChannelsPWM");
#endif

    writeChannelBegin(PCA9685_ALLLED_CHANNEL);

    uint16_t phaseBegin, phaseEnd;
    getPhaseCycle(PCA9685_ALLLED_CHANNEL, duty_cycle, &phaseBegin, &phaseEnd);

    writeChannelPWM(phaseBegin, phaseEnd);

    writeChannelEnd();
}

uint16_t PCA9685_getChannelPWM(uint16_t channel) {
	if (channel < 0 || channel > 15 || _isProxyAddresser) return 0;

    uint8_t regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("PCA9685::getChannelPWM channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

    int bytesRead = i2cWire_requestFrom((uint8_t)_i2cAddress, 4);
    if (bytesRead != 4) {
        while (bytesRead-- > 0)
            i2cWire_read();
#ifdef PCA9685_USE_SOFTWARE_I2C
        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif
        _lastI2CError = 4;
#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
        checkForErrors();
#endif
        return 0;
    }

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
#else
    uint16_t phaseEnd = (uint16_t)i2cWire_read();
    phaseEnd |= (uint16_t)i2cWire_read() << 8;
    uint16_t phaseBegin = (uint16_t)i2cWire_read();
    phaseBegin |= (uint16_t)i2cWire_read() << 8;
#endif
    
#ifdef PCA9685_USE_SOFTWARE_I2C
    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
#endif

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::getChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

    // See datasheet section 7.3.3
    uint16_t retVal;
    if (phaseEnd >= PCA9685_PWM_FULL)
        // Full OFF
        // Figure 11 Example 4: full OFF takes precedence over full ON
        // See also remark after Table 7
        retVal = 0;
    else if (phaseBegin >= PCA9685_PWM_FULL)
        // Full ON
        // Figure 9 Example 3
        retVal = PCA9685_PWM_FULL;
    else if (phaseBegin <= phaseEnd)
        // start and finish in same cycle
        // Section 7.3.3 example 1
        retVal = phaseEnd - phaseBegin;
    else
        // span cycles
        // Section 7.3.3 example 2
        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::getChannelPWM retVal: ");
    Serial.println(retVal);
#endif

    return retVal;
}


/**
 */
void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
	if (channel == PCA9685_ALLLED_CHANNEL) {
        *phaseBegin = 0; // ALLLED should not receive a phase shifted begin value
    } else {
        // Get phase delay begin
        switch(_phaseBalancer) {
            case PCA9685_PhaseBalancer_None:
            case PCA9685_PhaseBalancer_Count:
            case PCA9685_PhaseBalancer_Undefined:
                *phaseBegin = 0;
                break;

            case PCA9685_PhaseBalancer_Linear:
                // Distribute high phase area over more of the duty cycle range to balance load
                *phaseBegin = (channel * ((4096 / 16) / 16)) & PCA9685_PWM_MASK;
                break;
        }
    }

    // See datasheet section 7.3.3
    if (pwmAmount == 0) {
        // Full OFF -> time_end[bit12] = 1
        *phaseEnd = PCA9685_PWM_FULL;
    }
    else if (pwmAmount >= PCA9685_PWM_FULL) {
        // Full ON -> time_beg[bit12] = 1, time_end[bit12] = <ignored>
        *phaseBegin |= PCA9685_PWM_FULL;
        *phaseEnd = 0;
    }
    else {
        *phaseEnd = (*phaseBegin + pwmAmount) & PCA9685_PWM_MASK;
    }
}

void writeChannelBegin(int channel) {
	uint8_t regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::writeChannelBegin channel: ");
    Serial.print(channel);
    Serial.print(", regAddress: 0x");
    Serial.println(regAddress, HEX);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
}

void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
	#ifdef PCA9685_ENABLE_DEBUG_OUTPUT
    Serial.print("  PCA9685::writeChannelPWM phaseBegin: ");
    Serial.print(phaseBegin);
    Serial.print(", phaseEnd: ");
    Serial.println(phaseEnd);
#endif

#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
    i2cWire_write(lowByte(phaseBegin));
    i2cWire_write(highByte(phaseBegin));
    i2cWire_write(lowByte(phaseEnd));
    i2cWire_write(highByte(phaseEnd));
#else
    i2cWire_write(lowByte(phaseEnd));
    i2cWire_write(highByte(phaseEnd));
    i2cWire_write(lowByte(phaseBegin));
    i2cWire_write(highByte(phaseBegin));
#endif
}

void writeChannelEnd(void) {
	i2cWire_endTransmission();
}
