#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "../../utils/utils.h"
#include "PCA9685.h"

/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_i2c_start(){
    #if defined( PCA9685_I2C_MODULE ) 
	#if PCA9685_I2C_MODULE == 1 
    
    #elif PCA9685_I2C_MODULE == 2 

	#elif PCA9685_I2C_MODULE == 0
	
	#endif
#else
	#error No está definido el módulo I²C. Verificar.
#endif 
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_i2c_stop(){
    #if defined( PCA9685_I2C_MODULE ) 
	#if PCA9685_I2C_MODULE == 1 
    
    #elif PCA9685_I2C_MODULE == 2 

	#elif PCA9685_I2C_MODULE == 0
	
	#endif
#else
	#error No está definido el módulo I²C. Verificar.
#endif 
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_i2c_restart(){
    #if defined( PCA9685_I2C_MODULE ) 
	#if PCA9685_I2C_MODULE == 1 
    
    #elif PCA9685_I2C_MODULE == 2 

	#elif PCA9685_I2C_MODULE == 0
	
	#endif
#else
	#error No está definido el módulo I²C. Verificar.
#endif 
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_i2c_writeByte(uint8_t dato){
    #if defined( PCA9685_I2C_MODULE ) 
	#if PCA9685_I2C_MODULE == 1 
    
    #elif PCA9685_I2C_MODULE == 2 

	#elif PCA9685_I2C_MODULE == 0
	
	#endif
#else
	#error No está definido el módulo I²C. Verificar.
#endif 
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
uint8_t PCA9685_i2c_readByte(bool ack){
    #if defined( PCA9685_I2C_MODULE ) 
	#if PCA9685_I2C_MODULE == 1 
    return 0;////
    #elif PCA9685_I2C_MODULE == 2 
    return 0;////
	#elif PCA9685_I2C_MODULE == 0
	return 0;////
	#endif
#else
	#error No está definido el módulo I²C. Verificar.
#endif 
}

const uint8_t i = PCA9685_MAX_CHANNEL;


/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_writeRegister(uint8_t reg, uint8_t val){
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(_i2cAddress);
    PCA9685_i2c_writeByte(reg);
    PCA9685_i2c_writeByte(val);
    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
uint8_t PCA9685_readRegister(uint8_t reg){
    uint8_t retval;
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(_i2cAddress);
    PCA9685_i2c_writeByte(reg);
    retval = PCA9685_i2c_readByte(1);
    PCA9685_i2c_stop();
    return retval;
}

/**
 * @brief Función de inicialización de módulos PCA9685 individualmente (por direccionamiento directo mediante los pines [A5..0])
 * @param
 * @return
 */
void PCA9685_init(uint8_t address, uint8_t mode) {
}

/**
 * @brief Función para realizar secuencia de reset por software para los módulos PCA9685 presentes en el bus I²C
 * @param (void)
 * @return (void)
 */
void PCA9685_resetDevices(void) {
    #ifdef PCA9685_DEBUG
    printf("PCA9685 Software reset\r\n");
    #endif
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(0x00);                    // General Call Address
    PCA9685_i2c_writeByte(PCA9685_SOFTWARE_RESET);  // Constante para software reset
    PCA9685_i2c_stop();
    __delay_us(10);
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputDriverMode PCA9685_getOutputDriverMode(void) {
	return _driverMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputEnabledMode PCA9685_getOutputEnabledMode(void) {
	return _enabledMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputDisabledMode PCA9685_getOutputDisabledMode(void) {
	return _disabledMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_ChannelUpdateMode PCA9685_getChannelUpdateMode(void) {
	return _updateMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_PhaseBalancer PCA9685_getPhaseBalancer(void) {
	return _phaseBalancer;
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setFrequency(float pwm_freq) {
	if(pwm_freq < 0)	return;// Valor incorrecto

	uint32_t divisor = 4096 * (uint16_t)(pwm_freq);
	uint32_t preescala = (uint16_t)division_entera_sin_signo(PCA9685_OSC_CLOCK,divisor)-1;
	preescala = (uint8_t)constrain(preescala,3,255);	// Acotación [3,255]

	#ifdef PCA9685_DEBUG
    printf("Set PWM frequency -> %.2f [Hz] prescale-> 0x%X\r\n",pwm_freq,preescala);
	#endif

	// El registro PRE_SCALE solo se puede establecer cuando el bit SLEEP del registro MODE1 se establece en 1 lógico.
    uint8_t mode1Reg = PCA9685_readRegister(PCA9685_MODE1_REG);
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    PCA9685_writeRegister(PCA9685_MODE1_REG, mode1Reg);
    PCA9685_writeRegister(PCA9685_PRESCALE_REG, preescala);

    // Se necesitan 500 [μs] como máximo para que el oscilador esté en funcionamiento una vez que el bit SLEEP se haya establecido en 0 lógico.
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART;
    PCA9685_writeRegister(PCA9685_MODE1_REG, mode1Reg);
    __delay_us(500);



}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelOn(uint16_t channel) {
	if (channel < 0 || channel > 15) return;
	#ifdef PCA9685_DEBUG
    printf("Channel %u ON\r\n");
	#endif

    PCA9685_writeChannelBegin(channel);
    PCA9685_writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    PCA9685_writeChannelEnd();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelOff(uint16_t channel) {
	if (channel < 0 || channel > 15) return;

	#ifdef PCA9685_DEBUG
	printf("Channel %u OFF\r\n");
	#endif

    PCA9685_writeChannelBegin(channel);
    PCA9685_writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    PCA9685_writeChannelEnd();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelPWM(uint16_t channel, uint16_t duty_cycle) {
	if (channel < 0 || channel > 15) return;

	#ifdef PCA9685_DEBUG
    printf("Channel %u PWM-> %u\r\n",channel,duty_cycle);
	#endif

    PCA9685_writeChannelBegin(channel);

    uint16_t phaseBegin, phaseEnd;
    PCA9685_getPhaseCycle(channel, duty_cycle, &phaseBegin, &phaseEnd);

    PCA9685_writeChannelPWM(phaseBegin, phaseEnd);

    PCA9685_writeChannelEnd();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelsPWM(uint16_t starting_channel, uint16_t num_channels, uint16_t *duty_cycles) {
	if (starting_channel < 0 || starting_channel > 15 || num_channels < 0) return;
    if (starting_channel + num_channels > 16) num_channels -= (starting_channel + num_channels) - 16;

#ifdef PCA9685_DEBUG
    printf("Set channels PWM. numChannels -> %u",num_channels);
#endif

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

    while (num_channels > 0) {
        PCA9685_writeChannelBegin(starting_channel);

#ifndef PCA9685_USE_SOFTWARE_I2C
        int maxChannels = 0; // PENDIENTE: ////// min(num_channels, (PCA9685_I2C_BUFFER_LENGTH - 1) / 4);
#else // TODO: Software I2C doesn't have buffer length restrictions? -NR
        int maxChannels = numChannels;
#endif
        while (maxChannels-- > 0) {
            uint16_t phaseBegin, phaseEnd;
            PCA9685_getPhaseCycle(starting_channel++, *duty_cycles++, &phaseBegin, &phaseEnd);
            PCA9685_writeChannelPWM(phaseBegin, phaseEnd);
            --num_channels;
        }

        PCA9685_writeChannelEnd();
        if (_lastI2CError) return;
    }
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setAllChannelsPWM(uint16_t duty_cycle) {
	#ifdef PCA9685_DEBUG
    printf("Set all channels PWM -> %u",duty_cycle);
#endif

    PCA9685_writeChannelBegin(PCA9685_ALLLED_CHANNEL);

    uint16_t phaseBegin, phaseEnd;
    PCA9685_getPhaseCycle(PCA9685_ALLLED_CHANNEL, duty_cycle, &phaseBegin, &phaseEnd);

    PCA9685_writeChannelPWM(phaseBegin, phaseEnd);

    PCA9685_writeChannelEnd();
}

/**
 * @brief
 * @param
 * @return
 */
uint16_t PCA9685_getChannelPWM(uint16_t channel) {
	if (channel < 0 || channel > 15 || _isProxyAddresser) return 0;

    uint8_t regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_DEBUG
    printf("Channel %u regAddress -> 0x%X\r\n",channel,regAddress);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
    if (i2cWire_endTransmission()) {
#ifdef PCA9685_DEBUG
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
#ifdef PCA9685_DEBUG
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

#ifdef PCA9685_DEBUG
    printf(" * phaseBegin: %u\r\n",phaseBegin);
    printf(" * phaseEnd: %u\r\n",phaseEnd);
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

#ifdef PCA9685_DEBUG
    printf("retVal: %u\r\n",retVal);
#endif

    return retVal;
}


/**
 * @brief
 * @param
 * @return
 */
void PCA9685_getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
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

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_writeChannelBegin(int channel) {
	uint8_t regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_DEBUG
    printf("writeChannelBegin channel -> %u regAddress -> 0x%X\r\n",channel,regAddress);
#endif

    i2cWire_beginTransmission(_i2cAddress);
    i2cWire_write(regAddress);
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd) {
	#ifdef PCA9685_DEBUG
    printf(" * phaseBegin: %u\r\n",phaseBegin);
    printf(" * phaseEnd: %u\r\n",phaseEnd);
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

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_writeChannelEnd(void) {
	i2cWire_endTransmission();
}
