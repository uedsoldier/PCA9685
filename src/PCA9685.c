/**
  * @file PCA9685.c
  * 
  * @brief Librería para control de módulos I²C-PWM de 16 canales PCA9685
  * @author Ing. José Roberto Parra Trewartha (uedsoldier1990@gmail.com)
  * @version 1.0
*/

#include "PCA9685.h"

/**
 * @brief
 * @param
 * @param
 * @return
 */
inline void PCA9685_i2c_start(){
    __PCA9685_i2c_start();
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
inline void PCA9685_i2c_stop(){
    __PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
inline void PCA9685_i2c_restart(){
    __PCA9685_i2c_restart();
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
inline uint8_t PCA9685_i2c_writeByte(uint8_t dato){
    return __PCA9685_i2c_writeByte(dato);
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
inline uint8_t PCA9685_i2c_readByte(bool ack){
    return __PCA9685_i2c_readByte(ack);
}

/**
 * @brief
 * @param
 * @param
 * @return
 */
void PCA9685_writeRegister(PCA9685 *modulo, uint8_t reg, uint8_t val){
    uint8_t addr_w = modulo->i2cAddress_w;
    #ifdef PCA9685_DEBUG
    printf("Write register reg:0x%X val:0x%X to PCA9685 at 0x%02X \r\n",reg,val,addr_w);
    #endif
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(addr_w);
    PCA9685_i2c_writeByte(reg);
    PCA9685_i2c_writeByte(val);
    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
uint8_t PCA9685_readRegister(PCA9685 *modulo, uint8_t reg){
    uint8_t retval;
    uint8_t addr_r = modulo->i2cAddress_r;
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(modulo->i2cAddress_w);
    PCA9685_i2c_writeByte(reg);
    PCA9685_i2c_restart();
    PCA9685_i2c_writeByte(addr_r);
    retval = PCA9685_i2c_readByte(0);
    PCA9685_i2c_stop();
    #ifdef PCA9685_DEBUG
    printf("Read register reg:0x%X val:0x%X from PCA9685 at 0x%02X \r\n",reg,retval,addr_r);
    #endif
    return retval;
}

/**
 * @brief Función de inicialización de un módulo PCA9685 individualmente (por direccionamiento directo mediante los pines [A5..0])
 * @param modulo (PCA9685*) Apuntador a estructura de datos de tipo PCA9685
 * @param address (uint8_t) Direccionamiento físico del módulo
 * @return (bool) True en caso de inicializacion correcta, false en caso contrarooi
 */
bool PCA9685_init(PCA9685 *modulo, uint8_t address) {
    // Inicialización de apuntadores a callbacks
    #if defined(__XC8)
        #if defined( PCA9685_I2C_MODULE ) 
            #if PCA9685_I2C_MODULE == 1 
                #if defined (I2C_V1) || defined (I2C_V4)
    __PCA9685_i2c_start = i2c_start;
    __PCA9685_i2c_stop = i2c_stop;
    __PCA9685_i2c_restart = i2c_restart;
    __PCA9685_i2c_writeByte = i2c_writeByte;
    __PCA9685_i2c_readByte = i2c_readByte;
                #elif defined (I2C_V2) || defined (I2C_V3) || defined (I2C_V5) || defined (I2C_V6) || defined (I2C_V6_1) || defined (I2C_V6_2)
    __PCA9685_i2c_start = i2c1_start;
    __PCA9685_i2c_stop = i2c1_stop;
    __PCA9685_i2c_restart = i2c1_restart;
    __PCA9685_i2c_writeByte = i2c1_writeByte;
    __PCA9685_i2c_readByte = i2c1_readByte;
                #endif
            #elif PCA9685_I2C_MODULE == 2 
    __PCA9685_i2c_start = i2c2_start;
    __PCA9685_i2c_stop = i2c2_stop;
    __PCA9685_i2c_restart = i2c2_restart;
    __PCA9685_i2c_writeByte = i2c2_writeByte;
    __PCA9685_i2c_readByte = i2c2_readByte;
            #elif PCA9685_I2C_MODULE == 0
    __PCA9685_i2c_start = i2c_sw_start;
    __PCA9685_i2c_stop = i2c_sw_stop;
    __PCA9685_i2c_restart = i2c_sw_restart;
    __PCA9685_i2c_writeByte = i2c_sw_writeByte;
    __PCA9685_i2c_readByte = i2c_sw_readByte;
            #else
                #error Módulo I²C incorrecto. Verificar.
            #endif
        #else
            #error No está definido el módulo I²C. Verificar.
        #endif 
    
    #else
    // Agregar callbacks deseadas
    #endif

    if(PCA9685_isConnected(modulo)) {
        modulo->i2cAddress_w = (PCA9685_I2C_ADDRESS|address);
        modulo->i2cAddress_r = (modulo->i2cAddress_w|0x01);
        modulo->driverMode = PCA9685_OutputDriverMode_TotemPole;
        modulo->enabledMode = PCA9685_OutputEnabledMode_Normal;
        modulo->disabledMode = PCA9685_OutputDisabledMode_Low;
        modulo->updateMode = PCA9685_ChannelUpdateMode_AfterStop;
        modulo->phaseBalancer = PCA9685_PhaseBalancer_None;
        modulo->isProxyAddresser = false;
        modulo->lastI2CError = 0;
        PCA9685_writeRegister(modulo,PCA9685_MODE1_REG,PCA9685_MODE1_RESTART | PCA9685_MODE1_AUTOINC);
    #ifdef PCA9685_DEBUG
        printf("PCA9685 init OK\r\n");
        PCA9685_printDeviceDetails(modulo);
    #endif
        return true;
    }
    else{
        #ifdef PCA9685_DEBUG
        printf("PCA9685 not present. Verify i2c bus.\r\n");
        #endif
        return false;
    }
    
    
}

#ifdef PCA9685_DEBUG
/**
 * 
 * @param modulo
 */
void PCA9685_printDeviceDetails(PCA9685 *modulo){
    printf(" I2C bus speed: %u [kHz]\r\n",i2c1_getSpeed());
    printf(" Device name: %s\r\n",PCA9685_getDeviceName(modulo));
    printf(" -i2c addresses: %02X->R %02X->W\r\n",modulo->i2cAddress_r,modulo->i2cAddress_w);
    printf(" -driver mode: %s\r\n",PCA9685_outputDriverMode_str_P[PCA9685_getOutputDriverMode(modulo)]);
    printf(" -output enabled mode: %s\r\n",PCA9685_outputEnabledMode_str_P[PCA9685_getOutputEnabledMode(modulo)]);
    printf(" -output disabled mode: %s\r\n",PCA9685_outputDisabledMode_str_P[PCA9685_getOutputDisabledMode(modulo)]);
    printf(" -channel update mode: %s\r\n",PCA9685_channelUpdateMode_str_P[PCA9685_getChannelUpdateMode(modulo)]);
    printf(" -phase balancer: %s\r\n",PCA9685_phaseBalancer_str_P[PCA9685_getPhaseBalancer(modulo)]);
}
#endif


/**
 * @brief Función para realizar secuencia de reset por software para todos los módulos PCA9685 presentes en el bus I²C
 * @param (void)
 * @return (void)
 */
void PCA9685_resetDevices(void) {
    #ifdef PCA9685_DEBUG
    printf("PCA9685 Software reset...");
    #endif
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(PCA9685_GENERAL_CALL_ADDRESS);    // General Call Address
    PCA9685_i2c_writeByte(PCA9685_SOFTWARE_RESET);          // Constante para software reset
    PCA9685_i2c_stop();
    __delay_us(5);
    #ifdef PCA9685_DEBUG
    printf("OK\r\n");
    #endif
}

/**
 * 
 * @param address
 * @return 
 */
bool PCA9685_isConnected(PCA9685 *modulo){
    uint8_t _i2cstatus;
    bool connected = false;
    uint8_t addr = modulo->i2cAddress_w;
    PCA9685_i2c_start();
    _i2cstatus = PCA9685_i2c_writeByte(addr);
    connected = (_i2cstatus == I2C_ACK)? true:false;
    PCA9685_i2c_stop();
    __delay_us(2);
    #ifdef PCA9685_DEBUG
    printf("PCA9685 at i2c address 0x%02X is connected? %s\r\n",addr,connected? "OK":"NOT OK");
    #endif
    return connected;
}

/**
 * 
 * @param modulo
 * @return 
 */
char *PCA9685_getDeviceName(PCA9685 *modulo){
    return modulo->device_name;
}

/**
 * 
 * @param modulo
 * @param name
 */
void PCA9685_setDeviceName(PCA9685 *modulo, const char *name){
    strcpy(modulo->device_name,name);
}



/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputDriverMode PCA9685_getOutputDriverMode(PCA9685 *modulo) {
	return modulo->driverMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputEnabledMode PCA9685_getOutputEnabledMode(PCA9685 *modulo) {
	return modulo->enabledMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_OutputDisabledMode PCA9685_getOutputDisabledMode(PCA9685 *modulo) {
	return modulo->disabledMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_ChannelUpdateMode PCA9685_getChannelUpdateMode(PCA9685 *modulo) {
	return modulo->updateMode;
}

/**
 * @brief
 * @param
 * @return
 */
PCA9685_PhaseBalancer PCA9685_getPhaseBalancer(PCA9685 *modulo) {
	return modulo->phaseBalancer;
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setFrequency(PCA9685 *modulo, float pwm_freq) {
	if(pwm_freq < 0)	return;// Valor incorrecto

	uint32_t divisor = (uint32_t)(4096.0F * pwm_freq);
	uint32_t preescala = division_entera_sin_signo(PCA9685_OSC_CLOCK,divisor)-1;
	preescala = constrain(preescala,3,255);	// Acotación [3,255]
    uint8_t _pre = (uint8_t)preescala;

	#ifdef PCA9685_DEBUG
    printf("Set PWM frequency -> %.2f [Hz] prescale-> %u\r\n",pwm_freq,_pre);
	#endif

	// El registro PRE_SCALE solo se puede establecer cuando el bit SLEEP del registro MODE1 se establece en 1 lógico.
    uint8_t mode1Reg = PCA9685_readRegister( modulo, PCA9685_MODE1_REG);
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_RESTART) | PCA9685_MODE1_SLEEP;
    PCA9685_writeRegister(modulo, PCA9685_MODE1_REG, mode1Reg);
    PCA9685_writeRegister(modulo, PCA9685_PRESCALE_REG, preescala);

    // Se necesitan 500 [μs] como máximo para que el oscilador esté en funcionamiento una vez que el bit SLEEP se haya establecido en 0 lógico.
    mode1Reg = (mode1Reg & ~PCA9685_MODE1_SLEEP) | PCA9685_MODE1_RESTART;
    PCA9685_writeRegister(modulo, PCA9685_MODE1_REG, mode1Reg);
    
    __delay_us(500);



}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelOn(PCA9685 *modulo, uint8_t channel) {
	if (channel > 15)
        return;
	#ifdef PCA9685_DEBUG
    printf("Channel %u ON\r\n",channel);
	#endif

    PCA9685_writeChannelBegin(modulo, channel);
    PCA9685_writeChannelPWM(PCA9685_PWM_FULL, 0);  // time_on = FULL; time_off = 0;
    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelOff(PCA9685 *modulo, uint8_t channel) {
	if (channel > 15)
        return;
	#ifdef PCA9685_DEBUG
	printf("Channel %u OFF\r\n",channel);
	#endif

    PCA9685_writeChannelBegin(modulo, channel);
    PCA9685_writeChannelPWM(0, PCA9685_PWM_FULL);  // time_on = 0; time_off = FULL;
    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelPWM(PCA9685 *modulo, uint8_t channel, uint16_t duty_cycle) {
	if (channel > 15) 
        return;

	#ifdef PCA9685_DEBUG
    printf("Channel %u PWM-> %u\r\n",channel,duty_cycle);
	#endif

    PCA9685_writeChannelBegin(modulo, channel);

    uint16_t phaseBegin, phaseEnd;
    PCA9685_getPhaseCycle(modulo, channel, duty_cycle, &phaseBegin, &phaseEnd);

    PCA9685_writeChannelPWM(phaseBegin, phaseEnd);

    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setChannelsPWM(PCA9685 *modulo, uint8_t starting_channel, uint8_t num_channels, uint16_t *duty_cycles) {
	if (starting_channel > 15)
        return;
    if (starting_channel + num_channels > 16) num_channels -= (starting_channel + num_channels) - 16;

#ifdef PCA9685_DEBUG
    printf("Set channels PWM. numChannels -> %u",num_channels);
#endif

    // From avr/libraries/Wire.h and avr/libraries/utility/twi.h, BUFFER_LENGTH controls
    // how many channels can be written at once. Therefore, we loop around until all
    // channels have been written out into their registers. I2C_BUFFER_LENGTH is used in
    // other architectures, so we rely on PCA9685_I2C_BUFFER_LENGTH logic to sort it out.

//    while (num_channels > 0) {
//        PCA9685_writeChannelBegin(modulo, starting_channel);
//
//#ifndef PCA9685_USE_SOFTWARE_I2C
//        int maxChannels = 0; // PENDIENTE: ////// min(num_channels, (PCA9685_I2C_BUFFER_LENGTH - 1) / 4);
//#else // TODO: Software I2C doesn't have buffer length restrictions? -NR
//        int maxChannels = numChannels;
//#endif
//        while (maxChannels-- > 0) {
//            uint16_t phaseBegin, phaseEnd;
//            PCA9685_getPhaseCycle(modulo, starting_channel++, *duty_cycles++, &phaseBegin, &phaseEnd);
//            PCA9685_writeChannelPWM(phaseBegin, phaseEnd);
//            --num_channels;
//        }
//
//        PCA9685_i2c_stop();
//        if (modulo->lastI2CError) return;
//    }
}

/**
 * @brief
 * @param
 * @return
 */
void PCA9685_setAllChannelsPWM(PCA9685 *modulo, uint16_t duty_cycle) {
	#ifdef PCA9685_DEBUG
    printf("Set all channels PWM -> %u",duty_cycle);
#endif

//    PCA9685_writeChannelBegin(modulo, PCA9685_ALLLED_CHANNEL);
//
//    uint16_t phaseBegin, phaseEnd;
//    PCA9685_getPhaseCycle(modulo, PCA9685_ALLLED_CHANNEL, duty_cycle, &phaseBegin, &phaseEnd);
//
//    PCA9685_writeChannelPWM(phaseBegin, phaseEnd);
//
//    PCA9685_i2c_stop();
}

/**
 * @brief
 * @param
 * @return
 */
uint16_t PCA9685_getChannelPWM(PCA9685 *modulo, uint8_t channel) {
	if (channel > 15 || modulo->isProxyAddresser)
        return 0;

    uint8_t regAddress = PCA9685_LED0_REG + (channel << 2);

#ifdef PCA9685_DEBUG
    printf("Channel %u regAddress -> 0x%X\r\n",channel,regAddress);
#endif

////    i2cWire_beginTransmission(modulo->i2cAddress);
////    i2cWire_write(regAddress);
////    if (i2cWire_endTransmission()) {
//#ifdef PCA9685_DEBUG
////        checkForErrors();
//#endif
////        return 0;
////    }
//
//    int bytesRead = i2cWire_requestFrom((uint8_t)modulo->i2cAddress, 4);
//    if (bytesRead != 4) {
//        while (bytesRead-- > 0)
//            i2cWire_read();
//#ifdef PCA9685_USE_SOFTWARE_I2C
//        PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
//#endif
//        modulo->lastI2CError = 4;
//#ifdef PCA9685_DEBUG
//        checkForErrors();
//#endif
//        return 0;
//    }
//
//#ifndef PCA9685_SWAP_PWM_BEG_END_REGS
//    uint16_t phaseBegin = (uint16_t)i2cWire_read();
//    phaseBegin |= (uint16_t)i2cWire_read() << 8;
//    uint16_t phaseEnd = (uint16_t)i2cWire_read();
//    phaseEnd |= (uint16_t)i2cWire_read() << 8;
//#else
//    uint16_t phaseEnd = (uint16_t)i2cWire_read();
//    phaseEnd |= (uint16_t)i2cWire_read() << 8;
//    uint16_t phaseBegin = (uint16_t)i2cWire_read();
//    phaseBegin |= (uint16_t)i2cWire_read() << 8;
//#endif
//    
//#ifdef PCA9685_USE_SOFTWARE_I2C
//    PCA9685_i2c_stop(); // Manually have to send stop bit in software i2c mode
//#endif
//
//#ifdef PCA9685_DEBUG
//    printf(" * phaseBegin: %u\r\n",phaseBegin);
//    printf(" * phaseEnd: %u\r\n",phaseEnd);
//#endif
//
//    // See datasheet section 7.3.3
//    uint16_t retVal;
//    if (phaseEnd >= PCA9685_PWM_FULL)
//        // Full OFF
//        // Figure 11 Example 4: full OFF takes precedence over full ON
//        // See also remark after Table 7
//        retVal = 0;
//    else if (phaseBegin >= PCA9685_PWM_FULL)
//        // Full ON
//        // Figure 9 Example 3
//        retVal = PCA9685_PWM_FULL;
//    else if (phaseBegin <= phaseEnd)
//        // start and finish in same cycle
//        // Section 7.3.3 example 1
//        retVal = phaseEnd - phaseBegin;
//    else
//        // span cycles
//        // Section 7.3.3 example 2
//        retVal = (phaseEnd + PCA9685_PWM_FULL) - phaseBegin;
//
//#ifdef PCA9685_DEBUG
//    printf("retVal: %u\r\n",retVal);
//#endif
//
//    return retVal;
}


/**
 * @brief
 * @param
 * @return
 */
void PCA9685_getPhaseCycle(PCA9685 *modulo, uint8_t channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd) {
	if (channel == PCA9685_ALLLED_CHANNEL) {
        *phaseBegin = 0; // ALLLED should not receive a phase shifted begin value
    } else {
        // Get phase delay begin
        switch(modulo->phaseBalancer) {
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
void PCA9685_writeChannelBegin(PCA9685 *modulo, uint8_t channel) {
	uint8_t regAddress;

    if (channel != PCA9685_ALLLED_CHANNEL)
        regAddress = PCA9685_LED0_REG + (channel * 0x04);
    else
        regAddress = PCA9685_ALLLED_REG;

#ifdef PCA9685_DEBUG
    printf("writeChannelBegin channel -> %u regAddress -> 0x%X\r\n",channel,regAddress);
#endif
    PCA9685_i2c_start();
    PCA9685_i2c_writeByte(modulo->i2cAddress_w);
    PCA9685_i2c_writeByte(regAddress);
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
    
    PCA9685_i2c_writeByte(make8(phaseBegin,0));
    PCA9685_i2c_writeByte(make8(phaseBegin,1));
    PCA9685_i2c_writeByte(make8(phaseEnd,0));
    PCA9685_i2c_writeByte(make8(phaseEnd,1));

#else
    PCA9685_i2c_writeByte(make8(phaseEnd,0));
    PCA9685_i2c_writeByte(make8(phaseEnd,1));
    PCA9685_i2c_writeByte(make8(phaseBegin,0));
    PCA9685_i2c_writeByte(make8(phaseBegin,1));
#endif
}

///**
// * @brief
// * @param
// * @return
// */
//void PCA9685_writeChannelEnd(PCA9685 *modulo) {
//	i2cWire_endTransmission();
//}
