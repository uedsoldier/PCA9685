/**
  * @file PCA9685.h
  * 
  * @brief Librería para control de módulos PCA9685, drivers I²C-PWM de 16 canales 
  * @author Ing. José Roberto Parra Trewartha (uedsoldier1990@gmail.com)
  * @version 1.0
*/

#ifndef PCA9685_H
#define PCA9685_H

// Dependencias
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/**
 * Opciones de compilación según arquitectura
*/
#if defined(__XC8)
#include <xc.h>
#include "../../pconfig.h"
#if PCA9685_I2C_MODULE == 0
#include "../../emulated_protocols/I2C_SW/i2c_sw.h"
#else
#include "../../peripherals/I2C/i2c.h"
#endif
#include "../../utils/utils.h"
#elif defined(__arm__)
// TODO
#endif

/**
 * Apuntadores a funciones básicas I²C, para portabilidad de código a otras plataformas
 */
void (*__PCA9685_i2c_start)(void);
void (*__PCA9685_i2c_stop)(void);
void (*__PCA9685_i2c_restart)(void);
uint8_t (*__PCA9685_i2c_writeByte)(uint8_t);
uint8_t (*__PCA9685_i2c_readByte)(bool);


/**
 * @brief Número total de dispositivos PCS9685 presentes en el bus I²C. Máximo 6 líneas de direccionamiento, para un total de 62 (2⁶-2) dispositivos.
*/
#define PCA9685_DEVICE_COUNT 1

/**
  * @brief Macro de selección de módulo I²C a utilizar.
  * Para microcontroladores PIC de 8 bits:
  * -> 0 indica I²C emulado por software
  * -> 1,2,... indica I²C por hardware (depende del dispositivo)
  * Para Raspberry Pi //TODO
  * -> 0,1 para los diferentes modulos I²C por hardware de la Raspberry
*/
#define PCA9685_I2C_MODULE  1

/**
  * @brief Macro de selección de modalidad de depuración mediante puerto serie (tanto por hardware como por software) o por terminal. Definir en 0 en caso de no requerirse esta funcionalidad
*/
#define PCA9685_LOG 0


// Poner esta macro a 1 para intercambiar los valores de fase baja (inicial) / alta (final) de PWM en lecturas / escrituras de registro (necesario para algunos fabricantes de chips).
#define PCA9685_SWAP_PWM_BEG_END_REGS 0

/**
 * @brief Macro de frecuencia de oscilador del módulo. Por defecto es 25 [MHz], aunque puede cambiarse si se hace uso del pin EXTCLK
 */
#define PCA9685_OSC_CLOCK 25000000UL  

/**
 * @brief Canal mínimo para un módulo PCA9685
 */
#define PCA9685_MIN_CHANNEL         0

/**
 * @brief Número total de canales para un solo módulo PCA9685
 */
#define PCA9685_CHANNEL_COUNT       16

/**
 * @brief Canal máximo para un conjunto de módulos PCA9685 conectados al mismo bus I²C
 */
#define PCA9685_MAX_CHANNEL         ((PCA9685_DEVICE_COUNT * PCA9685_CHANNEL_COUNT)-1)

// Dirección I²C básica del PCA9685. Modo escritura (R/W = 0). Tomar en cuenta sus 6 bits de direccionamiento [A5.0].
#define PCA9685_I2C_BASE_MODULE_ADDRESS		0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK		0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  	0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  	0xFE

// Dirección I²C general de todos los PCA9685 en el bus. Modo escritura (R/W = 0) 
#define PCA9685_LED_ALLCALL     0xE0

// Dirección I²C para subdireccionamiento de grupo 1. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_1       0xE2

// Dirección I²C para subdireccionamiento de grupo 2. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_2       0xE4

// Dirección I²C para subdireccionamiento de grupo 3. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_3       0xE8

// Dirección para todos los dispositivos en el bus I²C 
#define PCA9685_GENERAL_CALL_ADDRESS    0x00

// Dato I²C para reset de módulos PCA9685 por software (SWRST). Se debe mandar después de direccionar a todos los dispositivos por medio de GENERAL CALL ADDRESS (dirección 0x00 I²C) 
#define PCA9685_SOFTWARE_RESET  0x06

// Registros internos del PCA9685
#define PCA9685_MODE1_REG               0x00    // Registro de modalidad 1
#define PCA9685_MODE2_REG               0x01    // Registro de modalidad 2
#define PCA9685_SUBADR1_REG             0x02    // Registro de subdireccionamiento 1
#define PCA9685_SUBADR2_REG             0x03    // Registro de subdireccionamiento 2
#define PCA9685_SUBADR3_REG             0x04    // Registro de subdireccionamiento 3
#define PCA9685_ALLCALL_REG             0x05    // Registro de direccionamiento LED All Call
#define PCA9685_LED0_REG                0x06    // Inicio de los registros LEDx, 4B per reg, 2B on phase, 2B off phase, formato little-endian
#define PCA9685_PRESCALE_REG            0xFE    // Registro de preescala
#define PCA9685_ALLLED_REG              0xFA    // Registro de todos los LEDs

// Mode1 register values
#define PCA9685_MODE1_RESTART           0x80    // En lectura, muestra el estado de la lógica de RESET. Previamente el usuario.
#define PCA9685_MODE1_EXTCLK            0x40    // En bajo utiliza reloj interno, en alto usa fuente de reloj externa del pin EXTCLK.
#define PCA9685_MODE1_AUTOINC           0x20    // Utilización de autoincremento de registros, para lectura o escritura secuencial de registros.
#define PCA9685_MODE1_SLEEP             0x10    // En bajo, configura en modo normal, mientras que en alto, entra en modo de bajo consumo, desactivando el circuito oscilador.
#define PCA9685_MODE1_SUBADR1           0x08    // En alto, para que el módulo PCA9685 en cuestión responda al subdireccionamiento 1. En bajo, ignora el subdireccionamiento.
#define PCA9685_MODE1_SUBADR2           0x04    // En alto, para que el módulo PCA9685 en cuestión responda al subdireccionamiento 2. En bajo, ignora el subdireccionamiento.
#define PCA9685_MODE1_SUBADR3           0x02    // En alto, para que el módulo PCA9685 en cuestión responda al subdireccionamiento 3. En bajo, ignora el subdireccionamiento.
#define PCA9685_MODE1_ALLCALL           0x01    // En alto, para que el módulo PCA9685 en cuestión responda a la dirección LED All Call. En bajo, ignora esta dirección.

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      0x04    // En alto, salidas configuradas en una arquitectura totem pole, en bajo, se configuran en colector abierto (open-drain)
#define PCA9685_MODE2_INVRT             0x10    // En alto, la lógica de salida es negada (cuando hay un driver de potencia externo), en bajo la lógica de salida es convencional (cuando no hay driver externo)
#define PCA9685_MODE2_OUTNE_TPHIGH      0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       0x02
#define PCA9685_MODE2_OCH_ONACK         0x08    // En alto, las salidas cambian de estado cuando se presenta ACK, mientras que en bajo cambian en la condición STOP

//
#define PCA9685_PWM_FULL                (uint16_t)0x1000    // Special value for full on/full off LEDx modes
#define PCA9685_PWM_MASK                (uint16_t)0x0FFF    // Mask for 12-bit/4096 possible phase positions

/**
 * @brief Enumeración de modo de control del driver de salida. Para mayor información, consultar la tabla 12 de la hoja de datos y las figuras 13, 14 y 15 sobre
 * el funcionamiento de de OUTDRV).
 * @note El modo totem-pole debe usarse cuando existe un controlador externo tipo N o tipo P, para que proporcione corriente de suministro mientras que
 * el modo de open-drain no lo hace. El límite de corriente por canal es de 25[mA] @ 5[V] en sumidero (sink), mientras que en suministro (source) en modo totem-pole,
 * es de 10 [mA] @ 5[V]. Sin embargo, de la hoja de datos Tabla 6. subnota [1]: "Algunos LED más nuevos incluyen diodos Zener integrados para limitar voltajes transitorios,
 * reducen las interferencias electromagnéticas y protegen los LED, y estos -DEBEN- ser controlados únicamente en el modo de open-drain para evitar el sobrecalentamiento del CI ".
 * También de la hoja de datos, Sección 10. sección 5: "en la arquitectura push-pull hay una ruta de baja resistencia a GND a través del Zener y esto [hace que] el CI se sobrecaliente ".
 */
typedef enum PCA9685_OutputDriverMode {
    PCA9685_OutputDriverMode_OpenDrain,         // Salidas del módulo configuradas en drenaje abierto (open-drain) con un límite de 400[mA] @5[V] de corriente en sumidero (sink), útil para LEDs y servomotores de baja potencia.
    PCA9685_OutputDriverMode_TotemPole,         // Salidas del módulo configuradas en totem-pole (aka push-pull) con un límite de 400[mA] @5[V] de corriente en sumidero (sink) y 160[mA] de corriente suministrada (source), útil para drivers externos (por defecto).
    PCA9685_OutputDriverMode_Count,             // Para uso interno únicamente.
    PCA9685_OutputDriverMode_Undefined = -1     // Para uso interno únicamente.
} PCA9685_OutputDriverMode;

/**
 * @brief Enumeración de modo del driver de salida (Output-enabled/active-low-OE-pin=LOW). Para mayor información, consultar la tabla 12 de la hoja de datos y las figuras 13, 14 y 15 sobre
 * el funcionamiento de de INVRT).
 * @note La inversión de polaridad a menudo se establece de acuerdo a si un controlador externo de tipo N (no debe usar INVRT) o controlador externo
 * tipo P / conexión directa (debe usar INVRT) se utiliza. La mayoría de los breakout boards tienen solo una resistencia de 220[Ω] entre las salidas PWM, lo cual
 * es útil cuando se alimentan LEDs. El rail V+ de la mayoría de los breakout boards se puede conectar a un condensador de desacoplamiento de 1000[μF] @ 10[V], el cual
 * típicamente ya está instalado, y son para reducir los picos de voltaje y el rebote de tierra durante los cambios de fase al inicio o final de la fase alta de PWM,
 * cuando muchos dispositivos de canal están conectados entre sí. Consulte https://forums.adafruit.com/viewtopic.php?f=8&t=127421 y https://forums.adafruit.com/viewtopic.php?f=8&t=162688
 * para obtener información sobre la instalación un condensador de desacoplamiento si surge la necesidad.
 */
typedef enum PCA9685_OutputEnabledMode {
    PCA9685_OutputEnabledMode_Normal,     // Cuando OE está habilitado/LOW, los canales emiten una señal normal, útil para controladores externos de tipo N (predeterminado).
    PCA9685_OutputEnabledMode_Inverted,   // Cuando OE está habilitado/LOW, los canales emiten una señal invertida, útil para controladores externos de tipo P o conexión directa.
    PCA9685_OutputEnabledMode_Count,            // Para uso interno únicamente.
    PCA9685_OutputEnabledMode_Undefined = -1    // Para uso interno únicamente.
} PCA9685_OutputEnabledMode;

/**
 * @brief Enumeración de modo de salida de driver (Output-not-enabled/active-low-OE-pin=HIGH) (consulte la sección 7.4 de la hoja de datos sobre el uso correcto de OUTNE).
 * @note El pin Active-low-OE se usa generalmente para sincronizar múltiples dispositivos PCA9685, pero también se puede usar como una señal de control de atenuación externa (dimming).
 */
typedef enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,          // Cuando OE está deshabilitado/HIGH, los canales emiten una señal baja (predeterminado).
    PCA9685_OutputDisabledMode_High,        // Cuando OE está deshabilitado/HIGH, los canales emiten una señal alta (solo disponible en modo totem-pole).
    PCA9685_OutputDisabledMode_Floating,    // Cuando OE está deshabilitado/HIGH, las salidas de canal pasan a un estado flotante (alta impedancia), que puede refinarse aún más mediante resistencias externas pull-up / pull-down.

    PCA9685_OutputDisabledMode_Count,           // Para uso interno únicamente.
    PCA9685_OutputDisabledMode_Undefined = -1   // Para uso interno únicamente.
} PCA9685_OutputDisabledMode;

/**
 * @brief Estrategia de actualización de canales utilizada cuando se actualizan varios canales simultáneamente.
 */
typedef enum PCA9685_ChannelUpdateMode {
    PCA9685_ChannelUpdateMode_AfterStop,          // Las actualizaciones del canal se confirman después de la señal de STOP de transmisión completa (predeterminado).
    PCA9685_ChannelUpdateMode_AfterAck,           // Las actualizaciones de canal se confirman después de la señal ACK de actualización de canal individual.
    PCA9685_ChannelUpdateMode_Count,              // Para uso interno únicamente.
    PCA9685_ChannelUpdateMode_Undefined = -1     // Para uso interno únicamente.
} PCA9685_ChannelUpdateMode;

/**
 * @brief Enumeración de esquema de equilibrio de fases basado en software.
 * @note  El equilibrio de fase basado en software intenta mitigar aún más el rebote de tierra y picos de voltaje durante los cambios de fase al inicio o final de la
 * fase alta de PWM al cambiar el primer flanco de cada fase alta sucesiva de PWM en cierta cantidad. Esto ayuda a que cada sumidero de corriente ocurra en todo el rango del
 * ciclo de trabajo en lugar de todos juntos al mismo tiempo. El equilibrio de fase basado en software puede resultar útil en determinadas situaciones, pero en la práctica ha
 * sido origen de muchos problemas, incluido el caso en el que el PCA9685 omite un ciclo entre cambios de PWM cuando el flanco anterior / posterior se mueva más allá de
 * cierto punto. Si bien es posible que se revise esta idea en el futuro, por ahora estamos contentos dejando None como predeterminado y limitando el cambio que aplica la opción Linear.
 */
typedef enum PCA9685_PhaseBalancer {
    PCA9685_PhaseBalancer_None,       // Deshabilita el equilibrio de fase basado en software, confiando en el hardware instalado para manejar la corriente entrante (sink) (predeterminado).
    PCA9685_PhaseBalancer_Linear,     // Utiliza un equilibrio de fase lineal basado en software, con cada canal a 16 pasos preestablecidos (fuera del rango de valores de 4096/12 bits) del canal anterior (puede causar parpadeo de LED / ciclos saltados en cambios de PWM).
    PCA9685_PhaseBalancer_Count,          // Para uso interno únicamente.
    PCA9685_PhaseBalancer_Undefined = -1  // Para uso interno únicamente.
} PCA9685_PhaseBalancer;

/**
 * @brief Estructura de datos de tipo PCA9685, para cada módulo o familia de módulos a utilizar (subdireccionamiento o direccionamiento global LED All Call)
 */
typedef struct PCA9685{
  uint8_t i2cAddress_w;                       // Dirección I²C del módulo en modo escritura
  uint8_t i2cAddress_r;                       // Dirección I²C del módulo en modo lectura
  PCA9685_OutputDriverMode driverMode;        // Modo de driver de salida Output driver mode
  PCA9685_OutputEnabledMode enabledMode;      // Modo de salida OE enabled
  PCA9685_OutputDisabledMode disabledMode;    // Mode de salida OE disabled
  PCA9685_ChannelUpdateMode updateMode;       // Modo de actualización de canales
  PCA9685_PhaseBalancer phaseBalancer;        // Esquema de balanceo de fases
  bool isProxyAddresser;                      // Bandera de direccionamiento Proxy (deshabilita algunas funcionalidades)
  uint8_t lastI2CError;                       // Last module i2c error PENDIENTE
  char device_name[16];                       // Nombre del dispositivo (opcional)
} PCA9685;

/**
 * Macros de ciclos de trabajo comunes
 */
#define PCA9685_DUTY_25     1024
#define PCA9685_DUTY_50     2048    
#define PCA9685_DUTY_75     3072


/**
 * @brief Valor especial para registros ALLLED PENDIENTE
 */
#define PCA9685_ALLLED_CHANNEL          (-1)



//PENDIENTES POR SER DE DIRECCIONAMIENTO
//#define PCA9685_I2C_BASE_MODULE_ADDRESS 0x40
//#define PCA9685_I2C_BASE_MODULE_ADRMASK 0x3F
//#define PCA9685_I2C_BASE_PROXY_ADDRESS  0xE0
//#define PCA9685_I2C_BASE_PROXY_ADRMASK  0xFE


/**
 * Prototipos de funciones
 */
// Funciones básicas
bool PCA9685_init(PCA9685 *modulo, uint8_t address);

#ifdef PCA9685_DEBUG
void PCA9685_printDeviceDetails(PCA9685 *modulo);
#endif

void PCA9685_resetDevices(void);
bool PCA9685_isConnected(PCA9685 *modulo);

char *PCA9685_getDeviceName(PCA9685 *modulo);
void PCA9685_setDeviceName(PCA9685 *modulo, const char *name);

// Funciones de lectura/escritura de registros del PCA9685
void PCA9685_writeRegister(PCA9685 *modulo, uint8_t reg, uint8_t val);
uint8_t PCA9685_readRegister(PCA9685 *modulo, uint8_t reg);

// Funciones de establecimiento de condiciones de operación 
void PCA9685_setFrequency(PCA9685 *modulo, float pwm_freq);
void PCA9685_setChannelOn(PCA9685 *modulo, uint8_t channel);
void PCA9685_setChannelOff(PCA9685 *modulo, uint8_t channel);    
void PCA9685_setChannelPWM(PCA9685 *modulo, uint8_t channel, uint16_t duty_cycle);
void PCA9685_setChannelsPWM(PCA9685 *modulo, uint8_t starting_channel, uint8_t num_channels, uint16_t *duty_cycles);
void PCA9685_setAllChannelsPWM(PCA9685 *modulo, uint16_t duty_cycle);
uint16_t PCA9685_getChannelPWM(PCA9685 *modulo, uint8_t channel);
void PCA9685_getPhaseCycle(PCA9685 *modulo, uint8_t channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);
void PCA9685_writeChannelBegin(PCA9685 *modulo, uint8_t channel);
void PCA9685_writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
void PCA968_useExternalClock(PCA9685 *modulo);
void PCA968_useInternalClock(PCA9685 *modulo);

// Funciones de configuración
PCA9685_OutputDriverMode PCA9685_getOutputDriverMode(PCA9685 *modulo);
PCA9685_OutputEnabledMode PCA9685_getOutputEnabledMode(PCA9685 *modulo);
PCA9685_OutputDisabledMode PCA9685_getOutputDisabledMode(PCA9685 *modulo);
PCA9685_ChannelUpdateMode PCA9685_getChannelUpdateMode(PCA9685 *modulo);
PCA9685_PhaseBalancer PCA9685_getPhaseBalancer(PCA9685 *modulo);


// Funciones básicas de comunicación I²C
inline void PCA9685_i2c_start();
inline void PCA9685_i2c_stop();
inline void PCA9685_i2c_restart();
inline uint8_t PCA9685_i2c_writeByte(uint8_t dato);
inline uint8_t PCA9685_i2c_readByte(bool ack);

//

#ifdef PCA9685_DEBUG
static const char PCA9685_outputDriverMode_0[] = "open drain";
static const char PCA9685_outputDriverMode_1[] = "totem-pole (default)";
static const char * const PCA9685_outputDriverMode_str_P[] = {PCA9685_outputDriverMode_0,PCA9685_outputDriverMode_1};

static const char PCA9685_outputEnabledMode_0[] = "normal (default)";
static const char PCA9685_outputEnabledMode_1[] = "inverted";
static const char * const PCA9685_outputEnabledMode_str_P[] = {PCA9685_outputEnabledMode_0,PCA9685_outputEnabledMode_1};

static const char PCA9685_outputDisabledMode_0[] = "low (default)";
static const char PCA9685_outputDisabledMode_1[] = "high";
static const char PCA9685_outputDisabledMode_2[] = "floating";
static const char * const PCA9685_outputDisabledMode_str_P[] = {PCA9685_outputDisabledMode_0,PCA9685_outputDisabledMode_1,PCA9685_outputDisabledMode_2};

static const char PCA9685_channelUpdateMode_0[] = "after stop (default)";
static const char PCA9685_channelUpdateMode_1[] = "after ack";
static const char * const PCA9685_channelUpdateMode_str_P[] = {PCA9685_channelUpdateMode_0,PCA9685_channelUpdateMode_1};

static const char PCA9685_phaseBalancer_0[] = "none (default)";
static const char PCA9685_phaseBalancer_1[] = "linear";
static const char * const PCA9685_phaseBalancer_str_P[] = {PCA9685_phaseBalancer_0,PCA9685_phaseBalancer_1};

#endif


#endif  /*PCA9685_h*/