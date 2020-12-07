/**
  * @file PCA9685.h
  * @brief Librería para control de módulos i2c-pwm de 16 canales PCA9685
  * @author Ing. José Roberto Parra Trewartha (uedsoldier1990@gmail.com)
  * @version 1.0
*/

#include <stdint.h>
#include <stdbool.h>

#ifndef PCA9685_H
#define PCA9685_H

/**
  * @brief Macro de selección de módulo i2c a utilizar. El número 0 indica i2c emulado por software.
*/
#define PCA8695_I2C_MODULE  1

/**
  * @brief Macro de selección de modalidad de depuración mediante puerto serie. Comentar en caso de no requerirse esta funcionalidad
*/
#define PCA8695_DEBUG

/**
 * @brief Macro de frecuencia de oscilador del módulo. Por defecto es 25 [MHz]
 */
#define PCA8695_OSC_CLOCK 25000000UL  


/**
 * @brief Canal mínimo para un módulo PCA9685
 */
#define PCA9685_MIN_CHANNEL         0

/**
 * @brief Número total de canales para un módulo PCA9685
 */
#define PCA9685_CHANNEL_COUNT       16

/**
 * @brief Canal máximo para un módulo PCA9685
 */
#define PCA9685_MAX_CHANNEL         (PCA9685_CHANNEL_COUNT-1)

/**
 * @brief Valor especial para registros ALLLED
 */
#define PCA9685_ALLLED_CHANNEL          -1                  

//PENDIENTES POR SER DE DIRECCIONAMIENTO
#define PCA9685_I2C_BASE_MODULE_ADDRESS (uint8_t)0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK (uint8_t)0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  (uint8_t)0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  (uint8_t)0xFE

// Registros internos del PCA9685
#define PCA9685_MODE1_REG               (uint8_t)0x00
#define PCA9685_MODE2_REG               (uint8_t)0x01
#define PCA9685_SUBADR1_REG             (uint8_t)0x02
#define PCA9685_SUBADR2_REG             (uint8_t)0x03
#define PCA9685_SUBADR3_REG             (uint8_t)0x04
#define PCA9685_ALLCALL_REG             (uint8_t)0x05
#define PCA9685_LED0_REG                (uint8_t)0x06          // Start of LEDx regs, 4B per reg, 2B on phase, 2B off phase, little-endian
#define PCA9685_PRESCALE_REG            (uint8_t)0xFE
#define PCA9685_ALLLED_REG              (uint8_t)0xFA

// Mode1 register values
#define PCA9685_MODE1_RESTART           (uint8_t)0x80
#define PCA9685_MODE1_EXTCLK            (uint8_t)0x40
#define PCA9685_MODE1_AUTOINC           (uint8_t)0x20
#define PCA9685_MODE1_SLEEP             (uint8_t)0x10
#define PCA9685_MODE1_SUBADR1           (uint8_t)0x08
#define PCA9685_MODE1_SUBADR2           (uint8_t)0x04
#define PCA9685_MODE1_SUBADR3           (uint8_t)0x02
#define PCA9685_MODE1_ALLCALL           (uint8_t)0x01

// Mode2 register values
#define PCA9685_MODE2_OUTDRV_TPOLE      (uint8_t)0x04
#define PCA9685_MODE2_INVRT             (uint8_t)0x10
#define PCA9685_MODE2_OUTNE_TPHIGH      (uint8_t)0x01
#define PCA9685_MODE2_OUTNE_HIGHZ       (uint8_t)0x02
#define PCA9685_MODE2_OCH_ONACK         (uint8_t)0x08

#define PCA9685_SW_RESET                (uint8_t)0x06          // Sent to address 0x00 to reset all devices on Wire line
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
    PCA9685_OutputDriverMode_OpenDrain,         // Salidas del módulo configuradas en drenaje abierto (open-drain) con un límite de 400[mA] @5[V] de corriente en sumidero (sink), útil para LEDs y servomotores de baja potencia
    PCA9685_OutputDriverMode_TotemPole,         // Salidas del módulo configuradas en totem-pole (aka push-pull) con un límite de 400[mA] @5[V] de corriente en sumidero (sink) y 160[mA] de corriente suministrada (source), útil para drivers externos (default)

    PCA9685_OutputDriverMode_Count,             // Para uso interno únicamente
    PCA9685_OutputDriverMode_Undefined = -1     // Para uso interno únicamente
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
    PCA9685_OutputEnabledMode_Normal,     // Cuando OE está habilitado/LOW, los canales emiten una señal normal, útil para controladores externos de tipo N (predeterminado)
    PCA9685_OutputEnabledMode_Inverted,   // Cuando OE está habilitado/LOW, los canales emiten una señal invertida, útil para controladores externos de tipo P o conexión directa

    PCA9685_OutputEnabledMode_Count,            // Para uso interno únicamente
    PCA9685_OutputEnabledMode_Undefined = -1    // Para uso interno únicamente
} PCA9685_OutputEnabledMode;

/**
 * @brief Enumeración de modo de salida de driver (Output-not-enabled/active-low-OE-pin=HIGH) (consulte la sección 7.4 de la hoja de datos sobre el uso correcto de OUTNE).
 * @note El pin Active-low-OE se usa generalmente para sincronizar múltiples dispositivos PCA9685, pero también se puede usar como una señal de control de atenuación externa (dimming).
 */
typedef enum PCA9685_OutputDisabledMode {
    PCA9685_OutputDisabledMode_Low,          // Cuando OE está deshabilitado/HIGH, los canales emiten una señal baja (predeterminado)
    PCA9685_OutputDisabledMode_High,        // Cuando OE está deshabilitado/HIGH, los canales emiten una señal alta (solo disponible en modo totem-pole)
    PCA9685_OutputDisabledMode_Floating,    // Cuando OE está deshabilitado/HIGH, las salidas de canal pasan a un estado flotante (alta impedancia), que puede refinarse aún más mediante resistencias externas pull-up / pull-down

    PCA9685_OutputDisabledMode_Count,           // Para uso interno únicamente
    PCA9685_OutputDisabledMode_Undefined = -1   // Para uso interno únicamente
} PCA9685_OutputDisabledMode;

/**
 * @brief Estrategia de actualización de canales utilizada cuando se actualizan varios canales simultáneamente.
 */
typedef enum PCA9685_ChannelUpdateMode {
  PCA9685_ChannelUpdateMode_AfterStop,          // Las actualizaciones del canal se confirman después de la señal de STOP de transmisión completa (predeterminado)
  PCA9685_ChannelUpdateMode_AfterAck,           // Las actualizaciones de canal se confirman después de la señal ACK de actualización de canal individual

  PCA9685_ChannelUpdateMode_Count,              // Para uso interno únicamente
   PCA9685_ChannelUpdateMode_Undefined = -1     // Para uso interno únicamente
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
    PCA9685_PhaseBalancer_None,       // Deshabilita el equilibrio de fase basado en software, confiando en el hardware instalado para manejar la corriente entrante (sink) (predeterminado)
    PCA9685_PhaseBalancer_Linear,     // Utiliza un equilibrio de fase lineal basado en software, con cada canal a 16 pasos preestablecidos (fuera del rango de valores de 4096/12 bits) del canal anterior (puede causar parpadeo de LED / ciclos saltados en cambios de PWM)

    PCA9685_PhaseBalancer_Count,          // Para uso interno únicamente
    PCA9685_PhaseBalancer_Undefined = -1  // Para uso interno únicamente
} PCA9685_PhaseBalancer;

/**
 * Prototipos de funciones
 */
void PCA9685_init(uint8_t add, uint8_t mode);
void PCA9685_resetDevices(void);

PCA9685_OutputDriverMode getOutputDriverMode(void);
PCA9685_OutputEnabledMode getOutputEnabledMode(void);
PCA9685_OutputDisabledMode getOutputDisabledMode(void);
PCA9685_ChannelUpdateMode getChannelUpdateMode(void);
PCA9685_PhaseBalancer getPhaseBalancer(void);

void PCA9685_setPWMFrequency(float pwm_freq);
void PCA9685_setChannelOn(uint16_t channel);
void PCA9685_setChannelOff(uint16_t channel);    
void PCA9685_setChannelPWM(uint16_t channel, uint16_t duty_cycle);
void PCA9685_setChannelsPWM(uint16_t starting_channel, uint16_t num_channels, uint16_t *duty_cycles);
void PCA9685_setAllChannelsPWM(uint16_t duty_cycle);
uint16_t PCA9685_getChannelPWM(uint16_t channel);


void getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);
void writeChannelBegin(int channel);
void writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
void writeChannelEnd(void);


/**
 * Variables internas
 */
static uint8_t _i2cAddress;                                       // Module's i2c address (default: B000000) PENDIENTE
static PCA9685_OutputDriverMode _driverMode;                  // Modo de driver de salida Output driver mode
static PCA9685_OutputEnabledMode _enabledMode;                // Modo de salida OE enabled
static PCA9685_OutputDisabledMode _disabledMode;              // Mode de salida OE disabled
static PCA9685_ChannelUpdateMode _updateMode;                 // Modo de actualización de canales
static PCA9685_PhaseBalancer _phaseBalancer;                  // Esquema de balanceo de fases
static bool _isProxyAddresser;                                // Bandera de direccionamiento Proxy (deshabilita algunas funcionalidades)
static uint8_t _lastI2CError;                                        // Last module i2c error PENDIENTE





#endif  /*PCA9685_h*/