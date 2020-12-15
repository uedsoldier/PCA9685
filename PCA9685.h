/**
  * @file PCA9685.h
  * @brief Librería para control de módulos I²C-PWM de 16 canales PCA9685
  * @author Ing. José Roberto Parra Trewartha (uedsoldier1990@gmail.com)
  * @version 1.0
*/

#ifndef PCA9685_H
#define PCA9685_H

/**
 * @brief Número total de dispositivos PCS9685 presentes en el bus I²C. Máximo 6 líneas de direccionamiento, para un total de 62 (2⁶-2) dispositivos.
*/
#define PCA9685_DEVICE_COUNT 1

/**
  * @brief Macro de selección de módulo I²C a utilizar. El número 0 indica I²C emulado por software.
*/
#define PCA9685_I2C_MODULE  1

/**
  * @brief Macro de selección de modalidad de depuración mediante puerto serie (tanto por hardware como por software). Comentar en caso de no requerirse esta funcionalidad
*/
#define PCA9685_DEBUG

/**
 * @brief Macro de frecuencia de oscilador del módulo. Por defecto es 25 [MHz]
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

// Dirección I²C básica del PCA9685, en formato de byte completo. Modo escritura (R/W = 0). Tomar en cuenta sus 6 bits de direccionamiento [A5.0].
#define PCA9685_I2C_ADDRESS     0x80

// Dirección I²C general de todos los PCA9685 en el bus, en formato de byte completo. Modo escritura (R/W = 0) 
#define PCA9685_LED_ALLCALL     0xE0

// Dirección I²C para subdireccionamiento de grupo 1, en formato de byte completo. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_1       0xE2

// Dirección I²C para subdireccionamiento de grupo 2, en formato de byte completo. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_2       0xE4

// Dirección I²C para subdireccionamiento de grupo 3, en formato de byte completo. Modo escritura (R/W = 0) 
#define PCA9685_SUBADDR_3       0xE8

// Dato I²C para reset de módulos PCA9685 por software (SWRST). Se debe mandar después de direccionar a todos los dispositivos por medio de GENERAL ADDRESS CALL (dirección 0x00 I²C) 
#define PCA9685_SOFTWARE_RESET  0x06

// Registros internos del PCA9685
#define PCA9685_MODE1_REG               0x00    // Registro de modalidad 1
#define PCA9685_MODE2_REG               0x01    // Registro de modalidad 2
#define PCA9685_SUBADR1_REG             0x02    // Registro de subdireccionamiento 1
#define PCA9685_SUBADR2_REG             0x03    // Registro de subdireccionamiento 2
#define PCA9685_SUBADR3_REG             0x04    // Registro de subdireccionamiento 3
#define PCA9685_ALLCALL_REG             0x05    // Registro de direccionamiento LED All Call
#define PCA9685_LED0_REG                0x06    // Inicio de los registros LEDx, 4B per reg, 2B on phase, 2B off phase, formato little-endian
#define PCA9685_PRESCALE_REG            0xFE
#define PCA9685_ALLLED_REG              0xFA

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
 * @brief Valor especial para registros ALLLED PENDIENTE
 */
#define PCA9685_ALLLED_CHANNEL          -1  





//PENDIENTES POR SER DE DIRECCIONAMIENTO
#define PCA9685_I2C_BASE_MODULE_ADDRESS (uint8_t)0x40
#define PCA9685_I2C_BASE_MODULE_ADRMASK (uint8_t)0x3F
#define PCA9685_I2C_BASE_PROXY_ADDRESS  (uint8_t)0xE0
#define PCA9685_I2C_BASE_PROXY_ADRMASK  (uint8_t)0xFE


PCA9685_OutputDriverMode PCA9685_getOutputDriverMode();
PCA9685_OutputEnabledMode PCA9685_getOutputEnabledMode();
PCA9685_OutputDisabledMode PCA9685_getOutputDisabledMode();
PCA9685_ChannelUpdateMode PCA9685_getChannelUpdateMode();
PCA9685_PhaseBalancer PCA9685_getPhaseBalancer();

/**
 * Prototipos de funciones
 */
void PCA9685_init(uint8_t address, uint8_t mode);
void PCA9685_resetDevices();


// Funciones de establecimiento de condiciones de operación 
void PCA9685_setFrequency(float pwm_freq);
void PCA9685_setChannelOn(uint16_t channel);
void PCA9685_setChannelOff(uint16_t channel);    
void PCA9685_setChannelPWM(uint16_t channel, uint16_t duty_cycle);
void PCA9685_setChannelsPWM(uint16_t starting_channel, uint16_t num_channels, uint16_t *duty_cycles);
void PCA9685_setAllChannelsPWM(uint16_t duty_cycle);
uint16_t PCA9685_getChannelPWM(uint16_t channel);

// Funciones básicas de comunicación I²C
void PCA9685_i2c_start();
void PCA9685_i2c_stop();
void PCA9685_i2c_restart();
void PCA9685_i2c_writeByte(uint8_t dato);
uint8_t PCA9685_i2c_readByte(bool ack);

void PCA9685_getPhaseCycle(int channel, uint16_t pwmAmount, uint16_t *phaseBegin, uint16_t *phaseEnd);
void PCA9685_writeChannelBegin(int channel);
void PCA9685_writeChannelPWM(uint16_t phaseBegin, uint16_t phaseEnd);
void PCA9685_writeChannelEnd();





void PCA9685_writeRegister(uint8_t reg, uint8_t val);
uint8_t PCA9685_readRegister(uint8_t reg);


/**
 * Variables internas
 */
static uint8_t _i2cAddress;                                   // Module's i2c address (default: B000000) PENDIENTE
static PCA9685_OutputDriverMode _driverMode;                  // Modo de driver de salida Output driver mode
static PCA9685_OutputEnabledMode _enabledMode;                // Modo de salida OE enabled
static PCA9685_OutputDisabledMode _disabledMode;              // Mode de salida OE disabled
static PCA9685_ChannelUpdateMode _updateMode;                 // Modo de actualización de canales
static PCA9685_PhaseBalancer _phaseBalancer;                  // Esquema de balanceo de fases
static bool _isProxyAddresser;                                // Bandera de direccionamiento Proxy (deshabilita algunas funcionalidades)
static uint8_t _lastI2CError;                                        // Last module i2c error PENDIENTE





#endif  /*PCA9685_h*/