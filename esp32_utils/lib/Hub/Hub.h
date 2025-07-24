#ifndef DRIVER_ROBOT
#define DRIVER_ROBOT


// Physical definitions

/// Wheels
#define DIAMETRE_RODES_MM       95.0
#define PERIMETRE_RODES_CM      (DIAMETRE_RODES_MM / 10.0 * 3.14159265358979323846)

/// Motors: Configuration 350RPM 12V Motors
#define MOTOR_REDUCCIO          34.0
#define MOTOR_POLSOS_PER_REV    (11.0 * 4 * MOTOR_REDUCCIO)                           // Polsos simples * 4 canvis/simple * reducció
#define MOTOR_POLSOS_PER_CM     (MOTOR_POLSOS_PER_REV / PERIMETRE_RODES_CM)           // Polsos/rev / cm/rev
#define MOTOR_VEL_MAX_RPM       350
#define MOTOR_VEL_MAX_PWM       250
#define MOTOR_VEL_MIN_PWM       140
#define MOTOR_MAX_LINEAL_VEL    1.75



// ESP32 01

/// Motors
#define GPIO_IN1_L           23
#define GPIO_IN2_L           25
#define GPIO_CH1_L           0
#define GPIO_CH2_L           1

#define GPIO_ENC1_L          4
#define GPIO_ENC2_L          16

#define GPIO_IN1_R           18
#define GPIO_IN2_R           19
#define GPIO_CH1_R           2
#define GPIO_CH2_R           3

#define GPIO_ENC1_R          34
#define GPIO_ENC2_R          35


#endif