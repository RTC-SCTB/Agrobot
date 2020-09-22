#pragma once 
#include "stdint.h"
/* Файл фиксированных параметров */

#define SERVO_CENTRAL_POSITION  350  // центральное положение серв (1500 мкс)
#define SERVO_MIN_POSITION  250  // границы положения
#define SERVO_MAX_POSITION  450
#define SERVO_FREQ  60  // частота ШИМ (~57Гц)
#define SERVO_DELAY 3 // задержка сервы при движении - влияет на скорость в рабочем режиме

// выводы драйвера моторов
#define MOTOR_ENABLE_A_CH     10  // пин разрешающий работу мотора A
#define MOTOR_ENABLE_B_CH     9   // пин разрешающий работу мотора B
#define MOTOR_PWM_A_CH        5  // канал ШИМа мотора А
#define MOTOR_PWM_B_CH        6  // канал ШИМа мотора B
#define MOTOR_PWM_INVERSE_A_CH   8   // пин инвертирующий ШИМ на канале мотора А
#define MOTOR_PWM_INVERSE_B_CH   4  // пин инвертирующий ШИМ на канале мотора B

// выводы джойстика
#define JOY_DAT_CH  A3
#define JOY_CMD_CH  A2
#define JOY_SEL_CH  2
#define JOY_CLK_CH  3

// вывод дисплея
#define DISPLAY_RESET_CH  4

// работа со звуком
#define BUZZER_CH 7  // вывод, подключённый к динамику

// работа с АЦП
#define ADC_VOLTAGE_CH  A7  // канал считывания напряжения
#define ADC_MAX_COUNT    15  // задержка преобразования АЦП
#define NUMBER_OF_ADC_MEASUREMENTS 10 //количество измерений для взятия среднего напряжения
#define MAX_SUPPLY_VOLTAGE 8.4 //Два аккумулятора формата 18650 выдют максимально 8.4 вольта
#define MIN_SUPPLY_VOLTAGE 6.0 //Мимальное рабочее напряжение для 2ух аакумуляторов 18650 2.75В * 2 = ~6В
#define ADC_MAX_VOLT_VALUE 1020 //значение АЦП при напряжении источника MAX_SUPPLY_VOLTAGE
#define ADC_MIN_VOLT_VALYE 735 //значение АЦП при напряжении источника MIN_SUPPLY_VOLTAGE 

