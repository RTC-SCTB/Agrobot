#pragma once
/* Файл переменной конфигурации 
 * файл должен быть подключен в начале скетча 
 */

// определение всех нужных параметров

#define TIME_VOLTAGE_MEASUREMENT    20  //время между измерениями напряжения. Общее время между обновлением значения нарпяжения равно (TIME_VOLTAGE_MEASUREMENT + 15) * ADC_MAX_COUNT * NUMBER_OF_ADC_MEASUREMENTS

// резмеры изображений
#define IMAGE_WIDTH   128
#define IMAGE_HEIGHT  64

// регулирование скорости
#define MOTOR_SPEED   255 // наибольшее допустимое значение 255

// режимы работы джойстика
#define JOY_PRESSURES false  // аналоговое считывание кнопок  
#define JOY_RUMBLE    false  // вибромотор

#define SERVO_STEP  2   // шаг изменения положения сервы при движении в рабочем режиме, влияет на скорость движения сервы

#if !(defined(JOY_PRESSURES) && defined(JOY_RUMBLE))  // если не продефайнены параметры джойстика
# error "Joystick modes not defined" 
#endif

// подключаем фиксированные параметры, проверку не делаем, т.к. не меняем их
#include "fixed.h"

