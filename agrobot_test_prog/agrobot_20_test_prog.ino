#include "config.h"
#include "stdint.h"
#include <SPI.h>
#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS2X_lib.h>

/*
 * Прошивка для агробота для тестирования платы agrobot2.0
 * В этой прошивке отрабатывают всеканалы и возможности платы. Проводится проверка
 * на непропай и работоспособность микросхем.
 * 
 * Управление через геймпад:
 * (Все каналы серв управляются попарно: 0 и 1, 2 и 3 и т.д.)
 * -стрелочками слева управляются: вверх-вниз 0 и 1 каналы серво, вправо-влево 2 и 3 каналы серво,
 * -кнопками справо: тр-ик и крест 4 и 5 каналы серво, круг и кв-т 6-7 каналы серво,
 * -кнопки R1 и L1 - мотор А форвард и реверс,
 * -кнопки R2 и L2 - мотор В форвард и реверс,
 * -кнопки R3 и L3 - светодиод и динамикю.
 */

volatile Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(); //инициализация i2c для pca с адресом 0x40
volatile Adafruit_SSD1306 display(IMAGE_WIDTH, IMAGE_HEIGHT, &Wire, DISPLAY_RESET_CH); // инициализация дисплея
volatile PS2X ps2x;  // cоздание экземпляра класса для джойстика

float voltage = 0;

void motorSetup()   // инициализация моторов
{
  pinMode(MOTOR_ENABLE_A_CH, OUTPUT);
  pinMode(MOTOR_ENABLE_B_CH, OUTPUT);
  pinMode(MOTOR_PWM_A_CH, OUTPUT);
  pinMode(MOTOR_PWM_B_CH, OUTPUT);
  pinMode(MOTOR_PWM_INVERSE_A_CH, OUTPUT);
  pinMode(MOTOR_PWM_INVERSE_B_CH, OUTPUT);
}

void buzzerSetup()  // инициализация пищалки
{
  pinMode(BUZZER_CH, OUTPUT);
  noTone(BUZZER_CH);
}

void displaySetup() // инициализация дисплея
{
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // Инициализация I2C для дисплея с адресом 0x3D
  display.display();
  delay(2000); //задержка для инициализации дисплея
  display.clearDisplay(); // очистка дисплея
}

void servoSetup() // инициализация серв
{
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);  // Установка частоты ШИМ
}

void servoCentering() // центрирование серв
{
  pwm.setPWM(0, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(1, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(2, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(3, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(4, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(5, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(6, 0, SERVO_CENTRAL_POSITION);
  pwm.setPWM(7, 0, SERVO_CENTRAL_POSITION);
}

void InfoDisplay(uint16_t CH_01, uint16_t CH_23, uint16_t CH_45, uint16_t CH_67, bool LedState) //вывод состояния на дисплей
{
  display.clearDisplay();
  display.setTextSize(1.5);
  display.setTextColor(WHITE);
  
  display.setCursor(0, 0);
  display.print("Voltage:  ");
  display.println(voltage);
  
  display.setCursor(0, 8);
  display.print("Servo 0-1:  ");
  display.println(CH_01);

  display.setCursor(0, 16);
  display.print("Servo 2-3:  ");
  display.println(CH_23);

  display.setCursor(0, 24);
  display.print("Servo 4-5:  ");
  display.println(CH_45);

  display.setCursor(0, 32);
  display.print("Servo 6-7:  ");
  display.println(CH_67);

  display.setCursor(0, 40);
  display.print("Led State:  ");
  if(LedState) display.print("ON");
  else display.print("OFF");
  
  display.display();    
}

//Запуск двигателей 
void setSpeedRight(int16_t mspeed)  // первый двигатель - А
{
  if (mspeed > 0)   // если заданная скорость больше нуля, то задаем Прямой ШИМ без инвертирования
  {
    analogWrite(MOTOR_PWM_A_CH, 255);
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  }
  else    // если меньше, то инвертируем направление
  {
    digitalWrite(MOTOR_PWM_INVERSE_A_CH, HIGH);
    analogWrite(MOTOR_PWM_A_CH, 0);
  }
  analogWrite(MOTOR_ENABLE_A_CH, abs(mspeed));    
}


void setSpeedLeft(int16_t mspeed) // второй двигатель - B
{
  if (mspeed > 0)
  {
    digitalWrite(MOTOR_PWM_INVERSE_B_CH, HIGH);
    analogWrite(MOTOR_PWM_B_CH, 0);
  }
  else
  {
    analogWrite(MOTOR_PWM_B_CH, 255);
    digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  }
  analogWrite(MOTOR_ENABLE_B_CH, abs(mspeed));   
}


void stopMotors()   // остановка двигателей
{
  analogWrite(MOTOR_PWM_A_CH, 0);
  digitalWrite(MOTOR_PWM_INVERSE_A_CH, LOW);
  digitalWrite(MOTOR_PWM_INVERSE_B_CH, LOW);
  analogWrite(MOTOR_PWM_B_CH, 0);
}


void beep(uint8_t num, uint16_t tim)
{
  for (uint16_t i = 0; i < num; i++)
  {
    digitalWrite(BUZZER_CH, HIGH);
    delay(tim);
    digitalWrite(BUZZER_CH, LOW);
    delay(50);
  }
}

void adcDataCounter(float* voltage)   // вычисление значения напряжения питания и тока, запись в параметры
{
  static uint8_t adcCount = ADC_MAX_COUNT;  // ограничение по частоте считывания данных с АЦП
  static float m_voltage = 0;   // доп переменные для того, чтобы не ждать 14 вызовов ф-ии, если пропустили первый 
  if (adcCount >= ADC_MAX_COUNT)    // каждые ADC_MAX_COUNT = 15 раз меняет значения,приходящие в параметрах
  {
    adcCount = 0;
    m_voltage = ADC_VOLT_DIV_CONST * analogRead(ADC_VOLTAGE_CH) * ADC_UAREF / ADC_MAX; //вычисление напряжения на выходе буффера
  }
  *voltage = m_voltage;
  adcCount++;
}

bool workFSM()    // рабочий режим
{
  static bool LedState = false;
  static uint16_t CH_01 = SERVO_CENTRAL_POSITION;
  static uint16_t CH_23 = SERVO_CENTRAL_POSITION;
  static uint16_t CH_45 = SERVO_CENTRAL_POSITION;
  static uint16_t CH_67 = SERVO_CENTRAL_POSITION; 
  
  InfoDisplay(CH_01, CH_23, CH_45, CH_67, LedState);
  
  static enum
  {
    LEAD,  // управляющий режим
    FORWARD_CH_01,  // Первая и вторая серва вперед
    BACK_CH_01,     // Первая и вторая серва назад
    FORWARD_CH_23,  // Вторая и третья серва вперед 
    BACK_CH_23,     // Вторая и третья серва назад
    FORWARD_CH_45,  // Четвертая и пятая серва вперед
    BACK_CH_45,     // Четвертая и пятая серва назад
    FORWARD_CH_67,  // Шестая седьмая серва вперед
    BACK_CH_67,     // Шестая седьмая серва назад
    MOTOR_A_FORWARD,// Мотор А вперед
    MOTOR_A_BACK,   // Мотор А назад
    MOTOR_B_FORWARD,// Мотор В вперед
    MOTOR_B_BACK,   // Мотор В назад
    LED,            // Изменить состояние светодиода
    ALARM,          // Звуковой сигнал
    NOTHING         // остановка + ничего не делать
  } state = LEAD;

  switch(state)
  {
    case LEAD:
      if (!((ps2x.Button(PSB_R2) || ps2x.Button(PSB_L2) ||    
             ps2x.Button(PSB_R1) || ps2x.Button(PSB_L1)))) { state = NOTHING; }
      if (ps2x.Button(PSB_PAD_UP)) { state = FORWARD_CH_01; }
      if (ps2x.Button(PSB_PAD_DOWN)) { state = BACK_CH_01; }
      if (ps2x.Button(PSB_PAD_LEFT)) { state = FORWARD_CH_23; }
      if (ps2x.Button(PSB_PAD_RIGHT)) { state = BACK_CH_23; }
      if (ps2x.Button(PSB_TRIANGLE)) { state = FORWARD_CH_45; }
      if (ps2x.Button(PSB_CROSS)) { state = BACK_CH_45; }
      if (ps2x.Button(PSB_SQUARE)) { state = FORWARD_CH_67; }
      if (ps2x.Button(PSB_CIRCLE)) { state = BACK_CH_67; }
      if (ps2x.ButtonPressed(PSB_R2)) { state = MOTOR_A_BACK; }
      if (ps2x.ButtonPressed(PSB_L2)) { state = MOTOR_A_FORWARD; }       
      if (ps2x.ButtonPressed(PSB_R1)) { state = MOTOR_B_BACK; }
      if (ps2x.ButtonPressed(PSB_L1)) { state = MOTOR_B_FORWARD; }
      if (ps2x.Button(PSB_L3)) { state = LED; }
      if (ps2x.Button(PSB_R3)) { state = ALARM; }
      break;

    case FORWARD_CH_01:
      CH_01 = CH_01 + SERVO_STEP;
      if (CH_01 > SERVO_MAX_POSITION) CH_01 = SERVO_MAX_POSITION;
      if (CH_01 < SERVO_MIN_POSITION) CH_01 = SERVO_MIN_POSITION;
      pwm.setPWM(0, 0, CH_01);
      pwm.setPWM(1, 0, CH_01); 
      state = LEAD; 
      break;

    case BACK_CH_01:
      CH_01 = CH_01 - SERVO_STEP;
      if (CH_01 > SERVO_MAX_POSITION) CH_01 = SERVO_MAX_POSITION;
      if (CH_01 < SERVO_MIN_POSITION) CH_01 = SERVO_MIN_POSITION;
      pwm.setPWM(0, 0, CH_01);
      pwm.setPWM(1, 0, CH_01); 
      state = LEAD; 
      break;

    case FORWARD_CH_23:
      CH_23 = CH_23 + SERVO_STEP;
      if (CH_23 > SERVO_MAX_POSITION) CH_23 = SERVO_MAX_POSITION;
      if (CH_23 < SERVO_MIN_POSITION) CH_23 = SERVO_MIN_POSITION;
      pwm.setPWM(2, 0, CH_23);
      pwm.setPWM(3, 0, CH_23); 
      state = LEAD; 
      break;

    case BACK_CH_23:
      CH_23 = CH_23 - SERVO_STEP;
      if (CH_23 > SERVO_MAX_POSITION) CH_23 = SERVO_MAX_POSITION;
      if (CH_23 < SERVO_MIN_POSITION) CH_23 = SERVO_MIN_POSITION;
      pwm.setPWM(2, 0, CH_23);
      pwm.setPWM(3, 0, CH_23); 
      state = LEAD; 
      break;

    case FORWARD_CH_45:
      CH_45 = CH_45 + SERVO_STEP;
      if (CH_45 > SERVO_MAX_POSITION) CH_45 = SERVO_MAX_POSITION;
      if (CH_45 < SERVO_MIN_POSITION) CH_45 = SERVO_MIN_POSITION;
      pwm.setPWM(4, 0, CH_45);
      pwm.setPWM(5, 0, CH_45); 
      state = LEAD; 
      break;

    case BACK_CH_45:
      CH_45 = CH_45 - SERVO_STEP;
      if (CH_45 > SERVO_MAX_POSITION) CH_45 = SERVO_MAX_POSITION;
      if (CH_45 < SERVO_MIN_POSITION) CH_45 = SERVO_MIN_POSITION;
      pwm.setPWM(4, 0, CH_45);
      pwm.setPWM(5, 0, CH_45); 
      state = LEAD;
      break;

    case FORWARD_CH_67:
      CH_67 = CH_67 + SERVO_STEP;
      if (CH_67 > SERVO_MAX_POSITION) CH_67 = SERVO_MAX_POSITION;
      if (CH_67 < SERVO_MIN_POSITION) CH_67 = SERVO_MIN_POSITION;
      pwm.setPWM(6, 0, CH_67);
      pwm.setPWM(7, 0, CH_67); 
      state = LEAD;
      break;

    case BACK_CH_67:
      CH_67 = CH_67 - SERVO_STEP;
      if (CH_67 > SERVO_MAX_POSITION) CH_67 = SERVO_MAX_POSITION;
      if (CH_67 < SERVO_MIN_POSITION) CH_67 = SERVO_MIN_POSITION;
      pwm.setPWM(6, 0, CH_67);
      pwm.setPWM(7, 0, CH_67); 
      state = LEAD;
      break;

    case MOTOR_A_FORWARD:
      setSpeedRight(MOTOR_SPEED);
      state = LEAD;
      break;

    case MOTOR_A_BACK:
      setSpeedRight(-MOTOR_SPEED);
      state = LEAD;
      break;

    case MOTOR_B_FORWARD:
      setSpeedLeft(MOTOR_SPEED);
      state = LEAD;
      break;

    case MOTOR_B_BACK:
      setSpeedLeft(-MOTOR_SPEED);
      state = LEAD;
      break;
      
    case LED:
      if(LedState){
        digitalWrite(LED_BUILTIN, LOW);
        LedState = false;
      }
      else{
        digitalWrite(LED_BUILTIN, HIGH);
        LedState = true;
      }
      state = LEAD;
      break;
      
    case ALARM:
      beep(4, 300);
      state = LEAD;
      break;
      
    case NOTHING:
      stopMotors();
      state = LEAD;
      break;
  }
}



void setup() 
{
  displaySetup(); // инициализация дисплея
  motorSetup();   // инициализация моторов
  servoSetup();   // инициализация серв
  servoCentering();   // центрирование серв
  buzzerSetup(); // инициализация динамика
  
  pinMode(A4, INPUT_PULLUP);    // подтяжка линий I2C к питанию, мб и не надо 
  pinMode(A5, INPUT_PULLUP);
  
  //установка выводов и настроек: GamePad(clock, command, attention, data, Pressures?, Rumble?) проверка ошибок
  ps2x.config_gamepad(JOY_CLK_CH, JOY_CMD_CH, JOY_SEL_CH, JOY_DAT_CH, JOY_PRESSURES, JOY_RUMBLE); 
  
  beep(1, 500);

  analogReference(EXTERNAL);  // настройка опорного напряжения для АЦП: внешний источник на выводе AREF
}

void loop()
{
  ps2x.read_gamepad(false, 0); // считывание данных с джойстика и установка скорости вибрации
  adcDataCounter(&voltage); // обновляем данные с АЦП
 
  workFSM();
  delay(15);
}
