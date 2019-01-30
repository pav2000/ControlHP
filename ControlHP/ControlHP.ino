#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

//#define DEMO                               // Признак демонстрации
#define RADIO                              // Признак использования радио модуля
#define VERSION "Ver. leOS2 1.46 020615"   // Текущая верстия

// Конфигурация платы UNO (куда что подключается) --------------------------------
// Прерывания 
#define sensorInterrupt1    0                  // Первый датчик потока адрес прерывания
#define sensorInterrupt2    1                  // Второй датчик потока адрес прерывания
// ЦИФРОВЫЕ Ноги 
#define PIN_FLOW1           2                  // Первый датчик потока ножка на которую вешается датчик Холла
#define PIN_FLOW2           3                  // Второй датчик потока ножка на которую вешается датчик Холла
#define PIN_ST7920_SS1      4                  // ST7920 SS1
#define PIN_ENERGY_COUNT    5                  // Нога на которую заводится сигнал от электросчетчика (подсчет импульсов) используется аппаратный счетчик импульсов
#define PIN_ONE_WIRE_BUS    6                  // DS18b20 Нога на которой весят датчики температуры
#define PIN_ST7920_MOSI     7                  // ST7920 MOSI
#define PIN_ST7920_SCK      8                  // ST7920 SCK
// Беспроодный модуль NRF24L
#define PIN_SPI_CSN         9                  // NRF24L нога CSN выбор режима приема/передача
// Используем аппаратный SPI UNO
#define PIN_SPI_CE          10                 // NRF24L SPI сигнал SS1  – Slave-select 1 LCD
#define PIN_SPI_MOSI        11                 // NRF24L SPI сигнал MOSI – Master-out, Slave-in
#define PIN_SPI_MISO        12                 // NRF24L SPI сигнал MISO – Master-in, Slave out.
#define PIN_SPI_SCK         13                 // NRF24L SPI сигнал SCK  – Serial clock

// АНАЛОГОВЫЕ ноги
#define PIN_ANALOG_I        0                  // Нога на которую заводится ток от датчика (трансформатор тока)
#define PIN_ANALOG_V        1                  // Нога на которую заводится напряжение от датчика (220в)
#define PIN_KEY             16                 // Pin 16 = Analog in 2 Нога куда прицеплена кнопка 
#define PIN_LED_FLOW        17                 // Pin 17 = Analog in 3 Нога куда прицеплено реле потока (если замкнуто то поток есть) - светодиод и реле к ТП 1-2

#define READVCC_CALIBRATION_CONST 1103702L     // Калибровка встроенного АЦП (встроенный ИОН) по умолчанию 1126400 дальше измеряем питание и смотрим на дисплей 1103782L
#define NUM_SCREEN                5            // Число основных экранов
#define CALIBRATION_CONST_220     147.13       // Калибровка датчика напряжения 220 В 
#define CALIBRATION_CONST_220_I   10.03        // Калибровка датчика тока 220 A
#define EE_SAVED_ADDRESS          100          // Адрес записи данных eeprom
#define TIME_WRITE_EEPROM         60           // Период сохранения в eeprom МИНУТЫ!!!
#define eCONST                    1000         // Число импульсов на киловат/час электросчетчика

// Дополнительные библиотеки
#include <OneWire.h>           // OneWire библиотека
#include <leOS2.h>             // Шедуллер задач
#include "U8glib.h"            // Экран ЖКИ 128х64 по SPI
#include "rusFont.h"	       // Ресурс дополнительных фонтов
#include "BitMap.h"	       // Ресурс картинки для вывода на дисплей
#include "emonLib.h"           // Монитор электроэнергии
#include <avr/pgmspace.h>      // библиотека для использования  PROGMEM и F  - строковые константы в rom


 // Радио модуль NRF42l
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
    struct type_packet_NRF24               // Структура передаваемого пакета 32 байта - это максимум
    {
        int id=1;                          // Идентификатор устройства
        int tOut1=-5000,tIn1=-5000;        // Текущие температуры ТП в сотых градуса !!! место экономим
        int PowerFloor=0;                  // Мощность теплого пола  вт
        int tOut2=-5000,tIn2=-5000;        // Текущие температуры ГК в сотых градуса !!! место экономим
        int PowerLand=0;                   // Мощность теплого пола  вт
        int I,U,P;                         // Ток в сотых ампера, напряжение в сотых вольта, мощность в вт.
        char note[12] = "Arduino UNO";     // Примечание не более 11 байт + 0
    };

#ifdef  RADIO    // Радио модуль NRF24
    const unsigned int  Time_RF24_send=5000; // Длительность цикла посылки данных на сервер через радиомодуль (примерно- точность определяется стороживым таймером)
    type_packet_NRF24 packet;                 // пакет данных
    RF24 radio(PIN_SPI_CE,PIN_SPI_CSN);       // Создание структуры 
    const uint64_t pipes[2] = {0xF0F0F0F0E1LL,0xF0F0F0F0D2LL};
    bool send_packet_ok=false;                // признак удачной отправки последнего пакета 
#endif 

// ----------------------------------------------------------------------  
boolean indicator = true;                   // Индикатор потока на дисплее - переменная для его анимации.
int fStart = 0;                             // Какой текущий экран вывода 0-стартовый
volatile byte ScreenResetCount=0;           // Для корректной работы экрана сброса кнопка должна быть нажата а затем отжата
volatile byte count=0;                      // Счетчик времени для записи в eeprom в минутах

const int  TimeMeasurement=9715;            // Длительность цикла измерения мсек  !! неточно дискретность 16 мксек
const int  TimeInterview=150;               // Длительность цикла опроса кнопок мсек
const unsigned int  TimeOneMinutes=58270;   // Длительность минутного цикла 65000 (unsigned int) 59100
const int  TEMPERATURE_PRECISION =12;       // разрешение датчика темературы в битах 9-12 бит
const float calibrationFactor = 0.435;      // Калибровочный коэфициент датчика потока 2-0.439 3-431
const float heat_capacity=4.191;            // Теплоемкость теплоносителя (вода 4.191)
 
volatile unsigned long oldTime=0;           // переменная для расчета interval
volatile long interval;                     // реальный интервал измерения (чуть больше TimeMeasurement особенность leOS2)
volatile long ChartInterval;                // реальный интервал вывода графика (чуть больше TimeChart особенность leOS2)
volatile unsigned long ChartOldTime=0;      // переменная для расчета ChartInterval
char buf[15];                               // буфер для вывода чисел на экран
byte oldKey=0;                              // Предыдущее знаение клавиши 
byte flagKey=1;                             // Флаг нажатия клавиши

// Электросчетчик -----------------------------
volatile unsigned long eCount=0;             // Счетчик импульсов от электросчетчика
volatile float ePower=0;                     // Пиковая мощность за TimeMeasurement
volatile float eEnergy=0;                    // Потребленная энергия
byte pChart[60];                             // Данные графика потеблемой мощности (датчики ток и напряжение)
byte ChartCOP[60];                           // Данные графика COP
volatile float pPower=0;                     // Пиковая мощность за TimeMeasurement
volatile float pEnergy=0;                    // Потребленная энергия

// Первый канал Теплые полы -------------------------------
const float errOut1=0.05;                    // Ошибка первого датчика - вычитается из показаний 0.05
const float errIn1=0.07;                     // Ошибка второго датчика - вычитается из показаний 0.17
byte ThermOut1[8] = {0x28,0xFF,0x37,0x7F,0x4A,0x04,0x00,0xCF};  // ТП обратка адрес датчика DS18B20 28FF377F4A0400CF закругленный конец 
byte ThermIn1 [8] = {0x28,0xBC,0x6B,0x3D,0x06,0x00,0x00,0x01};  // ТП подача адрес датчика DS18B20 28BC6B3D06000001 прямой конец  синий кембрик
volatile unsigned int pulseCount1=0;         // Счетчик импульсов первого датчика
volatile float flowRate1=0.0;                // Поток литров в минуту измеряется раз в минуту
volatile float totalLitres1=0.0;             // Общий объем прошедшей житкости в литрах Пишется во флеш каждый час
volatile float totalPower1=0.0;              // Общий объем тепла
volatile unsigned int count1 = 0;            // Число импульсов от датчика полученное за интервал измерения
volatile float HZ1=0.0;                      // Частота           
volatile float P1=0.0;                       // Мощность за секунду ТП
volatile float tOut1=-50,tIn1=-50;           // текущие температуры,
byte Chart1[60];                             // Данные первого графика (ТП)

// Второй канал Геоконтур ---------------------------
const float errOut2=0.17;                    // Ошибка первого датчика - вычитается из показаний
const float errIn2=0.17;                     // Ошибка второго датчика - вычитается из показаний
//byte ThermOut2 [8]= {0x28,0x34,0x70,0x2b,0x06,0x00,0x00,0x77};  // ТП обратка адрес датчика DS18B20 28FF377F4A0400CF закругленный конец 
// byte ThermIn2 [8] = {0x28,0x63,0x87,0x2b,0x06,0x00,0x00,0x2b};  // ТП подача адрес датчика DS18B20 28BC6B3D06000001 прямой конец
byte ThermOut2 [8]= {0x28,0x63,0x87,0x2b,0x06,0x00,0x00,0x2b};  // ТП обратка адрес датчика DS18B20 метка синия 
byte ThermIn2 [8] = {0x28,0x34,0x70,0x2b,0x06,0x00,0x00,0x77};  // ТП подача адрес датчика DS18B20 метка красная
volatile unsigned int pulseCount2=0;         // Счетчик импульсов первого датчика
volatile float flowRate2=0.0;                // Поток литров в минуту измеряется раз в минуту
volatile float totalLitres2=0.0;             // Общий объем прошедшей житкости в литрах Пишется во флеш каждый час
volatile float totalPower2=0.0;              // Общий объем тепла
volatile unsigned int count2 = 0;            // Число импульсов от датчика полученное за интервал измерения
volatile float HZ2=0.0;                      // Частота           
volatile float P2=0.0;                       // Мощность за секунду 
volatile float tOut2=-50,tIn2=-50;           // текущие температуры
byte Chart2[60];                             // Данные второго графика (ГК)

// Структура для записи в EEPROM
struct type_Counts                            // Тип данных - счетчики
{
    float GlobalEnergyP;                          //  Потребленная энергия с момента старта ТН - не сбрасывается ДАТЧИКИ ТОК И НАПРЯЖЕНИЕ в КИЛОВАТАХ
    float GlobalEnergyG;                          //  Выработанная энергия с момента старта ТН - не сбрасывается ДАТЧИКИ ТОК И НАПРЯЖЕНИЕ в КИЛОВАТАХ
    float YearEnergyP;                            //  Потребленная энергия с начала сезона отопления ТН - сбрасывается в КИЛОВАТАХ
    float YearEnergyG;                            //  Выработанная энергия с начала сезона отопления ТН - сбрасывается в КИЛОВАТАХ
    unsigned long GlobalHour;                     //  Мото часы (секунды) общие с момента старта ТН - не сбрасывается В СЕКУНДАХ
    unsigned long YearHour;                       //  Мото часы (секунды) общие с начала сезона отопления ТН - сбрасывается В СЕКУНДАХ
    unsigned long GlobalCountE;                   //  Потребленная энергия с момента старта ТН - не сбрасывается ЭЛЕКТРОСЧЕТЧИК  В ИМПУЛЬСАХ
    unsigned long YearCountE;                     //  Потребленная энергия с начала сезона отопления ТН - сбрасывается ЭЛЕКТРОСЧЕТЧИК В ИМПУЛЬСАХ
    float GlobalEnergyGeo;                        //  Потребленная энергия из геоконтура с момента старта ТН - не сбрасывается в КИЛОВАТАХ
    float YearEnergyGeo;                          //  Потребленная энергия из геоконтура с момента старта ТН - сбрасывается в КИЛОВАТАХ
}; 
volatile type_Counts CountsSRAM;                   // Рабочая копия счетчиков в памяти
type_Counts CountsEEPROM EEMEM;                    // Копия счетчиков в eeprom - туда пишем

volatile float EnergyP=0,EnergyG=0,EnergyGeo=0;    // промежуточные переменные
volatile unsigned long CountE=0;

// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ БИБЛИОТЕК
//U8GLIB_ST7920_128X64_4X u8g(PIN_SPI_SS1);          // Используем аппартный SPI но память минус 370 байт (увеличенный буфер на 4)скорость увеличивается где то на 100%
U8GLIB_ST7920_128X64_4X u8g(PIN_ST7920_SCK, PIN_ST7920_MOSI, PIN_ST7920_SS1, U8G_PIN_NONE); // Программный SPI и 4х буффер для скорости

leOS2 myOS;                                        // многозадачность
OneWire ds(PIN_ONE_WIRE_BUS);                      // поддержка температурных датчиков
EnergyMonitor emon1;                               // Мониторинг электросети

void setup(void) {
  u8g.setFont(my5x7rus);                // Русский шрифт 5х7
  // Установка входов-выходов и подтягивание внутренними резисторами
  pinMode(PIN_LED_FLOW, OUTPUT);        // Индикация потока светодиод и реле
  digitalWrite(PIN_LED_FLOW, HIGH);  
  pinMode(PIN_FLOW1, INPUT);            //  Подключение датчика потока 1
  digitalWrite(PIN_FLOW1, HIGH);
  pinMode(PIN_FLOW2, INPUT);            //  Подключение датчика потока 2
  digitalWrite(PIN_FLOW2, HIGH);
  pinMode(PIN_KEY, INPUT);              //  Включена кнопка
  digitalWrite(PIN_KEY, HIGH);
  pinMode(PIN_ENERGY_COUNT, INPUT);     //  Электросчетчик
  digitalWrite(PIN_ENERGY_COUNT, HIGH); 
 
 #ifdef  RADIO    // Радио модуль NRF42l  определение управляющих ног
    pinMode( PIN_SPI_CSN, OUTPUT);          
    digitalWrite( PIN_SPI_CSN, HIGH);
    pinMode( PIN_SPI_CE, OUTPUT);           
    digitalWrite(PIN_SPI_CE, HIGH);
 #endif 

  // Установка начальных переменных
  interval=TimeMeasurement;
  ChartInterval=TimeOneMinutes;  
  attachInterrupt(sensorInterrupt1, pulseCounter1, CHANGE);  // Прерывания на ОБА ФРОНТА
  attachInterrupt(sensorInterrupt2, pulseCounter2, CHANGE);
  
  myOS.begin();
  myOS.addTask(measurement,myOS.convertMs(TimeMeasurement));
  myOS.addTask(OneMinutes,myOS.convertMs(TimeOneMinutes));
     
// Программирование 1 таймера на подсчет событий на PIN_ENERGY_COUNT (туда заводится электросчетчик)
  TCCR1A = 0;   // сброс Timer 1 
  TCCR1B = 0; 
  TCNT1  = 0;  
  // start Timer 1 External clock source on T1 pin (D5). Clock on rising edge.
  TCCR1B =  bit (CS10) | bit (CS11) | bit (CS12);
  
  emon1.voltage(PIN_ANALOG_V, CALIBRATION_CONST_220,1.7);  // Инициализация датчика напряжения 220
  emon1.current(PIN_ANALOG_I, CALIBRATION_CONST_220_I);    // Инициализация датчика тока 220
  
  // проверка на запуск экрана сброса
 if ((digitalRead(PIN_KEY)==0)&&(digitalRead(PIN_KEY)==0)) fStart=-1; // Установка признака стартового экрана
  // Чтение счетчиков из флеша
  //  AllResetEeprom(); // обнуление всех счетчиков только один раз в момент пуска системы
  readEeprom();
 
  #ifdef  RADIO    // Радио модуль NRF24
      //---ИНИЦИАЛИЗАЦИЯ РАДИО МОДУЛЯ ------------------------------------- 
      radio.begin();
    // выбор скорости
      radio.setDataRate(RF24_250KBPS);
    //  radio.setDataRate(RF24_1MBPS);
    //  radio.setDataRate(RF24_2MBPS);
    radio.setPALevel(RF24_PA_MAX); // Максимальная мощность
    radio.setChannel(100);         //тут установка канала
    radio.setCRCLength(RF24_CRC_16);
    // radio.setAutoAck(false); // выключить аппаратное потверждение
    radio.setRetries(15,15);    // Задержка и число повторов
    radio.openWritingPipe(pipes[1]);   // Открываем канал передачи
    radio.openReadingPipe(1,pipes[0]); // Открываем канал приема  для получения подтверждения доставки пакета
    myOS.addTask(send_packet,myOS.convertMs(Time_RF24_send));  // Создание задачи по отправке пакетов
 #endif 
   }

// --------------------------------------------- LOOP ------------------------------------	 
void loop(void) {
  float t;
    // Старт преобразования датчиков время занимает до 0.7 секунды!!!!
    StartDS1820(ThermOut1);
    StartDS1820(ThermIn1);
    StartDS1820(ThermOut2);
    StartDS1820(ThermIn2);
    indicator=!indicator;   
 
    u8g.firstPage(); 
     do {
       // Общая часть для каждого экрана (для экономии длины кода) - заголовок экрана
       if (fStart>0)
        { 
          u8g.drawBox(0,0,128,9);  // Закрашиваем прямоугольник белым 
          u8g.setColorIndex(0);    // Черные чернила, Белый  фон
          u8g.setPrintPos(1, 7);
          u8g.print(int(fStart));
          u8g.drawLine(118, 0, 118, 8);  // Индикатор потока на экран и на реле
          if ((flowRate1>0)&&(flowRate2>0)) {digitalWrite(PIN_LED_FLOW, HIGH); 
            if (indicator==true) u8g.drawStr(121, 7,F("/")); else u8g.drawStr(121, 7,F("\\")); }// Светодиод и реле показывает что поток есть реле идет к ТН (1-2)
          else {digitalWrite(PIN_LED_FLOW, LOW); u8g.drawStr(121, 7,F("-"));}
          u8g.drawLine(98, 0, 98, 8);  // Индикатор потока на экран и на реле
          u8g.setPrintPos(100, 7);
          u8g.print(dtostrf(emon1.Vrms,3,0,buf)); // Вывод напряжения в строке состояния
            #ifdef  RADIO    // Радио модуль NRF42l ИНДИКАТОР ПЕРЕДАЧИ последнего пакета
                if (send_packet_ok==true)  u8g.drawXBMP(92, 1, 5, 7, antenna); else u8g.drawStr(92, 7,F("_"));
            #else
                u8g.drawStr(92, 7,F("-")); // радиомодуля нет выводим прочерк
            #endif 
           u8g.setPrintPos(8, 7);   // Далее сразу можно печатать заголовок
        }
       switch (fStart)
        {
        case -1:  ScreenReset(); break;  
        case  0:  ScreenStart(); break;
        case  1:  Screen1(); break;
        case  2:  Screen2(); break;
        case  3:  Screen3(); break;
        case  4:  Screen4(); break;
        case  5:  Screen5(); break;
        default:  Screen1();
        }
    
       } while( u8g.nextPage() );
       
       if (fStart==0) // Только один раз
         { delay(3000); myOS.addTask(scanKey,myOS.convertMs(TimeInterview)); } // Клавиатура активируется, начинается сканирование
        
       if (count>=TIME_WRITE_EEPROM) {writeEeprom();count=0;}  // Запись eeprom
       
        emon1.calcVI(40,2000);  // чтение датчиков тока и напряжения
   
        if (fStart>0 && myOS.getTaskStatus(scanKey)==0) myOS.restartTask(scanKey); //Борьба с зависаниями при переключении от кнопки - включение сканирования
        
        delay(350); 
        tOut1=VerificationTemp(tOut1,errOut1,ThermOut1);
        tIn1=VerificationTemp(tIn1,errIn1,ThermIn1);
        tOut2=VerificationTemp(tOut2,errOut2,ThermOut2);
        tIn2=VerificationTemp(tIn2,errIn2,ThermIn2);
  	}
     
   
float VerificationTemp(float newT,float errT, unsigned char *adrr)
{
 float t=getTempDS1820(adrr);
 if ((t>-15&&t<70&&abs(newT-t)<5)||(newT<-20)) t=t-errT;   // Иногда считывает глюки 
 return t;
} 
     
// Экран сброса настроек сезона --------------------------------------
void ScreenReset()
 {
  u8g.drawBox(0,0,128,9);  // Закрашиваем прямоугольник белым 
  u8g.setColorIndex(0);    // Черные чернила, Белый  фон
  u8g.drawStr(1, 7 , F("Сброс счетчиков сезона"));
  u8g.setColorIndex(1);
   
  u8g.drawStr(0, 16 , F("Выработка G кВт"));
  u8g.setPrintPos(84, 16);
  u8g.print(long(CountsSRAM.YearEnergyG));
    
  u8g.drawStr(0, 24 , F("Потреб. P кВт"));
  u8g.setPrintPos(84, 24);
  u8g.print(long(CountsSRAM.YearEnergyP));
   
  u8g.drawStr(0, 32 , F("Потреб. E кВт"));
  u8g.setPrintPos(84, 32);
  u8g.print(long(CountsSRAM.YearCountE/eCONST));  // импульсы в кВт
 
  u8g.drawStr(0, 40 , F("Наработка Часы"));
  u8g.setPrintPos(84, 40);
  u8g.print((unsigned int)(CountsSRAM.YearHour/60.0/60.0));
  
  u8g.drawLine(0, 46, 127, 46); 
  u8g.drawStr(0, 54 , F("СБРОС -  нажатие кнопки"));
  u8g.drawStr(0, 62 , F("ОТМЕНА - выключение"));
 
 // Опрос кнопки Сначало надо кнопку отпустить а потом нажимать
 if ((digitalRead(PIN_KEY)==1)&&(digitalRead(PIN_KEY)==1))  ScreenResetCount=1;   // кнопку отжали экран появился
 if ((digitalRead(PIN_KEY)==0)&&(digitalRead(PIN_KEY)==0)&&(ScreenResetCount==1)) // кнопку нажали и надо сбрасывать, простейшее подавление дребезка
 { fStart=1;  YearResetEeprom(); myOS.addTask(scanKey,myOS.convertMs(TimeInterview));}
 }   
// Стартовый экран --------------------------------------
void ScreenStart()
{
 u8g.setColorIndex(1);
 u8g.drawXBMP(0, 0, 56, 56, u8g_logo_bits); 
// u8g.drawXBMP(0, 0, 51, 51, aLogo); 
 u8g.drawStr(73, 8,  F("Контроль"));
 u8g.drawStr(70, 16, F("Теплового"));
 u8g.drawStr(78, 24, F("Насоса"));
 u8g.drawStr(66, 32, F("Arduino UNO"));
 #ifdef  RADIO    // Радио модуль NRF24
    u8g.drawStr(58, 41, F("Радио NRF24L"));
 #endif
 #ifdef  DEMO    // Демка
    u8g.drawStr(68, 52, F("-- DEMO --"));
 #endif
 u8g.drawStr(1, 63,F(VERSION));
}

// Отображение типа 1 ТЕПЛЫЙ ПОЛ-------------------------------------
void Screen1()
{
  byte i;
  u8g.print(F("Теплые Полы")); 
  u8g.setColorIndex(1);    // белые чернила, черный фон
  Screen12();
  u8g.setPrintPos(32, 16);
  u8g.print(dtostrf(flowRate1*60.0/1000.0,4, 3, buf));
  u8g.setPrintPos(95, 16);
  u8g.print(HZ1); 
  u8g.setPrintPos(8+14, 26); // Подача
  u8g.print(dtostrf(tOut1,4, 2, buf)); 
  u8g.setPrintPos(8+14, 26+9); // Обратка
  u8g.print(dtostrf(tIn1,4, 2, buf)); 
 
//  if (tOut1-tIn1<0) u8g.setPrintPos(8+13, 26+18); // Разность температур
//  else  u8g.setPrintPos(8+14+6, 26+9+9); 
   u8g.setPrintPos(8+14, 26+9+9);
  u8g.print(dtostrf(abs(tOut1-tIn1),4, 2, buf));  // Всегда больше нуля

  u8g.drawBox(64,19,64,9);  // Закрашиваем прямоугольник белым 
  u8g.setColorIndex(0);     // Черные чернила, Белый  фон
  u8g.drawStr(111, 26, F("кВт"));
  u8g.setPrintPos(86, 26);
  u8g.print(dtostrf(P1,4, 2, buf));
  if (tOut1>tIn1) u8g.drawStr(64, 26, F("Наг."));  // Показ режима работы теплового насоса
  else            u8g.drawStr(64, 26, F("Охл."));
  
  u8g.setColorIndex(1);    
  u8g.setPrintPos(12, 54);
  u8g.print(dtostrf(totalLitres1/60.0/1000.0,11, 2, buf)); 
  u8g.setPrintPos(12, 63);
  u8g.print(dtostrf(totalPower1,11, 2, buf)); 
  
  // График
  // for(i=0;i<60;i++) u8g.drawPixel(65+i,63-Chart1[i]);     // Точки
  for(i=0;i<60;i++) u8g.drawLine(65+i,64,65+i,64-Chart1[i]); // Линия 0 не выводится
}
// Общие процедуры для первого и второго экранов - экономим rom --------------------------------
void Screen12()
{
  u8g.drawStr(0, 16, F("Поток:         Част:"));
  u8g.drawXBMP(58, 10, 9, 7, cub);
  u8g.drawXBMP(120, 10, 8, 7, Hz);
  u8g.drawLine(0, 18, 127, 18); 
  u8g.drawXBMP(0, 20, 21, 25, T123); // Термометр и значки
  u8g.drawXBMP(8+45, 26-6, 9, 7, C0);
  u8g.drawXBMP(8+45, 26-6+9, 9, 7, C0);
  u8g.drawXBMP(8+45, 26-6+9+9, 9, 7, C0);
  u8g.drawLine(63, 18, 63, 63); 
  u8g.drawLine(0, 46, 63, 46);
  u8g.drawStr(0, 54, F("V:"));
  u8g.drawStr(0, 63, F("G:"));
} 

// Отображение типа 2 ГЕО КОНТУР-------------------------------------
void Screen2()
{
  byte i;
  u8g.print(F("Гео контур")); 
  u8g.setColorIndex(1);    // белые чернила, черный фон
  Screen12();
  u8g.setPrintPos(32, 16);
  u8g.print(dtostrf(flowRate2*60.0/1000.0,4, 3, buf));
  u8g.setPrintPos(95, 16);
  u8g.print(HZ2); 
  u8g.setPrintPos(8+14, 26); // Подача
  u8g.print(dtostrf(tOut2,4, 2, buf)); 
  
  u8g.setPrintPos(8+14, 26+9); // Обратка
  u8g.print(dtostrf(tIn2,4, 2, buf)); 
 
 // if (tOut2-tIn2<0) u8g.setPrintPos(8+13, 26+9+9); // Разность температур
//  else  u8g.setPrintPos(8+14+6, 26+9+9); 
  u8g.setPrintPos(8+14, 26+9+9);
  u8g.print(dtostrf(abs(tOut2-tIn2),4, 2, buf)); 
  
  u8g.drawBox(64,19,64,9);  // Закрашиваем прямоугольник белым 
  u8g.setColorIndex(0);     // Черные чернила, Белый  фон
  u8g.drawStr(111, 26, F("кВт"));
  u8g.setPrintPos(86, 26);
  u8g.print(dtostrf(P2,4, 2, buf));
  if (tOut2>tIn2) u8g.drawStr(64, 26, F("Наг."));  // Показ режима работы теплового насоса
  else            u8g.drawStr(64, 26, F("Охл."));
  
  
//  u8g.drawLine(63, 18, 63, 63); 
 
//  u8g.drawBox(64,19,63,9);  // Закрашиваем прямоугольник белым 
//  u8g.setColorIndex(0);     // Черные чернила, Белый  фон
//  u8g.drawStr(107, 26, F("кВт"));
//  u8g.setPrintPos(80, 26);
//  u8g.print(dtostrf(P2,4, 2, buf));
  u8g.setColorIndex(1);    
  
  u8g.setPrintPos(12, 54);
  u8g.print(dtostrf(totalLitres2/60.0/1000.0,11, 2, buf)); 
  u8g.setPrintPos(12, 63);
  u8g.print(dtostrf(totalPower2,11, 2, buf));
   // График
  // for(i=0;i<60;i++) u8g.drawPixel(65+i,63-Chart2[i]); 
  for(i=0;i<60;i++) u8g.drawLine(65+i,64,65+i,64-Chart2[i]); // 0 не выводится
   
}

// Отображение типа 3  Напряжение Ток-------------------------------------
void Screen3()
{
  int i;
  u8g.print(F("Напряжение&Ток"));
  u8g.setColorIndex(1);    // белые чернила, черный фон
  u8g.drawStr(0, 16, F("V rms:"));
  
  u8g.setPrintPos(31, 16);
  u8g.print(dtostrf(emon1.Vrms,4,1,buf));
  u8g.drawStr(0, 24, F("I rms:"));
  u8g.setPrintPos(31, 24);
  u8g.print(dtostrf(emon1.Irms,4,1,buf));
  u8g.drawStr(65, 16, F("Pow_f:"));
  u8g.setPrintPos(102, 16);
  u8g.print(dtostrf(emon1.powerFactor,3,2,buf));
  u8g.drawStr(0, 34, F("PW кВт:"));
  u8g.setPrintPos(39,34);
  u8g.print(dtostrf(abs(pPower),4, 2, buf));
  u8g.drawStr(75, 26, F("COP:"));
  u8g.setPrintPos(98, 26);
  if (pPower>0.500) u8g.print(dtostrf(P1/pPower,3,2,buf)); else u8g.print(dtostrf(0.0,3,2,buf));
  u8g.drawLine(63, 18, 63, 63); 
  u8g.drawLine(0, 26, 63, 26);
  u8g.drawLine(64, 18, 127, 18);
  
  //  for(i=0;i<=6;i++) u8g.drawPixel(62,64-i*4);  // Шкала мощность
  for(i=0;i<=5;i++)
  { u8g.drawPixel(64,64-i*7);   // Шкала СОР
    u8g.drawPixel(65,64-i*7);
  }
 
 for(i=0;i<60;i++) // График
    { 
     u8g.drawLine(i,64,i,64-pChart[i]);   
     u8g.drawLine(67+i,64,67+i,64-ChartCOP[i]); 
    }
}

// Отображение типа 4  Счетчики -------------------------------------
void Screen4()
{
  float temp;
  u8g.print(F("Счетчики")); 
  u8g.setColorIndex(1);    // белые чернила, черный фон
  
  u8g.drawLine(28, 9, 28, 63);
  u8g.drawLine(79, 9, 79, 63);
  u8g.drawLine(0, 43, 127, 43);
  u8g.drawStr(0, 16, F("        За сезон   Общий"));
  
  u8g.drawStr(0, 24, F("G кВт"));
  u8g.setPrintPos(31, 24);
  u8g.print(long(CountsSRAM.YearEnergyG));
  u8g.setPrintPos(82, 24);
  u8g.print(long(CountsSRAM.GlobalEnergyG));
  
  u8g.drawStr(0, 32, F("P кВт"));
  u8g.setPrintPos(31, 32);
  u8g.print(long(CountsSRAM.YearEnergyP));
  u8g.setPrintPos(82, 32);
  u8g.print(long(CountsSRAM.GlobalEnergyP));
  
  u8g.drawStr(0, 40, F("E кВт"));
  u8g.setPrintPos(31, 40);
  u8g.print(long(CountsSRAM.YearCountE/eCONST));  // импульсы в кВт
  u8g.setPrintPos(82, 40);
  u8g.print(long(CountsSRAM.GlobalCountE/eCONST)); // импульсы в кВт
  
  u8g.drawStr(0, 51, F("COP"));
  u8g.setPrintPos(30, 51);
  u8g.print(dtostrf((float)CountsSRAM.YearEnergyG/(float)CountsSRAM.YearEnergyP,4,2, buf));
  u8g.setPrintPos(81, 51);
  u8g.print(dtostrf((float)CountsSRAM.GlobalEnergyG/(float)CountsSRAM.GlobalEnergyP,4,2, buf));
  
  u8g.drawStr(30+22, 51, F(":"));
  u8g.drawStr(81+22, 51, F(":"));

  u8g.setPrintPos(30+26, 51);
  temp=(float)CountsSRAM.YearEnergyG/((float)CountsSRAM.YearCountE/(float)eCONST);
  if (temp<10.0) u8g.print(dtostrf(temp,4,2, buf));
      else u8g.print(F("-.-"));
  u8g.setPrintPos(81+26, 51);
  temp=(float)CountsSRAM.GlobalEnergyG/((float)CountsSRAM.GlobalCountE/(float)eCONST);
  if (temp<10.0) u8g.print(dtostrf(temp,4,2, buf));
      else u8g.print(F("-.-"));
  
  u8g.drawStr(0, 60, F("Часы"));
  u8g.setPrintPos(31, 60);
  u8g.print((unsigned int)(CountsSRAM.YearHour/60/60)); // Из секунд в часы
  u8g.setPrintPos(82, 60);
  u8g.print((unsigned int)(CountsSRAM.GlobalHour/60/60));
 }
 
// Отображение типа 5 Параметры  -------------------------------------
void Screen5()
{
  u8g.print(F("Параметры")); 
  u8g.setColorIndex(1);    // белые чернила, черный фон

  u8g.drawStr(0, 16, F("Питание UNO В"));
  u8g.setPrintPos(97, 16);
  u8g.print(dtostrf(emon1.readVcc()/1000.0,4, 3, buf));
    
  u8g.drawStr(0, 24, F("Температура чипа"));
  u8g.setPrintPos(97, 24);
  u8g.print(dtostrf(GetTemp(),4,2, buf));
  
  u8g.drawStr(0, 32, F("RAM free байт"));
  u8g.setPrintPos(97, 32);
  u8g.print(freeRam ()); 
  
  u8g.drawStr(0, 40, F("Таймера мс         :"));
  u8g.setPrintPos(65, 40);
  u8g.print(dtostrf(interval,5,0,buf));  
  u8g.setPrintPos(98, 40);
  u8g.print(dtostrf(ChartInterval,5,0,buf));  

  u8g.drawStr(0, 48, F("ТП err out/in"));
  u8g.setPrintPos(80, 48);
  u8g.print(dtostrf(errOut1,3, 2, buf));
  u8g.setPrintPos(108, 48);
  u8g.print(dtostrf(errIn1,3, 2, buf));
    
  u8g.drawStr(0, 56, F("Гео err out/in"));
  u8g.setPrintPos(80, 56);
  u8g.print(dtostrf(errOut2,3, 2, buf));
  u8g.setPrintPos(108, 56);
  u8g.print(dtostrf(errIn2,3, 2, buf));
  
  u8g.drawLine(103, 42, 103, 56); 
} 

// Прерывание подсчет импульсов от первого канала
void pulseCounter1() {   pulseCount1++; }
// Прерывание подсчет импульсов от второго канала
void pulseCounter2() {   pulseCount2++; }

// ОДНА СЕКУНДА Цикл измерения интервал задается -----------------------------------
void measurement()
{
 float intervalHour;           // интервал измерения в часах, используется для увеличения скорости
 cli();  // Запретить прерывания TimeMeasurement
 count1=pulseCount1;  pulseCount1 = 0;
 count2=pulseCount2;  pulseCount2 = 0;
 sei();  // разрешить прерывания
 
 interval=millis()-oldTime; // Точное определение интервала ----------------------------
 oldTime= interval+oldTime;  
 if (interval<=0) interval=TimeMeasurement; // Через 50 дней millis() сбросится по этому после этого первая итерация расчетная

 #ifdef  DEMO    // Демка
     count1=random(190,240); 
     count2=random(160,200);
     TCNT1= random(5,9);
     emon1.realPower=random(1900,2800);
 #endif

 intervalHour=((float)interval/1000.0)/3600.0;  // Предварительный расчет для оптимизации времени
 
 // Первый канал 
 HZ1=(float)(count1*1000.0)/interval/2; // делить на 2 - это т.к. прерывания работает на оба фронта импульса CHANGE
 flowRate1 = HZ1/calibrationFactor; // рассчитать поток в литрах за минуту
 totalLitres1 = totalLitres1 + flowRate1*(interval/1000.0); 
 P1=(flowRate1/60) // литры/килограммы в секунду
                   *heat_capacity*(abs(tOut1-tIn1)); 
 totalPower1 = totalPower1 + P1*intervalHour; 
 
 // Второй канал 
 HZ2=(float)(count2*1000.0)/interval/2; // делить на 2 - это т.к. прерывания работает на оба фронта импульса CHANGE
 flowRate2 = HZ2/calibrationFactor; // рассчитать поток в литрах за минуту
 totalLitres2 = totalLitres2 + flowRate2*(interval/1000.0); 
 P2=(flowRate2/60) // литры/килограммы в секунду
                   *heat_capacity*(abs(tOut2-tIn2)); 
 totalPower2 = totalPower2 + P2*intervalHour; 
 
 // Электросчетчик
 eCount=eCount+TCNT1;                                        // Общий счетчик
 ePower=(float)TCNT1*3600.0/(float)eCONST/(interval/1000.0); // Пиковый счетчик
 eEnergy=(float)eCount/(float)eCONST ;
 TCNT1 = 0;
 
 // Датчик напряжения и тока
 pPower=emon1.realPower/1000.0;         // Киловатты
 pEnergy=pEnergy+pPower*intervalHour;   // Потребленная энергия
 }

// Сканирование клавиш ------------------------------------------
void scanKey()
{
    byte key;  
    // cli();                                                         // Запретить прерывания TimeMeasurement
    key=digitalRead(PIN_KEY);                                         // Прочитать кнопку
    // sei();                                                         // разрешить прерывания
    if ((key==1)&&(oldKey==1)&&(flagKey==1)) {fStart++;flagKey=0;}    // Клавиша нажита
    if ((key==0)&&(oldKey==1))
        { flagKey=1;                           // Начало (по заднему фронту) ожидания нового нажатия 
         myOS.pauseTask(scanKey);              // Сканирование кнопки поставить на паузу, а то иногда висла задача
        }
    oldKey=key;
    if (fStart > NUM_SCREEN) fStart=1;
}

// РАЗ В МИНУТУ Подготовка массива точек для графиков и счетчиков ----------------------------------------
void OneMinutes()
{
 int i;
 float dP,dG,dGeo;
 unsigned long dE;
 
 count++;                                            // счетчик минут для записи eeprom
 
 ChartInterval=millis()-ChartOldTime;                // Точное определение интервала показа графика----------------------------
 ChartOldTime= ChartInterval+ChartOldTime;  
 if (ChartInterval<=0) ChartInterval=TimeMeasurement; // Через 50 дней millis() сбросится по этому после этого первая итерация расчетная
  // Сдвиг графиков и запись новых данных
  for(i=0;i<60;i++) 
     {  
      Chart1[i]=Chart1[i+1]; 
      Chart2[i]=Chart2[i+1];
      pChart[i]=pChart[i+1];
      ChartCOP[i]=ChartCOP[i+1];
     }
    Chart1[59]=P1*2.5;
    Chart2[59]=P2*2.5;
    pChart[59]=9*pPower; 
    if (pPower>0.500) ChartCOP[59]=(P1/(pPower))*7.0; else ChartCOP[59]=0;
     
 // Подготовка  накопительныч счетчиков для записи в eeprom ----------------------------------------------------
 // работают только если есть поток т.е. ТН работает!!!!! простой не учитывается СПОРНО надо думать
 
 //  Электричество
 dP=pEnergy-EnergyP;                  // Прирост 
 EnergyP=pEnergy;                     // запоминание для нового цикла
 CountsSRAM.GlobalEnergyP =CountsSRAM.GlobalEnergyP+dP;
 CountsSRAM.YearEnergyP   =CountsSRAM.YearEnergyP+dP;

 // Тепло ТП
 dG= totalPower1-EnergyG; // Прирост за час
 EnergyG=totalPower1;      // запоминание для нового цикла
 CountsSRAM.GlobalEnergyG =CountsSRAM.GlobalEnergyG+dG;
 CountsSRAM.YearEnergyG   =CountsSRAM.YearEnergyG+dG;
 
 // Тепло ГЕО контур
 dGeo= totalPower2-EnergyGeo; // Прирост за час
 EnergyGeo=totalPower2;      // запоминание для нового цикла
 CountsSRAM.GlobalEnergyGeo =CountsSRAM.GlobalEnergyGeo+dGeo;
 CountsSRAM.YearEnergyGeo   =CountsSRAM.YearEnergyGeo+dGeo;
 
 // Мото часы В секундах!!!!!!!!!!!!
 CountsSRAM.GlobalHour=CountsSRAM.GlobalHour+ChartInterval/1000; // в секундах
 CountsSRAM.YearHour=CountsSRAM.YearHour+ChartInterval/1000;     // в секундах
 
 // Электросчетчик
 dE=eCount-CountE;                   // Прирост электросчетчика
 CountE=eCount;                      // запоминание для нового цикла
 CountsSRAM.GlobalCountE =CountsSRAM.GlobalCountE+dE;
 CountsSRAM.YearCountE   =CountsSRAM.YearCountE+dE;
}

 // ---ПЕРЕДАЧА ДАННЫХ ЧЕРЕЗ РАДИОМОДУЛЬ -----------------------------
void send_packet()
{
#ifdef  RADIO    // Радио модуль NRF42l  
  packet.tOut1=100*tOut1;
  packet.tIn1=100*tIn1;
  packet.tOut2=100*tOut2;
  packet.tIn2=100*tIn2;
  packet.PowerFloor=P1*1000;
  packet.PowerLand=P2*1000;
  packet.I=emon1.Irms*100;
  packet.U=emon1.Vrms*100;
  packet.P=pPower*1000;
    myOS.pauseTask(send_packet);   // Остановить эту задачу а то при отсутствии связи идет сброс arduino
        radio.stopListening();     // Остановить приемник
    //    radio.flush_tx();          // Очистить буфер
        send_packet_ok = radio.write(&packet,sizeof(packet));
        radio.startListening();    // Включить приемник
    myOS.restartTask(send_packet); // Запустить задачу
  #endif  
 }  


// Старт преобразования одного датчика -----------------------------------------------
void StartDS1820(byte *addr)
{
   ds.reset();
   delay(10);
   ds.select(addr);
   delay(10);
   ds.write(0x44,0);
   delay(10);  
}
// Чтение температуры одного датчика -----------------------------------------------
float getTempDS1820(unsigned char *addr)
{
    byte i;
    byte type_s;
    byte present = 0;
    byte data[12];
    // Запрос уже послан ранее StartDS1820
    present = ds.reset();
    ds.select(addr);
    ds.write(0xBE); // Команда на чтение регистра температуры
    for ( i = 0; i < 9; i++) { data[i] = ds.read();}
    int16_t raw = (data[1] << 8) | data[0];
    if (type_s) {
     raw = raw << 3; // 9 bit resolution default
     if (data[7] == 0x10) {
      // "count remain" gives full 12 bit resolution
      raw = (raw & 0xFFF0) + 12 - data[6];
    }
  } else {
    byte cfg = (data[4] & 0x60);
    // at lower res, the low bits are undefined, so let's zero them
    if (cfg == 0x00) raw = raw & ~7;      // 9 bit resolution, 93.75 ms
    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
    //// default is 12 bit resolution, 750 ms conversion time
  }
  return (float)raw / 16.0;       
}

// Чтение внутреннего датчика температуры ---------------------------------------
double GetTemp(void)
{
  unsigned int wADC;
  double t;
  ADMUX = (_BV(REFS1) | _BV(REFS0) | _BV(MUX3));
  ADCSRA |= _BV(ADEN);  
  delay(20);           
  ADCSRA |= _BV(ADSC);  
  while (bit_is_set(ADCSRA,ADSC));
  wADC = ADCW;
  t = (wADC - 324.31 ) / 1.22;
  return (t); 
}
// Чтение свободной памяти --------------------------------------------------------------------
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
//----------------------------EEPROM FUNCTION--------------------------------------------------
void writeEeprom()
{ 
cli(); 
  eeprom_write_block((const void*)&CountsSRAM, (void*) &CountsEEPROM, sizeof(CountsSRAM)); 
sei();
}
// Полный сброс делается только один раз
void AllResetEeprom()
{
 CountsSRAM.GlobalEnergyP  =0;                       
 CountsSRAM.GlobalEnergyG  =0;                        
 CountsSRAM.YearEnergyP    =0;                          
 CountsSRAM.YearEnergyG    =0;                         
 CountsSRAM.GlobalHour     =0;                          
 CountsSRAM.YearHour       =0;  
 CountsSRAM.GlobalCountE   =0;                       
 CountsSRAM.YearCountE     =0;   
 CountsSRAM.GlobalEnergyGeo=0;
 CountsSRAM.YearEnergyGeo  =0;
 writeEeprom(); 
}
// Сброс только последнего сезона
void YearResetEeprom()
{
 CountsSRAM.YearEnergyP    =0;                          
 CountsSRAM.YearEnergyG    =0;                         
 CountsSRAM.YearHour       =0;  
 CountsSRAM.YearCountE     =0;   
 CountsSRAM.YearEnergyGeo  =0;
 writeEeprom(); 
}

//Чтение счетчиков из Eeprom 
void readEeprom()
{
cli(); 
   eeprom_read_block((void*)&CountsSRAM, (const void*) &CountsEEPROM, sizeof(CountsSRAM)); 
sei();
}


