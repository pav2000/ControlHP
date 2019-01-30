// --------------------------------------------------------------------------------------
// Вся теория и практика (и библиотека) изложена здесь:  http://openenergymonitor.org
// Переработанная библиотека emonLib. Выкинул все лишнее и добавил свое что было нужно. Затачивал под UNO
// Про измерение напряжения: http://openenergymonitor.org/emon/buildingblocks/measuring-voltage-with-an-acac-power-adapter
// Только я использовал переменные резисторы для настройки уровня и смещения сигнала (10 ком).
// Про измерение тока читать здесь:  http://openenergymonitor.org/emon/buildingblocks/ct-sensors-interface
// Использовал датчик тока AC-1020 нагрузка 60 ом + резистор регулирования смещения 10 кОм
// Для увеличения точности измерения калибровал внутренний источник опорного напряжения
// --------------------------------------------------------------------------------------
// define theoretical vref calibration constant for use in readvcc()
// 1100mV*1024 ADC steps http://openenergymonitor.org/emon/node/1186
// override in your code with value for your specific AVR chip
// determined by procedure described under "Calibrating the internal reference voltage" at
// http://openenergymonitor.org/emon/buildingblocks/calibration

// to enable 12-bit ADC resolution on Arduino Due, 
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.
#ifndef EmonLib_h
#define EmonLib_h 
#define ADC_BITS    10     // разрядность АЦП
#define ADC_COUNTS  (1<<ADC_BITS)
class EnergyMonitor
{
  public:

    void voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL);
    void current(unsigned int _inPinI, double _ICAL);
    void calcVI(unsigned int crossings, unsigned int timeout);
    double calcIrms(unsigned int NUMBER_OF_SAMPLES);
    long readVcc();
    //Useful value variables
    double realPower,
       apparentPower,
       powerFactor,
       Vrms,
       Irms;

  private:
    //Set Voltage and current input pins
    unsigned int inPinV;
    unsigned int inPinI;
    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    double VCAL;
    double ICAL;
    double PHASECAL;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
	int sampleV;  							 //sample_ holds the raw analog read value
	int sampleI;                     

	double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
	double filteredI;                  
	double offsetV;                          //Low-pass filter output
	double offsetI;                          //Low-pass filter output               

	double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.
	double sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous
	int startV;                                       //Instantaneous voltage at start of sample window.
	boolean lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
};
#endif

