// --------------------------------------------------------------------------------------
// Вся теория и практика (и библиотека) изложена здесь:  http://openenergymonitor.org
// Переработанная библиотека emonLib. Выкинул все лишнее и добавил свое что было нужно. Затачивал под UNO
// Про измерение напряжения: http://openenergymonitor.org/emon/buildingblocks/measuring-voltage-with-an-acac-power-adapter
// Только я использовал переменные резисторы для настройки уровня и смещения сигнала (10 ком).
// Про измерение тока читать здесь:  http://openenergymonitor.org/emon/buildingblocks/ct-sensors-interface
// Использовал датчик тока AC-1020 нагрузка 60 ом + резистор регулирования смещения 10 кОм
// Для увеличения точности измерения калибровал внутренний источник опорного напряжения
// --------------------------------------------------------------------------------------
#include "emonLib.h"
//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _inPinV, double _VCAL, double _PHASECAL)
{
   inPinV = _inPinV;
   VCAL = _VCAL;
   PHASECAL = _PHASECAL;
   offsetV = ADC_COUNTS>>1;
}

void EnergyMonitor::current(unsigned int _inPinI, double _ICAL)
{
   inPinI = _inPinI;
   ICAL = _ICAL;
   offsetI = ADC_COUNTS>>1;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int crossings, unsigned int timeout)
{
  int SupplyVoltage = readVcc();
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented  

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  boolean st=false;                                  //an indicator to exit the while loop
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.
  while(st==false)                                   //the while loop...
  {
     startV = analogRead(inPinV);                    //using the voltage waveform
     if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) st=true;  //check its within range
     if ((millis()-start)>timeout) st = true;
  }
  
  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //------------------------------------------------------------------------------------------------------------------------- 
  start = millis(); 

  while ((crossCount < crossings) && ((millis()-start)<timeout)) 
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation
    
    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = analogRead(inPinV);                 //Read in raw voltage signal
    sampleI = analogRead(inPinI);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);
	filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI-offsetI)/1024);
	filteredI = sampleI - offsetI;
   
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------  
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum
    
    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------   
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum 
    
    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV); 
    
    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------   
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP +=instP;                               //Sum  
    
    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength 
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------       
    lastVCross = checkVCross;                     
    if (sampleV > startV) checkVCross = true; 
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;                  
                     
    if (lastVCross != checkVCross) crossCount++;
  }
 
  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //------------------------------------------------------------------------------------------------------------------------- 
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied. 
  
  double V_RATIO = VCAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples); 
  
  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples); 

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------       
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(unsigned int Number_of_Samples)
{
  
   #if defined emonTxV3
	int SupplyVoltage=3300;
   #else 
	int SupplyVoltage = readVcc();
   #endif

  
  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = analogRead(inPinI);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset, 
	//  then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI-offsetI)/1024);
	filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;
    // 2) sum 
    sumI += sqI;
  }

  double I_RATIO = ICAL *((SupplyVoltage/1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / Number_of_Samples); 
  //Reset accumulators
  sumI = 0;
//--------------------------------------------------------------------------------------       
  return Irms;
}

long EnergyMonitor::readVcc() {
  long result;
   // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(1); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = READVCC_CALIBRATION_CONST  / result; // Back-calculate AVcc in mV
  return result;  
}

