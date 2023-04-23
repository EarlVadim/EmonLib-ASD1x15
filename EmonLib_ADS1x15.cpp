/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

// Proboscide99 10/08/2016 - Added ADMUX settings for ATmega1284 e 1284P (644 / 644P also, but not tested) in readVcc function

//#include "WProgram.h" un-comment for use on older versions of Arduino IDE
#include "EmonLib_ADS1x15.h"
#include <Adafruit_ADS1X15.h>


//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor::voltage(unsigned int _ads, bool _init, adsGain_t gain, uint16_t datarate, double _VCAL, double _PHASECAL)
{
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = ADC_COUNTS>>1;
  if (_init) {
   ads.setGain(gain);
   ads.setDataRate(datarate);
   ads.begin(_ads);
  }

}

void EnergyMonitor::current(unsigned int _ads, bool _init, adsGain_t gain, uint16_t datarate, double _ICAL)
{
  ICAL = _ICAL;
  offsetI = ADC_COUNTS>>1;
 // ads.setGain(GAIN_ONE);
  ads.setGain(gain);
  ads.setDataRate(datarate);
  ads.begin(_ads);
}


//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor::calcVI(unsigned int pinI, unsigned int pinV, unsigned int crossings, unsigned int timeout)
{
  
  int SupplyVoltage=3300;
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented
  filteredV = 0;

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
  //  startV = analogRead(inPinV); 
  
    switch (pinV) {
       case '0'...'3':
         startV = ads.readADC_SingleEnded(pinV);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         startV = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         startV = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         startV = ads.readADC_SingleEnded(1);
    }  

    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range  563 > X > 460
    if ((millis()-start)>timeout) break;
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
   // sampleV = analogRead(inPinV);                 //Read in raw voltage signal
  //  sampleI = analogRead(inPinI);                 //Read in raw current signal
  
    switch (pinI) {
       case '0'...'3':
         sampleI = ads.readADC_SingleEnded(pinI);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         sampleI = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         sampleI = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         sampleI = ads.readADC_SingleEnded(0);
    } 
	switch (pinV) {
       case '0'...'3':
         sampleV = ads.readADC_SingleEnded(pinI);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         sampleV = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         sampleV = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         sampleV = ads.readADC_SingleEnded(1);
    }
	
		
    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);   // x = 0 + (600-0)/1024 = 0 + 0.5859 = 0.5859 ->   x = 0.5859 + (700-0.5859)/1024 = 0.5859 + 0.6830 = 1.2689  x=1.2689 + (800-1.2689)/1024 = 1.2689+0.78 = 2.0489
	                                                // x = 0 + (600-0)/1024 = 0 + 0.5859 = 0.5859 ->   x = 0.5859 + (500-0.5859)/1024 = 0.5859 + 0.4877 = 1.0736
    filteredV = sampleV - offsetV;                  // 600-0.5859 = 599.4141                           700 - 1.2689 = 698.73
	                                                // 600-0.5859 = 599.4141                           500 - 1.0736 = 498.92
													
	
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
    phaseShiftedV = lastFilteredV + PHASECAL * sqrt(sq(filteredV - lastFilteredV));

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
//	InternalVoltage = filteredI;
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

double EnergyMonitor::calcVrms(unsigned int pinV, unsigned int crossings, unsigned int timeout)
{
  
  int SupplyVoltage=3300;
  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented
  filteredV = 0;

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
  //  startV = analogRead(inPinV); 
  
    switch (pinV) {
       case '0'...'3':
         startV = ads.readADC_SingleEnded(pinV);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         startV = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         startV = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         startV = ads.readADC_SingleEnded(1);
    }  

    if ((startV < (ADC_COUNTS*0.55)) && (startV > (ADC_COUNTS*0.45))) break;  //check its within range  563 > X > 460
    if ((millis()-start)>timeout) break;
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
   // sampleV = analogRead(inPinV);                 //Read in raw voltage signal
  //  sampleI = analogRead(inPinI);                 //Read in raw current signal  
   
	switch (pinV) {
       case '0'...'3':
         sampleV = ads.readADC_SingleEnded(pinV);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         sampleV = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         sampleV = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         sampleV = ads.readADC_SingleEnded(0);
    }
	
		
    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV-offsetV)/1024);   // x = 0 + (600-0)/1024 = 0 + 0.5859 = 0.5859 ->   x = 0.5859 + (700-0.5859)/1024 = 0.5859 + 0.6830 = 1.2689  x=1.2689 + (800-1.2689)/1024 = 1.2689+0.78 = 2.0489
	                                                // x = 0 + (600-0)/1024 = 0 + 0.5859 = 0.5859 ->   x = 0.5859 + (500-0.5859)/1024 = 0.5859 + 0.4877 = 1.0736
    filteredV = sampleV - offsetV;                  // 600-0.5859 = 599.4141                           700 - 1.2689 = 698.73
	                                                // 600-0.5859 = 599.4141                           500 - 1.0736 = 498.92
													
	
    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum	
  

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * sqrt(sq(filteredV - lastFilteredV));

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
  
  return Vrms;

//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
double EnergyMonitor::calcIrms(unsigned int pinI, unsigned int Number_of_Samples)   
{

  int SupplyVoltage=3300;
  
  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
	
    switch (pinI) {
       case '0'...'3':
         sampleI = ads.readADC_SingleEnded(pinI);  	//using the voltage waveform   readADC_SingleEnded
         break;
       case 10:
         sampleI = ads.readADC_Differential_0_1();  	//using the voltage waveform
         break;
       case 32:
         sampleI = ads.readADC_Differential_2_3();  	//using the voltage waveform
         break;
       default:
         sampleI = ads.readADC_SingleEnded(0);
    }	

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

void EnergyMonitor::serialprint()
{
  Serial.print(realPower);
  Serial.print(' ');
  Serial.print(apparentPower);
  Serial.print(' ');
  Serial.print(Vrms);
  Serial.print(' ');
  Serial.print(Irms);
  Serial.print(' ');
  Serial.print(powerFactor);
  Serial.println(' ');
  delay(100);
}

//thanks to http://hacking.majenko.co.uk/making-accurate-adc-readings-on-arduino
//and Jérôme who alerted us to http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/

