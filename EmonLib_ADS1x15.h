/*
  Emon.h - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/


#define EmonLib_ADS1x15_h
#include "Arduino.h"
#include <Adafruit_ADS1X15.h>


// to enable 12-bit ADC resolution on Arduino Due,
// include the following line in main sketch inside setup() function:
//  analogReadResolution(ADC_BITS);
// otherwise will default to 10 bits, as in regular Arduino-based boards.

#define ADC_BITS    16

#define ADC_COUNTS  (1<<ADC_BITS)

// #define ADS (0x49)


class EnergyMonitor
{
  public:

    void voltage(unsigned int _ads, bool _init, adsGain_t gain, uint16_t _datarate, double _VCAL, double _PHASECAL);
    void current(unsigned int _ads, bool _init, adsGain_t gain, uint16_t _datarate, double _ICAL);

    void calcVI(unsigned int pinI, unsigned int pinV, unsigned int crossings, unsigned int timeout);
	double calcVrms(unsigned int pinV, unsigned int crossings, unsigned int timeout);
    double calcIrms(unsigned int pinI, unsigned int NUMBER_OF_SAMPLES);
    void serialprint();

    //Useful value variables
    double realPower,
      apparentPower,
      powerFactor,
      Vrms,
      Irms,
	  InternalVoltage;

  private:


    //Calibration coefficients
    //These need to be set in order to obtain accurate results
    double VCAL;
    double ICAL;
    double PHASECAL;

    //--------------------------------------------------------------------------------------
    // Variable declaration for emon_calc procedure
    //--------------------------------------------------------------------------------------
    int sampleV;                        //sample_ holds the raw analog read value
    int sampleI;

    double lastFilteredV,filteredV;          //Filtered_ is the raw analog value minus the DC offset
    double filteredI;
    double offsetV;                          //Low-pass filter output
    double offsetI;                          //Low-pass filter output

    double phaseShiftedV;                             //Holds the calibrated phase shifted voltage.

    double sqV,sumV,sqI,sumI,instP,sumP;              //sq = squared, sum = Sum, inst = instantaneous

    int startV;                                       //Instantaneous voltage at start of sample window.

    boolean lastVCross, checkVCross;                  //Used to measure number of times threshold is crossed.
	
	Adafruit_ADS1115 ads;


};

