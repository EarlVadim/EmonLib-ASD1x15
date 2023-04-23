// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include <Adafruit_ADS1X15.h>
#include <EmonLib_ADS1x15.h>

#define ADS (0x48)

EnergyMonitor emon1;             // Create an instance

void setup()
{  
  Serial.begin(9600);  

  emon1.current(ADS, true, GAIN_ONE, RATE_ADS1115_250SPS, 111.1);          // Current : input ads_address, with init ads, Gain, Data rate, calibration. 
  emon1.voltage(ADS, false, GAIN_ONE, RATE_ADS1115_250SPS, 234.26, 1.7);   // Voltage : input ads_address, without init ads, Gain, Data rate, calibration, phase_shift
}

void loop()
{
  emon1.calcVI(0, 1, 20,2000);         // Calculate all. Irms from Single A0 pin, Vrms from A1 pin. No.of half wavelengths (crossings), time-out
 //  emon1.calcVI(10, 32, 20,2000);    // Calculate all. Irms from Differential A0-A1 pins, Vrms from A2-A3 pins. No.of half wavelengths (crossings), time-out
  emon1.serialprint();           // Print out all variables (realpower, apparent power, Vrms, Irms, power factor)
  
  float realPower       = emon1.realPower;        //extract Real Power into variable
  float apparentPower   = emon1.apparentPower;    //extract Apparent Power into variable
  float powerFActor     = emon1.powerFactor;      //extract Power Factor into Variable
  float supplyVoltage   = emon1.Vrms;             //extract Vrms into Variable
  float Irms            = emon1.Irms;             //extract Irms into Variable
}
