// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include <Adafruit_ADS1X15.h>
#include <EmonLib_ADS1x15.h>

#define ADS (0x48)

EnergyMonitor emon1;                   // Create an instance

void setup()
{  
  Serial.begin(9600);
  
  emon1.current(ADS, true, GAIN_ONE, RATE_ADS1115_250SPS, 111.1);          // Current : input ads_address, with init ads, Gain, Data rate, calibration.


}

void loop()
{
  double Irms = emon1.calcIrms(0, 1480);  // Calculate Irms only from Single A0 pin. , NUMBER_OF_SAMPLES
 // double Irms = emon1.calcIrms(32, 1480);  // Calculate Irms only from Differential A2-A3 pins. NUMBER_OF_SAMPLES
  
  Serial.print(Irms*230.0);            // Apparent power
  Serial.print(" ");
  Serial.println(Irms);                // Irms
}
