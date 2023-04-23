// EmonLibrary examples openenergymonitor.org, Licence GNU GPL V3

#include <Adafruit_ADS1X15.h>
#include <EmonLib_ADS1x15.h>

#define ADS (0x48)

EnergyMonitor emon1;                   // Create an instance

void setup()
{
  Serial.begin(9600);

  emon1.voltage(ADS, true, GAIN_ONE, RATE_ADS1115_250SPS, 234.26, 1.7);   // Voltage : input ads_address, with init ads, Gain, Data rate, calibration, phase_shift
  
}

void loop()
{
  double Vrms = emon1.calcVrms(0, 20, 2000);  // Calculate Vrms only from Single A0 pin. No.of half wavelengths (crossings), time-out
  // double Vrms = emon1.calcVrms(32, 20, 2000);  // Calculate Vrms only from Differential A2-A3 pins. No.of half wavelengths (crossings), time-out

  Serial.print("Voltage: ");
  Serial.println(Vrms);                    // Vrms
}
