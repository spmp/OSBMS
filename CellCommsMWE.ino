/**
 * MWE for using the CellComms and FEC libraries with the OSBMS v2.0 cell top modules.
 * It is advised to use PlatformIO to build; Place `CellComms` and `FEC` directories in `lib` after importing
 **/
#include <Arduino.h>

#include <CellComms.h>
#include <FEC.h>

// Serial 0 defines
#define SERIAL0_BAUD            9600

// Serial 2 defines
#define SERIAL2_TXPIN           26
#define SERIAL2_RXPIN           27
HardwareSerial Serial2(2);

// BMS CellComms defines
#define NUM_CELLS               1

// Cell comms constructor
CellComms cells(NUM_CELLS, Serial2);

// debug functions
void debugPrint(String debugMessage) {
  #ifdef debug
    Serial.println(debugMessage);
  #endif
}

void setup()   {
  Serial.begin(SERIAL0_BAUD);
  debugPrint("Setup Serial0");

  // Initialis Serial2
  Serial2.begin(CELLCOMMS_BAUD, SERIAL_8N1, SERIAL2_RXPIN, SERIAL2_TXPIN);
  debugPrint("Setup Serial2");
}

void loop() {
  // Send data
  cells.sendMillivolts(3650);
  delay(1000);
  // Read data
  cells.readCells();
  Serial.print(" Voltage: ");
  Serial.print(cells.millivoltsMean());
  Serial.print("mv, temperature: ");
  Serial.print(cells.temperatureMean());
  Serial.print(", balancing: ");
  Serial.print(cells.balancingNum());
  Serial.print(", over temperature: ");
  Serial.print(cells.overTemperatureNum());
  Serial.print(", over voltage: ");
  Serial.print(cells.overVoltageNum());
  Serial.print(", under voltage: ");
  Serial.print(cells.underVoltageNum());
  Serial.print("\n");
}
