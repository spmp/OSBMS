/***********************************************************
  CellComms.h - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Re-imagined by Jasper Aoranig, August, 2017.
  Released into the public domain.

  Instantiate the class with the number of cells and a
  valid Serial/Stream device.
***********************************************************/

#ifndef CellComms_h
#define CellComms_h

#include <iostream>
#include <vector>
#include <algorithm> //transform
#include <numeric>   //accumulate
#include "Arduino.h"
#include "HardwareSerial.h"
#include "Stream.h"
#include "FEC.h"

// Cell Monitors from v2.0 onwards are capable of 19200 baud
#define CELLCOMMS_BAUD			   19200

#define CTMUNENCODEDPACKETSIZE 4
#define CTMENCODEDPACKETSIZE   6

class CellData {
  public:
  CellData(void) {};
	uint16_t millivolts     = 0;
	uint16_t temperature    = 0;
	uint8_t balancing       = 0;
	uint8_t overTemperature = 0;
	uint8_t overVoltage     = 0;
	uint8_t underVoltage    = 0;
};

class CellComms
{
  public:
  // Constructor and Initialisation
	CellComms(int numCells, Stream & port);
  void     begin(void);
  // Cell interaction
	void     sendMillivolts(int16_t millivolts);
	void     sendMillivolts(void);
	uint8_t  readCells(void);
  // Cell stats
  uint16_t millivoltsMean(void);
  uint16_t millivoltsMax(void);
  uint16_t millivoltsMin(void);
  uint16_t temperatureMean(void);
  uint16_t temperatureMax(void);
  uint16_t temperatureMin(void);
  uint8_t  balancingNum(void);
  uint8_t  overVoltageNum(void);
  uint8_t  underVoltageNum(void);
  uint8_t  overTemperatureNum(void);
  // Cell slices
  std::vector<uint16_t> millivoltsVect(void);
  std::vector<uint16_t> temperatureVect(void);
  std::vector<uint8_t>  balancingVect(void);
  std::vector<uint8_t>  overVoltageVect(void);
  std::vector<uint8_t>  underVoltageVect(void);
  std::vector<uint8_t>  overTemperatureVect(void);
  // Supporting class variables
  Stream & _Port;                       // Serial device
  int cellNum;                          // Number of cells
  std::vector<CellData> cellDataVect;   // Vector of CellData
  private:
  void                  generateMasterMessage(int16_t millivolts);
  void                  getSerialVector();
  CellData              extractCellData(uint8_t decodedArr[]);
  uint8_t               extractPayloadsToCellData(void );
  // Variables
  uint8_t               txData[CTMENCODEDPACKETSIZE];
  std::vector<uint8_t>  recievedSerialBytesVect;
};

#endif
