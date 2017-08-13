/***********************************************************
  CellComms.h - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.

  The DARC interface board fits onto an Arduino Mega2560
  and uses Serial2 to communicate with the Cell Monitors.

  NOTE: You will need to edit
  C:\Users\<username>\AppData\Local\Arduino15\packages\arduino\hardware\avr\<version>\cores\arduino\HardwareSerial.h
  if you need to read more than 9 cells.

  User needs to select the appropriate CELL_BAUD value to match
  their Cell Monitor boards and set NUM_CELLS to match the
  number of cell monitors in their battery stack.

  sendMillivolts transmits the initiator packet
  readCells should be called after sendMillivolts allowing time
	for the message to propogate through the battery.
***********************************************************/

#ifndef CellComms_h
#define CellComms_h

#include "Arduino.h"
#include "HardwareSerial.h"
#include "Stream.h"

//#define CELL_BAUD			9600
#define CELLCOMMS_BAUD			  19200	// Cell Monitors from v2.0 onwards are capable of 19200 baud
//#define INTERLACING		1		// comment this to disable bitwise interlacing
#define CELL_INTERVAL  25

class CellData {
  public:
  CellData(void);
	unsigned int millivolts;
	unsigned int temperature;
	byte balancing;
	byte overVoltage;
	byte underVoltage;
	byte overTemperature;
};

// class CellComms
//   {
//   private:
//     Stream & port_;
//     int _numCells;
//     CellData * _CellDataArray;
//   public:
//     CellComms (int numCells, Stream & port);
//     void begin ();
//     void doShit (void);
//   };





class CellComms
{
  public:
	CellComms(int numCells, Stream & port);
  // void begin(Stream & port);
  void doShit(void);
  byte getCellsData(void);
  byte getCellsData(int millivolts);
	void sendMillivolts(int millivolts);
	int readCells(void);
	int getCellsAveV(void);
	int getCellsMinV(void);
	int getCellsMaxV(void);
	int getCellsAveT(void);
	int getCellsMinT(void);
	int getCellsMaxT(void);
	int getCellsBalancing(void);
	int getCellsOverVolt(void);
	int getCellsUnderVolt(void);
	int getCellsOverTemp(void);
	int getMinVCell(void);
	int getMaxVCell(void);
	int getMinTCell(void);
	int getMaxTCell(void);
  Stream & _Port;
  int _numCells;
  private:
	byte		fecEncode(byte p_msg[]);
	byte 		fecDecode(volatile byte en_msg[], byte de_msg[], byte u8_msg_len);
	void 		fecByteDecode(unsigned int *u16_code_word);
	byte 		_tx_packet[];
  CellData * _CellDataArray;
}; // ------------------------------------------------------

#endif
