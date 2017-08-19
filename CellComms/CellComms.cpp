/**********************************************************
  CellComms.cpp - Library for reading the OSBMS Cell Monitors.
  Created by David Burkitt, March 8, 2017.
  Released into the public domain.
**********************************************************/
#include "Arduino.h"
#include "CellComms.h"

/********************** Contructor and initialisation functions ***************/
// Constructor, sets the initial cellDataVect size and CTM serial device
CellComms::CellComms (int numCells, Stream & port) : _Port (port), cellDataVect(numCells)
{
		cellNum = numCells;
}

void CellComms::begin(void) {}

/********************** Cell interaction functions ****************************/

/**
 * Send a message which initialises cell top communications.
 * The message sets the balancing voltage for the cell top modules
 * Cells respond by appending their data to the message
 * @param millivolts - milliovolts to begin balancing
 **/
void CellComms::sendMillivolts(int16_t millivolts) {
	// Encode millivolts to CTM master message
	generateMasterMessage(3650);

	// Encode master message with FEC error correction
	fecEncode(txData, CTMUNENCODEDPACKETSIZE);

	// Send encoded and FEC'd message over the wire
	_Port.write(txData, CTMENCODEDPACKETSIZE);
}

/**
 * Overloaded function that does not change the balancing voltage
 **/
void CellComms::sendMillivolts(void) {
	sendMillivolts(0);
}

/**
 * Read the serial data and decode into CellData
 * @return The number of cells decoded
 **/
uint8_t CellComms::readCells(){
	// Read the cells
	getSerialVector();

	// Extract the payloads into CellData
	return extractPayloadsToCellData();
}

/********************** Cell stat functions ***********************************/
/**
 * Get the specific _thing_ from CellData objects
 **/
uint16_t getMillivolts(CellData  cellData)        {return cellData.millivolts; }
uint16_t getTemperature(CellData  cellData)       {return cellData.temperature; }
uint8_t  getBalancing(CellData  cellData)         {return cellData.balancing; }
uint8_t  getOverVoltage(CellData  cellData)       {return cellData.overVoltage; }
uint8_t  getUnderVoltage(CellData  cellData)      {return cellData.underVoltage; }
uint8_t  getOverTemperature(CellData  cellData)   {return cellData.overTemperature; }
/**
 * Accumulatore function (x, y) -> z
 **/
uint8_t  accumulateOverOne(uint8_t x, uint8_t y) {
	if( y > 0 ) {
		return x + 1;
	} else {
		return x;
	}
}

/**
 * Millivolt stats
 **/
uint16_t CellComms::millivoltsMean(void){
	// Get a vector of cell millivolts
	std::vector<uint16_t> tempVector = millivoltsVect();

	// return the mean using an accumulator to sum, divided by size
	if (tempVector.size() > 0) {
		return std::accumulate(
				tempVector.begin(),
				tempVector.end(),
			0) / tempVector.size();
	} else {
		return 0;
	}

}

uint16_t CellComms::millivoltsMax(void){
	// Get a vector of cell millivolts
	std::vector<uint16_t> tempVector = millivoltsVect();

	// return the max
	return *std::max_element(
			tempVector.begin(),
			tempVector.end()
		);
}

uint16_t CellComms::millivoltsMin(void){
	// Get a vector of cell millivolts
	std::vector<uint16_t> tempVector = millivoltsVect();

	// return the max
	return *std::min_element(
			tempVector.begin(),
			tempVector.end()
		);
}


/**
 * Temperature stats
 **/
uint16_t CellComms::temperatureMean(void){
	// Get a vector of cell temperatures
	std::vector<uint16_t> tempVector = temperatureVect();

	// return the mean using an accumulator to sum, divided by size
	if (tempVector.size() > 0) {
		return std::accumulate(
				tempVector.begin(),
				tempVector.end(),
			0) / tempVector.size();
	} else {
		return 0;
	}

}

uint16_t CellComms::temperatureMax(void){
	// Get a vector of cell temperatures
	std::vector<uint16_t> tempVector = temperatureVect();

	// return the max
	return *std::max_element(
			tempVector.begin(),
			tempVector.end()
		);
}

uint16_t CellComms::temperatureMin(void){
	// Get a vector of cell temperatures
	std::vector<uint16_t> tempVector = temperatureVect();

	// return the max
	return *std::min_element(
			tempVector.begin(),
			tempVector.end()
		);
}

uint8_t  CellComms::balancingNum(void) {
	// Get a vector of cell balancing state
	std::vector<uint8_t> tempVector = balancingVect();

	// use an accumulator to count cells which are balancing
	return std::accumulate(
		tempVector.begin(),
		tempVector.end(),
		0, accumulateOverOne);
}

uint8_t  CellComms::overVoltageNum(void) {
	// Get a vector of cell over voltage state
	std::vector<uint8_t> tempVector = overVoltageVect();

	// use an accumulator to count cells which are balancing
	return std::accumulate(
		tempVector.begin(),
		tempVector.end(),
		0, accumulateOverOne);
}

uint8_t  CellComms::underVoltageNum(void) {
	// Get a vector of cell under voltage state
	std::vector<uint8_t> tempVector = underVoltageVect();

	// use an accumulator to count cells which are balancing
	return std::accumulate(
		tempVector.begin(),
		tempVector.end(),
		0, accumulateOverOne);
}

uint8_t  CellComms::overTemperatureNum(void) {
	// Get a vector of cell over temperature state
	std::vector<uint8_t> tempVector = overTemperatureVect();

	// use an accumulator to count cells which are balancing
	return std::accumulate(
		tempVector.begin(),
		tempVector.end(),
		0, accumulateOverOne);
}

/********************** Cell slices functions *********************************/
std::vector<uint16_t> CellComms::millivoltsVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint16_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getMillivolts
	);
	// Return the vector
	return tempVector;
}

std::vector<uint16_t> CellComms::temperatureVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint16_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getTemperature
	);
	// Return the vector
	return tempVector;
}

std::vector<uint8_t>  CellComms::balancingVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint8_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getBalancing
	);
	// Return the vector
	return tempVector;
}
std::vector<uint8_t>  CellComms::overVoltageVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint8_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getOverVoltage
	);
	// Return the vector
	return tempVector;
}

std::vector<uint8_t>  CellComms::underVoltageVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint8_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getUnderVoltage
	);
	// Return the vector
	return tempVector;
}
std::vector<uint8_t>  CellComms::overTemperatureVect(void) {
	// Initialise temporary output vector the same size as the number of cells
	std::vector<uint8_t> tempVector;
	tempVector.resize(cellDataVect.size());

	// Fill the temporary output vector with the required _thing_
	std::transform(
		cellDataVect.begin(),
		cellDataVect.end(),
		tempVector.begin(),
		getOverTemperature
	);
	// Return the vector
	return tempVector;
}

/*************************** Low Level (private) Functions ********************/

/**
 * Read the serial data into a vector
 **/
void CellComms::getSerialVector(void) {
		uint8_t bytesAvailable = _Port.available();
		recievedSerialBytesVect.clear();
		for (bytesAvailable; bytesAvailable > 0; bytesAvailable--) {
			recievedSerialBytesVect.push_back(_Port.read());
		}
}

void CellComms::generateMasterMessage(int16_t millivolts){
  // mask off the top 3 bits.
  millivolts        &= 0x1FFF;

  // form into a 4byte buffer and encode
  txData[0]				= (millivolts >> 8);
  txData[0]				|= 0xE0;		// set the marker bits for a master message
  txData[1]				= (millivolts);
  txData[2]				= (millivolts >> 8);
  txData[2]				|= 0xE0;		// set the marker bits for a master message
  txData[3]				= (millivolts);
}

CellData CellComms::extractCellData(uint8_t decodedArr[]) {
  // Bytes 0 and 1 make up the voltage.
  // Bytes 2 and 3 make up the temperature.
  CellData cellData;

  uint16_t tempMillivolts	= ((decodedArr[0] & 0x1F) << 8) + decodedArr[1];
  // Ensure that the measurements are meanignful, i.e the cell is alive
  if ( (tempMillivolts > 1900)      // The regulator colapses at 2v
      && (tempMillivolts < 5100) ) // the adc will max out at 5v
  {
    cellData.millivolts = tempMillivolts;
    cellData.temperature = ((decodedArr[2] & 0x0F) << 8) + decodedArr[3];
    cellData.balancing = ((decodedArr[0] & 0x80) >> 7);
    cellData.overTemperature = ((decodedArr[2] & 0x80) >> 7);
    cellData.overVoltage = ((decodedArr[2] & 0x40) >> 6);
    cellData.underVoltage = ((decodedArr[2] & 0x20) >> 5);
  }
  return cellData;
}

/*******************************************************************************
 * Take a vector of raw data, and read into cell data, six bytes at a time,
 *  ignoring the header
 * @param  recievedSerialBytesVect : Vector of raw data from serial
 * @param  cellDataVect :  Vector of CellData's to update
 * @return The number of cells read, 0 for none
 ******************************************************************************/
uint8_t CellComms::extractPayloadsToCellData(void) {
  // We want to chunk in the `recievedSerialBytesVect` six bytes at a time
  uint16_t numBits = (uint16_t) recievedSerialBytesVect.size();
  // How many 6 byte packets are there
  uint8_t numPackets = numBits / CTMENCODEDPACKETSIZE;
  if (numPackets < 2 ) {
    return 0;
  }
  // There is data, now loop through all N packets, dropping the first
  for ( uint8_t packetNum = 1; packetNum < numPackets; packetNum++ ) {
    // Copy the packet to a temporary vector
    std::vector<uint8_t> packetVector(
      recievedSerialBytesVect.begin() + (packetNum*CTMENCODEDPACKETSIZE),
      recievedSerialBytesVect.begin() + (packetNum +1)*CTMENCODEDPACKETSIZE);
    // decode the packet
    // TODO: Convert this to vector
    uint8_t* payload = &packetVector[0];
    uint8_t decoded[CTMUNENCODEDPACKETSIZE];
    fecDecode(&payload[0], &decoded[0], CTMENCODEDPACKETSIZE);
    cellDataVect[packetNum - 1] = extractCellData(decoded);
  }
  return numPackets - 1;
}
