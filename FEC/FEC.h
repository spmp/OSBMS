/**
 *  FEC library of free functions to encode and decode objects using Forward
 *  Error correction
 **/

#ifndef FEC_h
#define FEC_h

#include "Arduino.h"
#include <vector>

/*******************************************************************************
 * Encodes/Decodes with low-density parity-check codes
 * The first byte (header) is not encoded, thereafter the 8-bits are encoded
 * into 12-bits. The bytes are sent in the order;
 *   B1   B2   LDPC   B3   B4   LDPC   ...
 * Hence every 3rd-byte has the error correction bits for the previous 2-bytes.
 * The interleaving is done over blocks of 6-bytes.
 ******************************************************************************/
 uint8_t fecEncode(uint8_t p_msg[], uint8_t u8_msg_len);
 uint8_t fecDecode(volatile uint8_t en_msg[], uint8_t de_msg[], uint8_t u8_msg_len);
 void    fecuint8_Decode(unsigned int *u16_code_word);

#endif
