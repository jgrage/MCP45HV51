/**
 * @file MCP45HV51.cpp
 * @author  Jonas Grage <grage@physik.tu-berlin.de>
 * @version 1.0
 *
 * @section LICENSE
 *
 * BSD 3-Clause License
 * 
 * Copyright (c) 2021, Jonas Grage
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *
 * @section DESCRIPTION
 *
 * Library for the Microchip MCP45HV51 digital potentiometer. Datasheet available
 * at https://www.microchip.com/wwwproducts/en/MCP45HV51
 */

#include <stdint.h>
#include <Wire.h>
#include "MCP45HV51.h"

/**
 * Init a MCP45HV51 object. Make sure the hardware i2c interface is enabled before calling any methods.
 * @param i2c_address Adress of the selected chip. The two least significant bits are set by connecting the A0 and A1 pins to DGND or VL.
 */
MCP45HV51::MCP45HV51(uint8_t i2c_address){
	address = i2c_address;
}

/**
 * Send a command and its argument to the chip.
 * @param command Command byte containing the register address and the command id (datasheet pages 56-57)
 * @param data Argument
 */
void MCP45HV51::transmit(uint8_t command, uint8_t data){
	Wire.beginTransmission(address);
	Wire.write(command);
	Wire.write(data);
	Wire.endTransmission();
}

/**
 * Send a command with no argument to the chip.
 * @param command Command byte containing the register address and the command id (datasheet pages 56-57)
 */
void MCP45HV51::transmit(uint8_t command){
	Wire.beginTransmission(address);
	Wire.write(command);
	Wire.endTransmission();
}

/**
 * Read data from the chip.
 * @param command Command byte containing the register address and the command id (datasheet pages 56-57)
 * @return response Signed integer containing either a positive value between 0 and 255 or -1 if the command failed
 */
int16_t MCP45HV51::receive(uint8_t command){
	uint8_t count = 0;
	int16_t response = -1;      // ensure that response is negative (invalid) if Wire is not available and while loop will not be entered or the transmission is not complete.
	
	Wire.beginTransmission(address);    // select device to be read
	Wire.write(command);                // select memory location to be read by issuing a write command
	Wire.endTransmission(false);        // send repeated start bit. no data will be transmitted
	Wire.requestFrom(address, (uint8_t) 2);       // request data from previously accessed memory location (i2c write bit is now cleared). The first byte of the response is always 0 (see manual).
	
	while(Wire.available()){
		uint8_t byte = Wire.read();
        
		// second byte contains the data we want
		if(count == 1){
			response = (int16_t)byte;   // cast to signed 16bit int
		}
		count++;
	}
	return response;
}

/**
 * Write value to the wiper register
 * @param data Value between 0 and 255
 */
void MCP45HV51::set(uint8_t data){
	command_byte = 0b00000000;
	transmit(command_byte, data);
}

/**
 * Set the terminal connection register. For more datails see datasheet page 36
 * @param data 8bit tcon register
 */
void MCP45HV51::set_tcon(uint8_t data){
	command_byte = 0b01000000;
	transmit(command_byte, data);
}

/**
 * Increment the wiper register
 */
void MCP45HV51::increment(void){
	command_byte = 0b00000100;
	transmit(command_byte);
}

/**
 * Decrement the wiper register
 */
void MCP45HV51::decrement(void){
	command_byte = 0b00001000;
	transmit(command_byte);
}

/** 
 * Get the wiper register value
 * @return signed integer containing the value between 0 and 255 or -1 in case of an error
 */
int16_t MCP45HV51::get(void){
	command_byte = 0b00001100;
	return receive(command_byte);
}

/** 
 * Get the terminal connection register value
 * @return signed integer containing the value or -1 in case of an error
 */
int16_t MCP45HV51::get_tcon(void){
	command_byte = 0b01001100;
	return receive(command_byte);
}
