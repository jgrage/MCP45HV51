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
 * Test
 */
MCP45HV51::MCP45HV51(uint8_t i2c_address){
	address = i2c_address;
}

void MCP45HV51::transmit(uint8_t command, uint8_t data){
	Wire.beginTransmission(address);
	Wire.write(command);
	Wire.write(data);
	Wire.endTransmission();
}

void MCP45HV51::transmit(uint8_t command){
	Wire.beginTransmission(address);
	Wire.write(command);
	Wire.endTransmission();
}

int16_t MCP45HV51::receive(uint8_t command){
	uint8_t count = 0;
	int16_t response = -1;		// ensure that response is negative (invalid) if Wire is not available and while loop will not be entered or the transmission is not complete.
	
	Wire.beginTransmission(address);	// select device to be read
	Wire.write(command);				// select memory location to be read by issuing a write command
	Wire.endTransmission(false);		// send repeated start bit. no data will be transmitted
	Wire.requestFrom(address, 2);		// request data from previously accessed memory location (write bit is now cleared). First byte is always 0 (see manual). Second byte contains the 8 bit value.
	
	while(Wire.available()){
		uint8_t byte = Wire.read();
		
		// second byte contains the data we want
		if(count == 1){
			response = (int16_t)byte;	// cast to unsigned int
		}
		count++;
	}
	return response;
}

void MCP45HV51::set(uint8_t data){
	command_byte = 0b00000000;
	transmit(command_byte, data);
}

void MCP45HV51::set_tcon(uint8_t data){
	command_byte = 0b01000000;
	transmit(command_byte, data);
}

void MCP45HV51::increment(void){
	command_byte = 0b00000100;
	transmit(command_byte);
}

void MCP45HV51::decrement(void){
	command_byte = 0b00001000;
	transmit(command_byte);
}

int16_t MCP45HV51::get(void){
	command_byte = 0b00001100;
	return receive(command_byte);
}

int16_t MCP45HV51::get_tcon(void){
	command_byte = 0b01001100;
	return receive(command_byte);
}
