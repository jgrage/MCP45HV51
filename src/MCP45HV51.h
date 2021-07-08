/**
 * @file MCP45HV51.hpp
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

#ifndef _AD5724_H
#define _AD5724_H

/**
 * Test
 */
#define DEFAULT_ADDRESS 0b0111100



class MCP45HV51 {
	
	public:
		MCP45HV51(uint8_t i2c_address = DEFAULT_ADDRESS);
		
		void set(uint8_t data);
		void set_tcon(uint8_t data);
		void increment(void);
		void decrement(void);
		
		int16_t get(void);
		int16_t get_tcon(void);
		
	
	private:
		void transmit(uint8_t command);
		void transmit(uint8_t command, uint8_t data);
		int16_t receive(uint8_t);
		
		uint8_t address;
		uint8_t command_byte;
};

#endif
