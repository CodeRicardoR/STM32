/*
 * ADC_Library.h
 *
 *  Created on: Nov 5, 2023
 *      Author: ricar
 */

#ifndef INC_ADC_LIBRARY_H_
#define INC_ADC_LIBRARY_H_

#include<stdint.h>
#include "main.h"

uint32_t Read_Channel(uint32_t);
float DataConversion(uint32_t);

#endif /* INC_ADC_LIBRARY_H_ */
