/*
 * LCD_Library.c
 *
 *  Created on: Nov 01, 2023
 *      Author: Ricardo Rojas
 */

#include "LCD_Library.h"


LCD_Struct_t LCD_Create(LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType portRS, LCD_PinType pinRS,
		LCD_PortType portENA, LCD_PinType pinENA){

	LCD_Struct_t LCD;

	LCD.dataport = port;
	LCD.datapin = pin;
	LCD.RS_port = portRS;
	LCD.RS_pin = pinRS;
	LCD.ENA_port = portENA;
	LCD.ENA_pin = pinENA;

	return LCD;
}


void LCD_Init(LCD_Struct_t *LCD){

	HAL_Delay(20);
	LCD_Command(LCD, 0x30);
	HAL_Delay(5);
	LCD_Command(LCD, 0x30);
	HAL_Delay(1);
	LCD_Command(LCD, 0x30);
	HAL_Delay(1);

	LCD_Command(LCD, 0x32);
	LCD_Command(LCD, 0x28);
	LCD_Command(LCD, 0x0C);
	LCD_Command(LCD, 0x01);
	LCD_Command(LCD, 0x06);

	return;
}

void LCD_Command(LCD_Struct_t *LCD, uint8_t command){
	uint8_t cmd = 0;
	uint8_t j;

	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);
	HAL_GPIO_WritePin(LCD->RS_port, LCD->RS_pin, RESET);

	cmd = (command>>4)&0x0F;
	for(j=0; j<4; j++){
		HAL_GPIO_WritePin(LCD->dataport[j], LCD->datapin[j], (cmd>>j)&0x01);
	}
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);

	cmd = (command)&0x0F;
	for(j=0; j<4; j++){
		HAL_GPIO_WritePin(LCD->dataport[j], LCD->datapin[j], (cmd>>j)&0x01);
	}
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);

	return;
}
void LCD_Character(LCD_Struct_t *LCD, char data){
	uint8_t dat = 0;
	uint8_t j;

	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);
	HAL_GPIO_WritePin(LCD->RS_port, LCD->RS_pin, SET);

	dat = (data>>4)&0x0F;
	for(j=0; j<4; j++){
		HAL_GPIO_WritePin(LCD->dataport[j], LCD->datapin[j], (dat>>j)&0x01);
	}
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);

	dat = (data)&0x0F;
	for(j=0; j<4; j++){
		HAL_GPIO_WritePin(LCD->dataport[j], LCD->datapin[j], (dat>>j)&0x01);
	}
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCD->ENA_port, LCD->ENA_pin, RESET);

	return;
}

void LCD_String(LCD_Struct_t *LCD, char *str){
	uint8_t j;

	for(j=0; j<strlen(str); j++){
		LCD_Character(LCD, str[j]);
	}

	return;
}
void LCD_Gotoxy(LCD_Struct_t *LCD, uint8_t x, uint8_t y){
	uint8_t conta_address;

	switch (y) {
	case 0:
		conta_address = 0x80;
		break;
	case 1:
		conta_address = 0xC0;
		break;
	case 2:
		conta_address = 0x94;
		break;
	case 3:
		conta_address = 0xD4;
		break;
	default:
		break;
	}

	conta_address = conta_address + x;
	LCD_Command(LCD, conta_address);

	return;
}

void LCD_CreateCharacter(LCD_Struct_t *LCD, uint8_t address, char *data){
	uint8_t CMD_CGRAM, j;

	CMD_CGRAM = CGRamCreate | address;
	LCD_Command(LCD, CMD_CGRAM);
	HAL_Delay(1);
	for(j=0; j<8; j++){
		LCD_Character(LCD, data[j]);
	}
	return;
}


