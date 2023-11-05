/*
 * LCD_Library.h
 *
 *  Created on: Nov 01, 2023
 *      Author: Ricardo Rojas
 */

#include <main.h>
#include <string.h>

#ifndef INC_LCD_LIBRARY_H_
#define INC_LCD_LIBRARY_H_

#define LCD_PinType uint16_t
#define LCD_PortType GPIO_TypeDef*

//Commands definitions:
#define ClearDisplay    0x01
#define ReturnHome      0x02
#define CGRamCreate     0x40

typedef struct{
	LCD_PortType *dataport;
	LCD_PinType *datapin;

	LCD_PortType RS_port;
	LCD_PinType RS_pin;

	LCD_PortType ENA_port;
	LCD_PinType ENA_pin;
}LCD_Struct_t;

//Functions
LCD_Struct_t LCD_Create(LCD_PortType port[], LCD_PinType pin[],
		LCD_PortType portRS, LCD_PinType pinRS,
		LCD_PortType portENA, LCD_PinType pinENA);

void LCD_Init(LCD_Struct_t *LCD);
void LCD_Command(LCD_Struct_t *LCD, uint8_t command);
void LCD_Character(LCD_Struct_t *LCD, char data);
void LCD_String(LCD_Struct_t *LCD, char *str);
void LCD_Gotoxy(LCD_Struct_t *LCD, uint8_t x, uint8_t y);
void LCD_CreateCharacter(LCD_Struct_t *LCD, uint8_t address, char *data);

#endif /* INC_LCD_LIBRARY_H_ */

