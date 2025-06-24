
/** Includes ---------------------------------------------------------------- */
#include "keypad.h"

GPIO_InitTypeDef _GPIO_InitStructKeypad;

/** Public functions -------------------------------------------------------- */
/**
  ******************************************************************************
  * @brief	Initialize GPIO pins for keypad.
  * @param	None
  * @retval	None
  ******************************************************************************
  */
/**
  ******************************************************************************
  * @brief	Get which key is pressed by scanning the columns and read the rows.
  * @param	None
  * @retval	Pressed key char value.
  ******************************************************************************
  */
uint8_t KeypadGetKey()
{
	// Scan column 0 (column 0 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(GPIOF, COL_1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, COL_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL_4_GPIO_Port, COL_4_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(ROW_1_GPIO_Port, ROW_1_Pin))
		return '1';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_2_Pin))
		return '4';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_3_Pin))
		return '7';
	if (!HAL_GPIO_ReadPin(ROW_4_GPIO_Port, ROW_4_Pin))
		return '*';
		
	// Scan column 1 (column 1 pin is grounded, other column pins is open drain)
	HAL_GPIO_WritePin(GPIOF, COL_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, COL_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL_4_GPIO_Port, COL_4_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(ROW_1_GPIO_Port, ROW_1_Pin))
		return '2';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_2_Pin))
		return '5';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_3_Pin))
		return '8';
	if (!HAL_GPIO_ReadPin(ROW_4_GPIO_Port, ROW_4_Pin))
		return '0';
		
	// Scan column 2 (column 2 pin is grounded, other column pins is open drain)
HAL_GPIO_WritePin(GPIOF, COL_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(COL_4_GPIO_Port, COL_4_Pin,GPIO_PIN_SET);
	HAL_Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(ROW_1_GPIO_Port, ROW_1_Pin))
		return '3';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_2_Pin))
		return '6';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_3_Pin))
		return '9';
	if (!HAL_GPIO_ReadPin(ROW_4_GPIO_Port, ROW_4_Pin))
          return '#';
		
	// Scan column 3 (column 3 pin is grounded, other column pins is open drain)
HAL_GPIO_WritePin(GPIOF, COL_1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_2_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOF, COL_3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(COL_4_GPIO_Port, COL_4_Pin,GPIO_PIN_RESET);
	HAL_Delay(1);
	// Read rows
	if (!HAL_GPIO_ReadPin(ROW_1_GPIO_Port, ROW_1_Pin))
		return 'A';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_2_Pin))
		return 'B';
	if (!HAL_GPIO_ReadPin(GPIOC, ROW_3_Pin))
		return 'C';
	if (!HAL_GPIO_ReadPin(ROW_4_GPIO_Port, ROW_4_Pin))
		return 'D';
	
	return KEYPAD_NO_PRESSED;
}

/********************************* END OF FILE ********************************/
/******************************************************************************/
