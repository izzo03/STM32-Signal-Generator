
#ifndef TM_HD44780_H
#define TM_HD44780_H 120
/*
LCD		STM32F4XX		DESCRIPTION

GND		GND				Ground
VCC		+5V				Power supply for LCD
V0		Potentiometer	Contrast voltage. Connect to potentiometer
RS		PF15				Register select, can be overwritten in your project's defines.h file
RW		GND				Read/write
E		PE13				Enable pin, can be overwritten in your project's defines.h file
D0		-				Data 0 - doesn't care
D1		-				Data 1 - doesn't care
D2		-				Data 2 - doesn't care
D3		-				Data 3 - doesn't  care
D4		PF14			Data 4, can be overwritten in your project's defines.h file
D5		PE11			Data 5, can be overwritten in your project's defines.h file
D6		PE9			Data 6, can be overwritten in your project's defines.h file
D7		PF13			Data 7, can be overwritten in your project's defines.h file
A		+3V3			Back light positive power
K		GND				Ground for back light

*/
//RS - Register select pin
#define HD44780_RS_PORT     GPIOF
#define HD44780_RS_PIN      LCD_RS_Pin
//E - Enable pin
#define HD44780_E_PORT      GPIOE
#define HD44780_E_PIN       LCD_E_Pin
//D4 - Data 4 pin
#define HD44780_D4_PORT     GPIOF
#define HD44780_D4_PIN      LCD_D4_Pin
//D5 - Data 5 pin
#define HD44780_D5_PORT     GPIOE
#define HD44780_D5_PIN      LCD_D5_Pin
//D6 - Data 6 pin
#define HD44780_D6_PORT     GPIOE
#define HD44780_D6_PIN      LCD_D6_Pin
//D7 - Data 7 pin
#define HD44780_D7_PORT     GPIOF
#define HD44780_D7_PIN      LCD_D7_Pin
//@endverbatim

/* 4 bit mode */
/* Control pins, can be overwritten */
/* RS - Register select pin */
#ifndef HD44780_RS_PIN
#define HD44780_RS_PORT				GPIOF
#define HD44780_RS_PIN				GPIO_PIN_15
#endif
/* E - Enable pin */
#ifndef HD44780_E_PIN
#define HD44780_E_PORT				GPIOE
#define HD44780_E_PIN				GPIO_PIN_13
#endif
/* Data pins */
/* D4 - Data 4 pin */
#ifndef HD44780_D4_PIN
#define HD44780_D4_PORT				GPIOF
#define HD44780_D4_PIN				GPIO_PIN_14
#endif
/* D5 - Data 5 pin */
#ifndef HD44780_D5_PIN
#define HD44780_D5_PORT				GPIOE
#define HD44780_D5_PIN				GPIO_PIN_11
#endif
/* D6 - Data 6 pin */
#ifndef HD44780_D6_PIN
#define HD44780_D6_PORT				GPIOE
#define HD44780_D6_PIN				GPIO_PIN_9
#endif
/* D7 - Data 7 pin */
#ifndef HD44780_D7_PIN
#define HD44780_D7_PORT				GPIOF
#define HD44780_D7_PIN				GPIO_PIN_13
#endif


void TM_HD44780_Init(uint8_t cols, uint8_t rows);

/**
 * @brief  Turn display on
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOn(void);

/**
 * @brief  Turn display off
 * @param  None
 * @retval None
 */
void TM_HD44780_DisplayOff(void);

/**
 * @brief  Clears entire LCD
 * @param  None
 * @retval None
 */
void TM_HD44780_Clear(void);

/**
 * @brief  Puts string on lcd
 * @param  x location
 * @param  y location
 * @param  *str: pointer to string to display
 * @retval None
 */
void TM_HD44780_Puts(uint8_t x, uint8_t y, char* str);

/**
 * @brief  Enables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOn(void);

/**
 * @brief  Disables cursor blink
 * @param  None
 * @retval None
 */
void TM_HD44780_BlinkOff(void);

/**
 * @brief  Shows cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOn(void);

/**
 * @brief  Hides cursor
 * @param  None
 * @retval None
 */
void TM_HD44780_CursorOff(void);

/**
 * @brief  Scrolls display to the left
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollLeft(void);

/**
 * @brief  Scrolls display to the right
 * @param  None
 * @retval None
 */
void TM_HD44780_ScrollRight(void);

/**
 * @brief  Creates custom character
 * @param  location: Location where to save character on LCD. LCD supports up to 8 custom characters, so locations are 0 - 7
 * @param *data: Pointer to 8-bytes of data for one character
 * @retval None
 */
void TM_HD44780_CreateChar(uint8_t location, uint8_t* data);

/**
 * @brief  Puts custom created character on LCD
 * @param  location: Location on LCD where character is stored, 0 - 7
 * @retval None
 */
void TM_HD44780_PutCustom(uint8_t x, uint8_t y, uint8_t location);


#endif

