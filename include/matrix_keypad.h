#ifndef MATRIX_KEYPAD_H
#define MATRIX_KEYPAD_H

// be sure to use your own device header
#include "stm32f411xe.h"

#define ROW_PINS 4
#define COLUMN_PINS 4

#include <stdint.h> // uint...
#include <stdbool.h> // bool

typedef struct keypad
{
	GPIO_TypeDef*	row_ports[ROW_PINS];
	uint8_t 		row_pins[ROW_PINS];
	GPIO_TypeDef*	column_ports[COLUMN_PINS];
	uint8_t 		column_pins[COLUMN_PINS];
} keypad;


/**
 * @brief Initialize keypad by setting up GPIO clocks and configuring row pins as output and column pins and input (with internal pull-up resistors).
 *
 * Note, I wrote this code for a 4x4 matrix keypad but can be customized by modifying the defines at the top of this file.
 *
 * For matrix keypads, we send signal to it via making the row pins a HIGH signal output. Then we set the column pins as inputs with pull-up resistors (here I use the internal resistors but you can use external ones if desired). For the column pins we trigger an interrupt and determine which row is currently a low signal (it will be continuously cycling in the while loop) and determine which column button was pressend to finally figure out which row-column combination has the low signal.
 *
 * @param _keypad Pointer to keypad struct
 * @param row_ports GPIO ports for row pins (e.g. GPIOB). These are ordered in ascending order (e.g. row 1 --> row 4).
 * @param row_pins Corresponding pins to row_ports. These are ordered in ascending order (e.g. row 1 --> row 4).
 * @param column_ports GPIO ports for column pins (e.g. GPIOB). These are ordered in ascending order (e.g. column 1 --> column 4).
 * @param column_pins Corresponding pins to column_ports. These are ordered in ascending order (e.g. column 1 --> column 4).
 *
 */
void keypad_init(
		keypad* 		_keypad,
		GPIO_TypeDef* 	row_ports[ROW_PINS],
		uint8_t 		row_pins[ROW_PINS],
		GPIO_TypeDef* 	column_ports[COLUMN_PINS],
		uint8_t 		column_pins[COLUMN_PINS]);

/**
 * @brief Infinitely cycle through the rows until an interrupt occurs, essentially "listening" for keypresses as we cycle. You can break the loop by passing 'false' in the second argument.
 *
 * @warning This is a completely blocking loop (loops indefinitely). This is mainly designed for an MCU's main function to be dedicated to a keypad.
 *
 * @param _keypad Pointer to keypad struct
 * @param active 'true' if the keypad is actively listening, 'false' to break the loop and stop listening (this is added in case you want to stop listening when an interrupt occurrs and you swap the active flag).
 */
void keypad_listen(
		keypad* _keypad,
		bool active);

/**
 * @brief Check if a particular row was pressed. Typically used within an interrupt.
 *
 * @note row argument is ONE-INDEXED (i.e. it starts at 1 instead of 0)
 *
 * When using keypad_listen(), the current active row is continuously cycled. Whenever an interrupt occurrs by pressing a column button, one can use this function to determine which row was pressed and thus have the exact (row, column) button that was pressed on the keypad.
 *
 * @param _keypad Pointer to keypad struct
 * @param row The row to check
 *
 * @return 'true' if specified row was pressed, 'false' otherwise.
 */
bool keypad_row_pressed(
		keypad* _keypad,
		uint8_t row);
#endif
