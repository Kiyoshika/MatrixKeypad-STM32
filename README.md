# Matrix Keypad - STM32
This is a simple API to help with using matrix keypads. In theory, this API should support any NxM keypad with the proper code changes in the header. By default, I wrote this for a 4x4 keypad.

# Supported Boards
This code in particular was written for the STM32F411xx (Black Pill) but if you modify the registers it should work for your board.

# How it Works
For matrix keypads, we set the ROW pins as output and COLUMN pins as inputs with pullup resistors. We cycle through the rows and setting one LOW with all other rows HIGH at small delay intervals. Interrupts are setup on the COLUMN pins and API provides a way to check which row was pressed to fully determine the (row, column) pair. See example code below for the interrupt setup and API usage.

# Notes
1. Due to interrupts being handled differently from MCU to MCU, I am leaving it up to programmer's responsibility to enable and implement the interrupts. All the clocks and necessary settings should be taken care of by the API but you must implement the handler callbacks yourself, see example below.

2. The ports & pins arrays are ordered in ASCENDING order, meaning `row_ports[0]` refers to the GPIO port of ROW 1 and `row_ports[3]` refers to the GPIO port of ROW 4 and so on for the others.

# Example (4x4 Keypad)
Example code below shows example usage with a 4x4 matrix keypad.
```c
#include "stm32f411xe.h"
#include "core_cm4.h"
#include "matrix_keypad.h"

// making keypad global so we can reference it in interrupts
keypad kp;

// column pins are on line 2, 3, 4, 5

void EXTI2_IRQHandler(void) // COLUMN 1
{
	if (keypad_row_pressed(&kp, 1))
		// row 1, column 1 pressed
	else if (keypad_row_pressed(&kp, 2))
		// row 2, column 1 pressed
	else if (keypad_row_pressed(&kp, 3))
		// row 3, column 1 pressed
	else if (keypad_row_pressed(&kp, 4))
		// row 4, column 1 pressed

	EXTI->PR |= (1 << 2);
}

void EXTI3_IRQHandler(void) // COLUMN 2
{
	if (keypad_row_pressed(&kp, 1))
		// row 1, column 2 pressed
	else if (keypad_row_pressed(&kp, 2))
		// row 2, column 2 pressed
	else if (keypad_row_pressed(&kp, 3))
		// row 3, column 2 pressed
	else if (keypad_row_pressed(&kp, 4))
		// row 4, column 2 pressed

	EXTI->PR |= (1 << 3);
}

void EXTI4_IRQHandler(void) // COLUMN 3
{
	if (keypad_row_pressed(&kp, 1))
		// row 1, column 3 pressed
	else if (keypad_row_pressed(&kp, 2))
		// row 2, column 3 pressed
	else if (keypad_row_pressed(&kp, 3))
		// row 3, column 3 pressed
	else if (keypad_row_pressed(&kp, 4))
		// row 4, column 3 pressed

	EXTI->PR |= (1 << 4);
}

// ****WARNING***: if you have multiple buttons connected a multi-line interrupt like below
// then you must do some extra work to figure out which column button was pressed.
// You could probably use similar logic to keypad_row_pressed if you peek into the header.
// In my case I only have a single button connected to this interrupt line so I don't have to worry about it here.

void EXTI9_5_IRQHandler(void) // COLUMN 4
{
	if (keypad_row_pressed(&kp, 1))
		// row 1, column 4 pressed
	else if (keypad_row_pressed(&kp, 2))
		// row 2, column 4 pressed
	else if (keypad_row_pressed(&kp, 3))
		// row 3, column 4 pressed
	else if (keypad_row_pressed(&kp, 4))
		// row 4, column 4 pressed

	EXTI->PR |= (1 << 5);
}

int main()
{
	// column pins are on line 2, 3, 4, 5
	NVIC_EnableIRQ(EXTI2_IRQn);
	NVIC_SetPriority(EXTI2_IRQn, 0);

	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_SetPriority(EXTI3_IRQn, 0);

	NVIC_EnableIRQ(EXTI4_IRQn);
	NVIC_SetPriority(EXTI4_IRQn, 0);

	NVIC_EnableIRQ(EXTI9_5_IRQn);
	NVIC_SetPriority(EXTI9_5_IRQn, 0);

	// R1, R2, R3, R4
	GPIO_TypeDef* row_ports[4] = {
		GPIOA, GPIOA, GPIOB, GPIOB
	};

	uint8_t row_pins[4] = {
		6, 7, 0, 1
	};

	// C1, C2, C3, C4
	GPIO_TypeDef* column_ports[4] = {
		GPIOA, GPIOA, GPIOA, GPIOA
	};

	uint8_t column_pins[4] = {
		2, 3, 4, 5
	};

	keypad_init(
		&kp,
		row_ports,
		row_pins,
		column_ports,
		column_pins);

	// for debugging, setup PA12 as output GPIO (GPIOA clock should already be enabled by API)
	//RCC->AHB1ENR |= (1 << 0);	

	GPIOA->MODER &= ~(0b11 << (2 * 12));
	GPIOA->MODER |= (0b01 << (2 * 12));
	
	while(1)
	{
		// listen for inputs from keypad (cycle through rows)
		keypad_listen(&kp, true);
	}
	return 0;
}
```
