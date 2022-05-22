#include "matrix_keypad.h"

/**********************
 * INTERNAL FUNCTIONS *
 **********************/

int8_t get_gpio_port_bit(GPIO_TypeDef* gpio_port)
{
	if (gpio_port == GPIOA) return 0;
	if (gpio_port == GPIOB) return 1;
	if (gpio_port == GPIOC) return 2;
	if (gpio_port == GPIOE) return 3;
	if (gpio_port == GPIOH) return 7;
	return -1;
}

// when selecting source for interrupts in SYSCFG_EXTICR...) each
// port has a specific value
int8_t get_interrupt_port_value(GPIO_TypeDef* gpio_port)
{
	if (gpio_port == GPIOA) return 0b0000;
	if (gpio_port == GPIOB) return 0b0001;
	if (gpio_port == GPIOC) return 0b0010;
	if (gpio_port == GPIOD) return 0b0011;
	if (gpio_port == GPIOD) return 0b1000;
	if (gpio_port == GPIOH) return 0b0111;
	return -1;
}

static void gpio_setup_clocks(keypad* _keypad)
{
	// enable row clocks
	for (uint8_t i = 0; i < ROW_PINS; ++i)
		RCC->AHB1ENR |= (1 << get_gpio_port_bit(_keypad->row_ports[i]));

	// enable column clocks
	for (uint8_t i = 0; i < COLUMN_PINS; ++i)
		RCC->AHB1ENR |= (1 << get_gpio_port_bit(_keypad->column_ports[i]));
	
	// enable syscfg clock (used for interrupts)
	// note that some MCUs don't require this (e.g. STM32WB55xx). please refer to your datasheet.
	RCC->APB2ENR |= (1 << 14);
}

static void gpio_setup_rows(keypad* _keypad)
{
	// set row pins as output
	for (uint8_t i = 0; i < ROW_PINS; ++i)
	{
		// GPIOx_MODER are 2-bit registers, so we multiply
		// the pin value by 2 when we shift
		_keypad->row_ports[i]->MODER &= ~(0b11 << (2 * _keypad->row_pins[i])); // clear bits
		_keypad->row_ports[i]->MODER |= (0b01 << (2 * _keypad->row_pins[i])); // 0b01 is output mode
	}
}

static void gpio_setup_columns(keypad* _keypad)
{
	// set column pins as input
	for (uint8_t i = 0; i < COLUMN_PINS; ++i)
	{
		// GPIOx_MODER are 2-bit registers, so we multiply
		// the pin value by 2 when we shift
		_keypad->column_ports[i]->MODER &= ~(0b11 << (2 * _keypad->column_pins[i])); // clear bits but also input mode is 0b00 anyways
	}

	// enable internal pullup resistors (want a default signal of HIGH)
	// if you want to use external resistors, you can comment this portion out
	for (uint8_t i = 0; i < COLUMN_PINS; ++i)
	{
		// like MODER, these are also 2-bit registers so we multiply pin value by 2
		_keypad->column_ports[i]->PUPDR &= ~(0b11 << (2 * _keypad->column_pins[i])); // clear bits
		_keypad->column_ports[i]->PUPDR |= (0b01 << (2 * _keypad->column_pins[i])); // 0b01 is pullup mode
	}
}

static void enable_interrupts(keypad* _keypad)
{
	uint8_t register_index;
	for (uint8_t i = 0; i < COLUMN_PINS; ++i)
	{

		// select interrupt source in SYSCFG
	
		// note, these are 4-bit registers so we multiply
		// the pin value by 4 when we shift. SYSCFG_EXTICR... is
		// also broken up into an array with [4] elements
		// representing line 0 to line 15.

		// each SYSCFG_EXTICR register is broken into 4 lines each
		register_index = _keypad->column_pins[i] / 4;
		SYSCFG->EXTICR[register_index] &= ~(0b1111 << (4 * _keypad->column_pins[i])); // clear bits
		SYSCFG->EXTICR[register_index] |= (get_interrupt_port_value(_keypad->column_ports[i]) << (4 * _keypad->column_pins[i]));

		// disable masking
		EXTI->IMR |= (1 << _keypad->column_pins[i]);		

		// enable falling trigger
		EXTI->FTSR |= (1 << _keypad->column_pins[i]);
	}
}

static void pseudo_delay()
{
	for (uint32_t i = 0; i < 20000; ++i);
}

/**********************
 * API FUNCTIONS      *
 **********************/
void keypad_init(
		keypad* 		_keypad,
		GPIO_TypeDef* 	row_ports[ROW_PINS],
		uint8_t 		row_pins[ROW_PINS],
		GPIO_TypeDef*	column_ports[COLUMN_PINS],
		uint8_t			column_pins[COLUMN_PINS])
{

	// copy passed data arrays into pointer arrays
	for (uint8_t i = 0; i < ROW_PINS; ++i)
	{
		_keypad->row_ports[i] = row_ports[i];
		_keypad->row_pins[i] = row_pins[i];
	}

	for (uint8_t i = 0; i < COLUMN_PINS; ++i)
	{
		_keypad->column_ports[i] = column_ports[i];
		_keypad->column_pins[i] = column_pins[i];
	}

	gpio_setup_clocks(_keypad);
	gpio_setup_rows(_keypad);
	gpio_setup_columns(_keypad);
	enable_interrupts(_keypad);
}

void keypad_listen(
		keypad* _keypad,
		bool active)
{
	uint8_t active_row = 0;

	while (active)
	{
		// recall that an active row is actually the ONLY row with a LOW signal
		// all other rows should be set HIGH
		for (uint8_t i = 0; i < ROW_PINS; ++i)
		{
			if (i == active_row)
				_keypad->row_ports[i]->ODR &= ~(1 << _keypad->row_pins[i]);
			else
				_keypad->row_ports[i]->ODR |= (1 << _keypad->row_pins[i]);
		}
		pseudo_delay();
		active_row = (active_row + 1) % ROW_PINS;
	}
}

bool keypad_row_pressed(
		keypad* _keypad,
		uint8_t row)
{
	// recall that an 'active' row is the only one with LOW signal
	// so we check NOT 1
	// also, row is passed ONE-INDEXED (starts at 1 instead of 0)
	return ((_keypad->row_ports[row - 1]->ODR >> _keypad->row_pins[row - 1]) & 0x01) == false;
}
