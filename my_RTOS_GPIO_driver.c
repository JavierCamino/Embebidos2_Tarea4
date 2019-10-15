/**
	\file
	\brief
		This is the source file for the GPIO device driver for Kinetis K64.
		It contains all the implementation for configuration functions and runtime functions.
		i.e., this is the application programming interface (API) for the GPIO peripheral.
	\author J. Luis Pizano Escalante, luispizano@iteso.mx
	\date	18/02/2019
	\todo
	    Interrupts are not implemented in this API implementation.
 */

#include <my_RTOS_GPIO_driver.h>


static void (*gpio_A_callback)(void) = 0;
static void (*gpio_B_callback)(void) = 0;
static void (*gpio_C_callback)(void) = 0;
static void (*gpio_D_callback)(void) = 0;
static void (*gpio_E_callback)(void) = 0;


void GPIO_clock_gating(gpio_port_name_t port_name)
{
	switch(port_name)/** Selecting the GPIO for clock enabling*/
	{
	case GPIO_A: /** GPIO A is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTA; /** Bit 9 of SIM_SCGC5 is  set*/
		break;
	case GPIO_B: /** GPIO B is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTB; /** Bit 10 of SIM_SCGC5 is set*/
		break;
	case GPIO_C: /** GPIO C is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTC; /** Bit 11 of SIM_SCGC5 is set*/
		break;
	case GPIO_D: /** GPIO D is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTD; /** Bit 12 of SIM_SCGC5 is set*/
		break;
	case GPIO_E: /** GPIO E is selected*/
		SIM->SCGC5 |= GPIO_CLOCK_GATING_PORTE; /** Bit 13 of SIM_SCGC5 is set*/
		break;
	default: /**If doesn't exist the option*/
		break;
	}// end switch
	/**Successful configuration*/
}// end function

void GPIO_set_pin_control_register(gpio_port_name_t port_name, bit_t pin, gpio_pin_control_register_t*  pin_control_register)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		PORTA->PCR[pin] = *pin_control_register;
		break;
	case GPIO_B:/** GPIO B is selected*/
		PORTB->PCR[pin] = *pin_control_register;
		break;
	case GPIO_C:/** GPIO C is selected*/
		PORTC->PCR[pin] = *pin_control_register;
		break;
	case GPIO_D:/** GPIO D is selected*/
		PORTD->PCR[pin] = *pin_control_register;
		break;
	case GPIO_E: /** GPIO E is selected*/
		PORTE->PCR[pin] = *pin_control_register;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	/**Successful configuration*/
}

void GPIO_edit_pin_control_register(gpio_port_name_t port_name, bit_t pin, uint32_t section, uint32_t data)
{
	// Store the original configuration of PCR.
	uint32_t original_config = GPIO_get_pin_control_register(port_name, pin);
	// Erase the specified section.
	original_config &= ~section;

	// Variable to store the position of the section.
	uint8_t position = 0;
	// Find the position of the section.
	while(0 == section % 2)
	{
		section = section >> 1;
		position++;
	}

	// Append data to original config in the erased section. Store the result in a gpio_pin_control_register_t variable.
	gpio_pin_control_register_t new_config = original_config | (data << position);

	//Rewrite configuration.
	GPIO_set_pin_control_register(port_name, pin, &new_config);
}

uint32_t GPIO_get_pin_control_register(gpio_port_name_t port_name, bit_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		return PORTA->PCR[pin];
		break;
	case GPIO_B:/** GPIO B is selected*/
		return PORTB->PCR[pin];
		break;
	case GPIO_C:/** GPIO C is selected*/
		return PORTC->PCR[pin];
		break;
	case GPIO_D:/** GPIO D is selected*/
		return PORTD->PCR[pin];
		break;
	case GPIO_E: /** GPIO E is selected*/
		return PORTE->PCR[pin];
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	// The function should never reach this point.
	return REG32_HIGH;
}

void GPIO_data_direction_port(gpio_port_name_t port_name, gpio_port_direction_t direction)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		if(GPIO_INPUT == direction)
			GPIOA->PDDR = REG32_LOW;
		else
			GPIOA->PDDR = REG32_HIGH;
		break;
	case GPIO_B:/** GPIO B is selected*/
		if(GPIO_INPUT == direction)
			GPIOB->PDDR = REG32_LOW;
		else
			GPIOB->PDDR = REG32_HIGH;
		break;
	case GPIO_C:/** GPIO C is selected*/
		if(GPIO_INPUT == direction)
			GPIOC->PDDR = REG32_LOW;
		else
			GPIOC->PDDR = REG32_HIGH;
		break;
	case GPIO_D:/** GPIO D is selected*/
		if(GPIO_INPUT == direction)
			GPIOD->PDDR = REG32_LOW;
		else
			GPIOD->PDDR = REG32_HIGH;
		break;
	case GPIO_E: /** GPIO E is selected*/
		if(GPIO_INPUT == direction)
			GPIOE->PDDR = REG32_LOW;
		else
			GPIOE->PDDR = REG32_HIGH;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_data_direction_pin(gpio_port_name_t port_name, bit_t pin, gpio_port_direction_t state)
{
	uint32_t pin_one_hot = 1 << pin;
	uint32_t pin_one_cold = ~pin_one_hot;

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		if(GPIO_OUTPUT == state)
			GPIOA->PDDR |= pin_one_hot;
		else
			GPIOA->PDDR &= pin_one_cold;
		break;
	case GPIO_B:/** GPIO B is selected*/
		if(GPIO_OUTPUT == state)
			GPIOB->PDDR |= pin_one_hot;
		else
			GPIOB->PDDR &= pin_one_cold;
		break;
	case GPIO_C:/** GPIO C is selected*/
		if(GPIO_OUTPUT == state)
			GPIOC->PDDR |= pin_one_hot;
		else
			GPIOC->PDDR &= pin_one_cold;
		break;
	case GPIO_D:/** GPIO D is selected*/
		if(GPIO_OUTPUT == state)
			GPIOD->PDDR |= pin_one_hot;
		else
			GPIOD->PDDR &= pin_one_cold;
		break;
	case GPIO_E: /** GPIO E is selected*/
		if(GPIO_OUTPUT == state)
			GPIOE->PDDR |= pin_one_hot;
		else
			GPIOE->PDDR &= pin_one_cold;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

uint32_t GPIO_read_port(gpio_port_name_t port_name)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		return GPIOA->PDIR;
		break;
	case GPIO_B:/** GPIO B is selected*/
		return GPIOB->PDIR;
		break;
	case GPIO_C:/** GPIO C is selected*/
		return GPIOC->PDIR;
		break;
	case GPIO_D:/** GPIO D is selected*/
		return GPIOD->PDIR;
		break;
	case GPIO_E: /** GPIO E is selected*/
		return GPIOE->PDIR;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	// The function should never reach this point.
	return REG32_HIGH;
}

boolean_t GPIO_read_pin(gpio_port_name_t port_name, bit_t pin)
{
	uint32_t pin_one_hot = 1 << pin;
	uint32_t port_value;

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		port_value = GPIOA->PDIR;
		if(REG32_LOW == (port_value & pin_one_hot))
			return FALSE;
		else
			return TRUE;
		break;
	case GPIO_B:/** GPIO B is selected*/
		port_value = GPIOB->PDIR;
		if(REG32_LOW == (port_value & pin_one_hot))
			return FALSE;
		else
			return TRUE;
		break;
	case GPIO_C:/** GPIO C is selected*/
		port_value = GPIOC->PDIR;
		if(REG32_LOW == (port_value & pin_one_hot))
			return FALSE;
		else
			return TRUE;
		break;
	case GPIO_D:/** GPIO D is selected*/
		port_value = GPIOD->PDIR;
		if(REG32_LOW == (port_value & pin_one_hot))
			return FALSE;
		else
			return TRUE;
		break;
	case GPIO_E: /** GPIO E is selected*/
		port_value = GPIOE->PDIR;
		if(REG32_LOW == (port_value & pin_one_hot))
			return FALSE;
		else
			return TRUE;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	// The function should never reach this point.
	return TRUE;
}

void GPIO_write_port(gpio_port_name_t port_name, uint32_t data)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PDOR = data;
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PDOR = data;
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PDOR = data;
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PDOR = data;
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PDOR = data;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	/**Successful configuration*/
}

void GPIO_write_pin(gpio_port_name_t port_name, bit_t pin, boolean_t data)
{
	// Check whether the value to be written in a logic 0 or logic 1 and act accordingly.
	switch(data)
	{
	case BIT_LOW:
		GPIO_clear_pin(port_name, pin);
		break;
	case BIT_HIGH:
		GPIO_set_pin(port_name, pin);
		break;
	default:
		break;
	}
}

void GPIO_set_pin(gpio_port_name_t port_name, bit_t pin)
{
	uint32_t pin_one_hot = 1 << pin;
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PSOR |= pin_one_hot;
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PSOR |= pin_one_hot;
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PSOR |= pin_one_hot;
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PSOR |= pin_one_hot;
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PSOR |= pin_one_hot;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_clear_pin(gpio_port_name_t port_name, bit_t pin)
{
	uint32_t pin_one_hot = 1 << pin;

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PCOR |= pin_one_hot;
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PCOR |= pin_one_hot;
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PCOR |= pin_one_hot;
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PCOR |= pin_one_hot;
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PCOR |= pin_one_hot;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_toggle_pin(gpio_port_name_t port_name, bit_t pin)
{
	uint32_t pin_one_hot = 1 << pin;

	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		GPIOA->PTOR |= pin_one_hot;
		break;
	case GPIO_B:/** GPIO B is selected*/
		GPIOB->PTOR |= pin_one_hot;
		break;
	case GPIO_C:/** GPIO C is selected*/
		GPIOC->PTOR |= pin_one_hot;
		break;
	case GPIO_D:/** GPIO D is selected*/
		GPIOD->PTOR |= pin_one_hot;
		break;
	case GPIO_E: /** GPIO E is selected*/
		GPIOE->PTOR |= pin_one_hot;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_interrupt_enable(gpio_port_name_t port_name, uint32_t priority)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		NVIC_EnableIRQ(PORTA_IRQn);
		NVIC_SetPriority(PORTA_IRQn, priority);
		break;
	case GPIO_B:/** GPIO B is selected*/
		NVIC_EnableIRQ(PORTB_IRQn);
		NVIC_SetPriority(PORTB_IRQn, priority);
		break;
	case GPIO_C:/** GPIO C is selected*/
		NVIC_EnableIRQ(PORTC_IRQn);
		NVIC_SetPriority(PORTC_IRQn, priority);
		break;
	case GPIO_D:/** GPIO D is selected*/
		NVIC_EnableIRQ(PORTD_IRQn);
		NVIC_SetPriority(PORTD_IRQn, priority);
		break;
	case GPIO_E: /** GPIO E is selected*/
		NVIC_EnableIRQ(PORTE_IRQn);
		NVIC_SetPriority(PORTE_IRQn, priority);
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_interrupt_disable(gpio_port_name_t port_name)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		NVIC_DisableIRQ(PORTA_IRQn);
		break;
	case GPIO_B:/** GPIO B is selected*/
		NVIC_DisableIRQ(PORTB_IRQn);
		break;
	case GPIO_C:/** GPIO C is selected*/
		NVIC_DisableIRQ(PORTC_IRQn);
		break;
	case GPIO_D:/** GPIO D is selected*/
		NVIC_DisableIRQ(PORTD_IRQn);
		break;
	case GPIO_E: /** GPIO E is selected*/
		NVIC_DisableIRQ(PORTE_IRQn);
		break;
	default:/**If doesn't exist the option*/
		break;
	}
}

void GPIO_interrupt_callback(gpio_port_name_t port_name, void (*handler)(void))
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		gpio_A_callback = handler;
		break;
	case GPIO_B:/** GPIO B is selected*/
		gpio_B_callback = handler;
		break;
	case GPIO_C:/** GPIO C is selected*/
		gpio_C_callback = handler;
		break;
	case GPIO_D:/** GPIO D is selected*/
		gpio_D_callback = handler;
		break;
	case GPIO_E: /** GPIO E is selected*/
		gpio_E_callback = handler;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	/**Successful configuration*/
}

uint32_t GPIO_get_interrupt_flag(gpio_port_name_t port_name)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		return PORTA->ISFR;
		break;
	case GPIO_B:/** GPIO B is selected*/
		return PORTB->ISFR;
		break;
	case GPIO_C:/** GPIO C is selected*/
		return PORTC->ISFR;
		break;
	case GPIO_D:/** GPIO D is selected*/
		return PORTD->ISFR;
		break;
	case GPIO_E: /** GPIO E is selected*/
		return PORTE->ISFR;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	// The function should never reach this point.
	return REG32_HIGH;
}

void GPIO_clear_interrupt_flag(gpio_port_name_t port_name)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		PORTA->ISFR = REG32_HIGH;
		break;
	case GPIO_B:/** GPIO B is selected*/
		PORTB->ISFR = REG32_HIGH;
		break;
	case GPIO_C:/** GPIO C is selected*/
		PORTC->ISFR = REG32_HIGH;
		break;
	case GPIO_D:/** GPIO D is selected*/
		PORTD->ISFR = REG32_HIGH;
		break;
	case GPIO_E: /** GPIO E is selected*/
		PORTE->ISFR = REG32_HIGH;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	/**Successful configuration*/
}

void GPIO_pin_clear_interrupt_flag(gpio_port_name_t port_name, bit_t pin)
{
	switch(port_name)
	{
	case GPIO_A:/** GPIO A is selected*/
		PORTA->PCR[pin] |= GPIO_PCR_ISF;
		break;
	case GPIO_B:/** GPIO B is selected*/
		PORTB->PCR[pin] |= GPIO_PCR_ISF;
		break;
	case GPIO_C:/** GPIO C is selected*/
		PORTC->PCR[pin] |= GPIO_PCR_ISF;
		break;
	case GPIO_D:/** GPIO D is selected*/
		PORTD->PCR[pin] |= GPIO_PCR_ISF;
		break;
	case GPIO_E: /** GPIO E is selected*/
		PORTE->PCR[pin] |= GPIO_PCR_ISF;
		break;
	default:/**If doesn't exist the option*/
		break;
	}
	/**Successful configuration*/
}





void PORTA_IRQHandler(void)
{
	if(gpio_A_callback)
	{
		gpio_A_callback();
	}

	GPIO_clear_interrupt_flag(GPIO_A);
}
void PORTB_IRQHandler(void)
{
	if(gpio_B_callback)
	{
		gpio_B_callback();
	}

	GPIO_clear_interrupt_flag(GPIO_B);
}
void PORTC_IRQHandler(void)
{
	if(gpio_C_callback)
	{
		gpio_C_callback();
	}

	GPIO_clear_interrupt_flag(GPIO_C);
}
void PORTD_IRQHandler(void)
{
	if(gpio_D_callback)
	{
		gpio_D_callback();
	}

	GPIO_clear_interrupt_flag(GPIO_D);
}
void PORTE_IRQHandler(void)
{
	if(gpio_E_callback)
	{
		gpio_E_callback();
	}

	GPIO_clear_interrupt_flag(GPIO_E);
}
