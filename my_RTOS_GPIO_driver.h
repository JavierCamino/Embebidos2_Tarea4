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
#ifndef MY_RTOS_GPIO_DRIVER_H_
#define MY_RTOS_GPIO_DRIVER_H_


#include "MK64F12.h"
#include "stdint.h"
#include "Bits.h"



////////////////////////////////Clock Gating////////////////////////////////////////////////////////////////
/** Constant that represent the clock enable for GPIO A */
#define GPIO_CLOCK_GATING_PORTA 0x00000200
/** Constant that represent the clock enable for GPIO B */
#define GPIO_CLOCK_GATING_PORTB 0x00000400
/** Constant that represent the clock enable for GPIO C */
#define GPIO_CLOCK_GATING_PORTC 0x00000800
/** Constant that represent the clock enable for GPIO D  */
#define GPIO_CLOCK_GATING_PORTD 0x00001000
/** Constant that represent the clock enable for GPIO E */
#define GPIO_CLOCK_GATING_PORTE 0x00002000
////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////////////PCR/////////////////////////////////////////////////////////////////////////////////
/** Interrupt Status Flag pin
 * 0 = Disabled
 * 1 = Enabled */
#define GPIO_PCR_ISF	0x01000000

/** Section of PCR corresponding to PCR_IRQ */
#define GPIO_PCR_IRQ	0x000F0000
/** Interrupt Configuration 0000
 * Interrupt/DMA request disabled. */
#define GPIO_PCR_IRQC0	0x00000000
/** Interrupt Configuration 0001
 * DMA request on rising edge. */
#define GPIO_PCR_IRQC1	0x00010000
/** Interrupt Configuration 0010
 * DMA request on falling edge. */
#define GPIO_PCR_IRQC2	0x00020000
/** Interrupt Configuration 0011
 * DMA request on either edge. */
#define GPIO_PCR_IRQC3	0x00030000
/** Interrupt Configuration 1000
 * Interrupt when logic 0. */
#define GPIO_PCR_IRQC8	0x00080000
/** Interrupt Configuration 1001
 * Interrupt on rising edge. */
#define GPIO_PCR_IRQC9	0x00090000
/** Interrupt Configuration 1010
 * Interrupt on falling edge. */
#define GPIO_PCR_IRQC10	0x000A0000
/** Interrupt Configuration 1011
 * Interrupt on either edge. */
#define GPIO_PCR_IRQC11	0x000B0000
/** Interrupt Configuration 1100
 * Interrupt when logic 1. */
#define GPIO_PCR_IRQC12	0x000C0000

/** Lock Register
 * 0 PCR[15:0] are not locked
 * 1 PCR[15:0] are locked and cannot be updated until the next system reset. */
#define GPIO_PCR_LK		0x00008000

/** Selects alternative function 0
 * Pin disabled (analog). */
#define GPIO_PCR_MUX0  	0x00000000
/** Selects alternative function 1
 * GPIO. */
#define GPIO_PCR_MUX1  	0x00000100
/** Selects alternative function 2
 * Chip-specific. */
#define GPIO_PCR_MUX2  	0x00000200
/** Selects alternative function 3
 * Chip-specific. */
#define GPIO_PCR_MUX3  	0x00000300
/** Selects alternative function 4
 * Chip-specific. */
#define GPIO_PCR_MUX4  	0x00000400
/** Selects alternative function 5
 * Chip-specific. */
#define GPIO_PCR_MUX5  	0x00000500
/** Selects alternative function 6
 * Chip-specific. */
#define GPIO_PCR_MUX6  	0x00000600
/** Selects alternative function 7
 * Chip-specific. */
#define GPIO_PCR_MUX7  	0x00000700

/** Drive Strength Enable
 * 0 Low drive strength is configured on the corresponding pin, if pin is configured as a digital output.
 * 1 High drive strength is configured on the corresponding pin, if pin is configured as a digital output. */
#define GPIO_PCR_DSE   	0x00000040

/** Open Drain Enable
 * 0 Open drain output is disabled on the corresponding pin.
 * 1 Open drain output is enabled on the corresponding pin, if the pin is configured as a digital output.  */
#define GPIO_PCR_ODE   	0x00000020

/** Passive Filter Enable
 * 0 Passive input filter is disabled on the corresponding pin.
 * 1 Passive input filter is enabled on the corresponding pin, if the pin is configured as a digital input. Refer
to the device data sheet for filter characteristics. */
#define GPIO_PCR_PFE   	0x00000010

/** Slew Rate Enable
 * 0 Fast slew rate is configured on the corresponding pin, if the pin is configured as a digital output.
 * 1 Slow slew rate is configured on the corresponding pin, if the pin is configured as a digital output. */
#define GPIO_PCR_SRE   	0x00000004

/** Pull Enable
 * 0 Internal pullup or pulldown resistor is not enabled on the corresponding pin.
 * 1 Internal pullup or pulldown resistor is enabled on the corresponding pin, if the pin is configured as a
digital input. */
#define GPIO_PCR_PE    	0x00000002

/** Pull Select
 * 0 Internal pulldown resistor is enabled on the corresponding pin, if the corresponding PE field is set.
 * 1 Internal pullup resistor is enabled on the corresponding pin, if the corresponding PE field is set. */
#define GPIO_PCR_PS   	0x00000001
////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*! This definition is used to configure whether a pin is an input or an output*/
typedef enum
{
	GPIO_INPUT  = 0,/*!< Definition to configure a pin as input */
	GPIO_OUTPUT = 1 /*!< Definition to configure a pin as output */
} gpio_port_direction_t;

/*! These constants are used to select an specific port in the different API functions*/
typedef enum
{
	GPIO_A, /*!< Definition to select GPIO A */
	GPIO_B, /*!< Definition to select GPIO B */
	GPIO_C, /*!< Definition to select GPIO C */
	GPIO_D, /*!< Definition to select GPIO D */
	GPIO_E, /*!< Definition to select GPIO E */
	GPIO_F  /*!< Definition to select GPIO F */
} gpio_port_name_t;

/*! This data type is used to configure the pin control register*/
typedef const uint32_t gpio_pin_control_register_t;

/*! This data type helps to point to a specific pin in a Port*/
typedef struct
{
	gpio_port_name_t port;
	bit_t pin;
} PortPin_t;

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables the GPIO clock by configuring the corresponding bit
 	 	 and register in the System Clock Gating Control Register.
 	 \param[in]  port_name Port to be configured.
 */
void GPIO_clock_gating(gpio_port_name_t port_name);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function configure different characteristics in an specific GPIO:
 	 	 pullup or pulldown resistor,slew rate, drive strength, passive filter,open drain pin,alternative functions in the GPIO
 	 \param[in] port_name Port to be configured.
 	 \param[in]  pin Specific pin to be configured.
 	 \param[in]  pinControlRegister Pointer to a constant configuration value that configures the pin characteristics. In particular this function
 	 uses the definitions GPIO_PS, GPIO_PE, GPIO_MUX1 etc. For example, in order to configure the pullup resistor ans the pin as GPIO it is need to
 	 declare the type in following way:
 	 gpio_pin_control_register_t PinControlRegister = GPIO_MUX1|GPIO_PS|GPIO_PE;
 */
void GPIO_set_pin_control_register(gpio_port_name_t port_name, bit_t pin, gpio_pin_control_register_t* pinControlRegister);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function modifies the contents of one section of a PCR register.
 	 \param[in]  port_name Port in which the pin is found.
 	 \param[in]  pin Specific pin to edit configuration.
 	 \param[in]  section the part od a 32 bit reg to be modified.
 	 	 	 	 It is marked as a block of adyacent ones over an empty register.
 	 	 	 	 For example: 0000_0000_0000_1111_0000_0000_0000_0000
 	 	 	 	 This represents that the desired section to edit are bits 16-19.
 	 \param[in]  data This is the information to be writen in the section.
 	 	 	 	 The function expects data to be an unsigned number that can fit in the amount
 	 	 	 	 of bits that make up section. This value must not be shifted, but rather referenced
 	 	 	 	 by zero. For example, for the section marked in the previous example, data could be
 	 	 	 	 any unsigned integer in the range [0,15].
 */
void GPIO_edit_pin_control_register(gpio_port_name_t port_name, bit_t pin, uint32_t section, uint32_t data);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief
 	 	 This function returns the content of PCR for the specified pin PORT.
 	 \param[in]  port_name Port in which the pin is found.
 	 \param[in]  pin Specific pin to be requested information.
 	 \return value of PORTX->PCR[pin].
 */
uint32_t GPIO_get_pin_control_register(gpio_port_name_t port_name, bit_t pin);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This function configure all the GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] port_name Port to configure
 	 \param[in] direction Input value to specify the port as input or output.
 */
void GPIO_data_direction_port(gpio_port_name_t port_name, gpio_port_direction_t direction);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief  This function configure specific pins of a GPIO port as input when 1 logic is written or output when 0 logic is written.
 	 \param[in] port_name Port to configure.
 	 \param[in] state Value to specify if the pin behaves as input or output.
 	 \param[in] pin Input value to specify the pin number.
 */
void GPIO_data_direction_pin(gpio_port_name_t port_name, bit_t pin, gpio_port_direction_t state);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads all the GPIO port.
 	 \param[in] port_name Port to be read.
 	 \return  It is the value read from a GPIO. It is a 32-bit value.
 */
uint32_t GPIO_read_port(gpio_port_name_t port_name);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function reads a specific GPIO pin.
	 \param[in] port_name Port to be read.
 	 \param[in] pin Pin to be read.
 	 \return This function return 0 if the value of the pin is 0 logic or 1 is the value the pin is 1 logic.
 */
boolean_t GPIO_read_pin(gpio_port_name_t port_name, bit_t pin);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This function writes all the GPIO port.
 	 \param[in] port_name Port to be written.
 	 \param[in] data Value to be written.
 */
void GPIO_write_port(gpio_port_name_t port_name, uint32_t data);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This function writes the provided data to the specified pin.
 	 	 	It is assumed that the pin has already been initialized and configured as output.
 	 \param[in] port_name Port to be written.
 	 \param[in] pin Pin where data will be written.
 	 \param[in] data Value to be written.
 */
void GPIO_write_pin(gpio_port_name_t port_name, bit_t pin, boolean_t data);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	\brief This set an specific pin in a GPIO port, it uses GPIO_PSOR register.
 	\param[in] port_name Port to be selected.
 	\param[in] pin Pin to be set.
 */
void GPIO_set_pin(gpio_port_name_t port_name, bit_t pin);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This clear an specific pin in a GPIO port, it uses GPIO_PCOR register.
 	 \param[in] port_name Selected Port.
 	 \param[in] pin Pin to be clear.
 */
void GPIO_clear_pin(gpio_port_name_t port_name, bit_t pin);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief This toggle the value of a specific pin in a GPIO port, it uses GPIO_PTOR register.
 	 \param[in] port_name Selected Port.
 	 \param[in] pin Pin to be toggled.
 */
void GPIO_toggle_pin(gpio_port_name_t port_name, bit_t pin);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function enables a Port interruption and sets a priority to it.
 	 \param[in]  port_name Port which interruption is desired to enable.
 	 \param[in]  priority From highest (0) to lowest (15), the priority of the interrupt.
 */
void GPIO_interrupt_enable(gpio_port_name_t port_name, uint32_t priority);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function disables a Port interruption.
 	 \param[in]  port_name Port which interruption is desired to disable.
 */
void GPIO_interrupt_disable(gpio_port_name_t port_name);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function sets the callback for a specific port interrupt.
 	 \param[in]  port_name Port to which handler is to be attached.
 	 \param[in]  handler Callback function to attach to port interrupt.
 */
void GPIO_interrupt_callback(gpio_port_name_t port_name, void (*handler)(void));

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function returns the content of the corresponding IFSR.
 	 \param[in]  port_name Port to find pin.
 */
uint32_t GPIO_get_interrupt_flag(gpio_port_name_t port_name);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function clears all interrupt flags for the specified PORT.
 	 \param[in]  port_name Port to find pin.
 */
void GPIO_clear_interrupt_flag(gpio_port_name_t port_name);

/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
 	 \brief	 This function clears the interrupt flag for the specified pin PORT.
 	 \param[in]  port_name Port to find pin.
 	 \param[in]  pin Pin to clear interrupt flag.
 */
void GPIO_pin_clear_interrupt_flag(gpio_port_name_t port_name, bit_t pin);





#endif /* MY_RTOS_GPIO_DRIVER_H_ */
