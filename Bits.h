/*
 * Bits.h
 *
 *  Created on: 01/03/2019
 *      Author: Javi
 */

#ifndef BITS_H_
#define BITS_H_


////////////////////Binary Data Macros///////////////////////////////////////////////////////////////////////
/** Define bits to turn-off.*/
#define BIT_LOW 	0
/** Define bits to turn-on.*/
#define BIT_HIGH 	1
/** This is the definition of the low state of all the bits in a register of 32 bits. */
#define REG32_LOW 	0x00000000
/** This is the definition of the high state of all the bits in a register of 32 bits. */
#define REG32_HIGH 	0xffffffff
/////////////////////////////////////////////////////////////////////////////////////////////////////////////


/** Data type for possible bits in a 32bit register.*/
typedef enum
{
	bit_0  = 0,  /*!< Bit 0 */
	bit_1  = 1,  /*!< Bit 1 */
	bit_2  = 2,  /*!< Bit 2 */
	bit_3  = 3,  /*!< Bit 3 */
	bit_4  = 4,  /*!< Bit 4 */
	bit_5  = 5,  /*!< Bit 5 */
	bit_6  = 6,  /*!< Bit 6 */
	bit_7  = 7,  /*!< Bit 7 */
	bit_8  = 8,  /*!< Bit 8 */
	bit_9  = 9,  /*!< Bit 9 */
	bit_10 = 10, /*!< Bit 10 */
	bit_11 = 11, /*!< Bit 11 */
	bit_12 = 12, /*!< Bit 12 */
	bit_13 = 13, /*!< Bit 13 */
	bit_14 = 14, /*!< Bit 14 */
	bit_15 = 15, /*!< Bit 15 */
	bit_16 = 16, /*!< Bit 16 */
	bit_17 = 17, /*!< Bit 17 */
	bit_18 = 18, /*!< Bit 18 */
	bit_19 = 19, /*!< Bit 19 */
	bit_20 = 20, /*!< Bit 20 */
	bit_21 = 21, /*!< Bit 21 */
	bit_22 = 22, /*!< Bit 22 */
	bit_23 = 23, /*!< Bit 23 */
	bit_24 = 24, /*!< Bit 24 */
	bit_25 = 25, /*!< Bit 25 */
	bit_26 = 26, /*!< Bit 26 */
	bit_27 = 27, /*!< Bit 27 */
	bit_28 = 28, /*!< Bit 28 */
	bit_29 = 29, /*!< Bit 29 */
	bit_30 = 30, /*!< Bit 30 */
	bit_31 = 31 /*!< Bit 31 */
} bit_t;


typedef enum
{
	FALSE = 0,
	TRUE  = 1
} boolean_t;



#endif /* BITS_H_ */
