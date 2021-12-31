/*
 * main.c
 *
 * Created: 12/28/2021 12:36:44 PM
 *  Author: Seba

 * Optimized for 1 sec loop. Loop first checks temperature value, then triggers 
 * conversion. If loop time reduced, then DS18B20 may have not sufficient time for conversion. It sucks... conversion shall be 1 sec, checked every minute
 */ 

#define F_CPU 1000000U
/* debug session flag */
//#define S_DEBUG
#include <xc.h>


#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include <ds18b20/ds18b20.h> 

typedef enum
{
	MINUS = 0,
	PLUS,
}Sign_enum;

#define ALIVE_LOOP_PERIOD		1000
#define CHECK_LOOP_COUNT		(uint8_t) 60
#define SET_TEMPERATURE			(int8_t)5
#define HISTERESIS				(int8_t)1

#define Std_RetVal		uint8_t
#define E_OK			(uint8_t) 0U
#define E_NOT_OK		(uint8_t) 1U

#define ONEWIRE_PIN		PORTB0
#define PIN_RELAY		PORTD6
#define PIN_ALIVE		PORTB1

#ifdef S_DEBUG
#define PIN_DEBUG_VAL   PORTB4
#define PIN_DEBUG_PWM	PORTB3

#define DEBUG_1_LEN		3		/* length of 1 in debug print, in ms */
#define DEBUG_0_LEN		1		/* length of 0 in debug print, in ms */
#endif

#define SetPin(port, pin)		port |= (1 << pin)
#define ClearPin(port, pin)		port &= ~(1 << pin)
#define TogglePin(port, pin)	port ^= (1 << pin);

#define SetAlivePin()		SetPin(PORTB, PIN_ALIVE)
#define ClearAlivePin()		ClearPin(PORTB, PIN_ALIVE)
#define ToggleAlivePin()	TogglePin(PORTB, PIN_ALIVE)

#define SetRelayPin()	SetPin(PORTD, PIN_RELAY)
#define ClearRelayPin()	ClearPin(PORTD, PIN_RELAY)

#ifdef S_DEBUG
#define SetDebugPin()	SetPin(PORTB, PIN_DEBUG_VAL)
#define ClearDebugPin()	ClearPin(PORTB, PIN_DEBUG_VAL)
#endif

typedef struct
{
	int8_t intPart;	/* Integer */
	uint8_t fractPart;	/* Fractional part */
}floatOnInt8_Str;

typedef enum
{
	State_Undefined = 0,
	State_OFF,
	State_ON,
}States_enum;

floatOnInt8_Str Set_Temp={SET_TEMPERATURE, 0U};

#ifdef S_DEBUG
void print_u08(uint8_t val)
{
	for (uint8_t i = 8; i > 0; i--)
	{
		ClearDebugPin();
		_delay_ms(DEBUG_0_LEN);
		SetDebugPin();
		if (val & (1 << (i - 1)))
		{
			_delay_ms(DEBUG_1_LEN);
		}
		else
		{
			_delay_ms(DEBUG_0_LEN);
			ClearDebugPin();
			_delay_ms(DEBUG_1_LEN - DEBUG_0_LEN);
		}
	}
	ClearDebugPin();
	_delay_ms(DEBUG_1_LEN);
}
#endif

void init()
{
	/* Set output pins */
	DDRB |= (1 << PIN_ALIVE);
	DDRD |= (1 << PIN_RELAY);

	#ifdef S_DEBUG
	DDRB |= (1 << PIN_DEBUG_VAL) | (1 << PIN_DEBUG_PWM);
	/* Setup debug PWM on PB4 */
	TCCR1A |= (1 << COM1A1) | (1 << WGM10) | (1 << WGM11);
	TCCR1B |= (1 << CS12);
	// OCR0A = 0x81;
	#endif

	SetAlivePin();
}

void SetRelay(floatOnInt8_Str *Temp, floatOnInt8_Str *Setting)
{
	static States_enum Last_State_u08 = State_Undefined;

	#ifdef S_DEBUG
	print_u08(Temp->intPart);
	#endif
	
	#ifdef S_DEBUG
	print_u08(Setting->intPart);
	#endif

	switch (Last_State_u08)
	{
		case State_OFF:
		{
			if (Temp->intPart < (int8_t)(Setting->intPart - HISTERESIS))
			{
				SetRelayPin();
				Last_State_u08 = State_ON;
			}
			break;
		}
		case State_ON:
		{
			if (Temp->intPart > (int8_t)(Setting->intPart + HISTERESIS))
			{
				ClearRelayPin();
				Last_State_u08 = State_OFF;
			}
			break;
		}
		default:
		{
			if (Temp->intPart > Setting->intPart)
			{
				ClearRelayPin();
				Last_State_u08 = State_OFF;
			}
			else
			{
				SetRelayPin();
				Last_State_u08 = State_ON;
			}
		}
	}
}

Std_RetVal Read_Temp(int16_t *temp)
{	
	/* Trigger next conversion (without ROM matching) */
	ds18b20convert( &PORTB, &DDRB, &PINB, (1 << ONEWIRE_PIN), NULL );

	/* Delay (sensor needs time to perform conversion) */
	_delay_ms( 1000 );

	/* Read temperature (without ROM matching) */
	ds18b20read( &PORTB, &DDRB, &PINB, (1 << ONEWIRE_PIN), NULL, temp);
	
	#ifdef S_DEBUG
	//print_u08(OCR1A);
	print_u08((*temp >> 4U) & 0xFF);
	print_u08((*temp) & 0x0F);
	#endif

	return E_OK;
}

void ConvertToStruct(int16_t temp, floatOnInt8_Str *temp_str)
{
	temp_str->intPart = (int8_t)(temp >> 4U) & 0xFF;
	temp_str->fractPart = temp & 0x0F;
}


Std_RetVal Operate()
{
	int16_t Temp_s16;
	floatOnInt8_Str Temp_str;
	
	/* Read temperature from sensor */
	Read_Temp(&Temp_s16);

	/* Convert read temp to simple structure */
	ConvertToStruct(Temp_s16, &Temp_str);

	/* Set relay according to temperature */
	SetRelay(&Temp_str, &Set_Temp);

	return E_OK;
}

int main(void)
{
	uint8_t LoopCounter_u08 = CHECK_LOOP_COUNT;
	
	init();

    while(1)
    {
	  
		ToggleAlivePin();

		if (LoopCounter_u08 < CHECK_LOOP_COUNT)
		{
			LoopCounter_u08++;
		}
		else
		{
			if (E_OK == Operate())
			{
				LoopCounter_u08 = 1U;
			}
		}

		_delay_ms(ALIVE_LOOP_PERIOD);
	}
}