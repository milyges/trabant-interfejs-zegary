#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#define TACHOOUT_DDR     DDRC
#define TACHOOUT_PORT    PORTC
#define TACHOOUT_PINNO   PC4

#define SPEEDOUT_DDR     DDRC
#define SPEEDOUT_PORT    PORTC
#define SPEEDOUT_PINNO   PC5

#define TEMPOUT_DDR      DDRB
#define TEMPOUT_PINNO    PB2
#define TEMPOUT_OCR      OCR1B

#define FUELOUT_DDR      DDRB
#define FUELOUT_PINNO    PB1
#define FUELOUT_OCR      OCR1A

#define OILOUT_DDR       DDRD
#define OILOUT_PORT      PORTD
#define OILOUT_PINNO     PD6

#define SPEEDIN_DDR      DDRD
#define SPEEDIN_PORT     PORTD
#define SPEEDIN_PINNO    PD2

#define TACHOIN_DDR      DDRD
#define TACHOIN_PORT     PORTD
#define TACHOIN_PINNO    PD3

#define FUELIN_DDR       DDRC
#define FUELIN_PINNO     PC1
#define FUELIN_ADC       1

#define TEMPIN_DDR       DDRC
#define TEMPIN_PINNO     PC0
#define TEMPIN_ADC       0

#define OILIN_DDR        DDRC
#define OILIN_PORT       PORTC
#define OILIN_PIN        PINC
#define OILIN_PINNO      PC3

#define OILIN_READ()     ((OILIN_PIN & (1 << OILIN_PINNO)) == (1 << OILIN_PINNO))
#define OILOUT_WRITE(x)  OILOUT_PORT = (x) ? (OILOUT_PORT | (1 << OILOUT_PINNO)) : (OILOUT_PORT & ~(1 << OILOUT_PINNO))

#define USART_BAUDRATE  9600
#define USART_UBR       (F_CPU / 16 / USART_BAUDRATE - 1)

static uint16_t _rpm_val = 0xFFFF;
static uint16_t _speed_val = 0xFFFF;
static volatile uint16_t _int0_imp;
static volatile uint16_t _int1_imp;
static volatile uint16_t _rpm_input;
static volatile uint8_t _speed_input;
static volatile uint8_t _temp_input;
static volatile uint8_t _fuel_input;

/* Przerwanie Timer0 - generowanie przebiegów dla prędkościomierza i obrotomierza */
ISR(TIMER0_OVF_vect) {
	static uint16_t rpm_cnt = 0;
	static uint16_t speed_cnt = 0;
	
	if (!rpm_cnt) {
		if (_rpm_val != 0xFFFF) {
			TACHOOUT_PORT ^= (1 << TACHOOUT_PINNO);
		}
		rpm_cnt = _rpm_val;
	}
		
	if (!speed_cnt) {
		if (_speed_val != 0xFFFF) {
			SPEEDOUT_PORT ^= (1 << SPEEDOUT_PINNO);
		}
		speed_cnt = _speed_val;
	}
	
	rpm_cnt--; speed_cnt--;
	
	TCNT0 = 248; // 256 - 2^3
}

/*
 * Dla 1000 -> 18
 * Dla 2000 -> 2*18
 * 
 * 
 * */
/* Timer 2 do zliczania impulsów w czasie, przepiełnia z f = 8MHz / 1024 / 256 = 30.52Hz */
ISR(TIMER2_OVF_vect) {
	static uint8_t int0_counter = 0;
        static uint8_t int1_counter = 0;
	uint32_t int0, int1;

	if (int1_counter >= 4) { /* To nadal za dużo - zmniejszamy aby przepełniał się co 30.5 / 4 = 6.1Hz */		
		int1 = _int1_imp; _int1_imp = 0;
		
                _rpm_input = int1 * 1000 / 18;
		int1_counter = 0;
	}
	else {
		int1_counter++;
	}
	
	if (int0_counter >= 4) {
		int0 = _int0_imp; _int0_imp = 0;
		
		_speed_input = int0;
		int0_counter = 0;
	}
	else {
		int0_counter++;
	}
}

/* Przerwanie od impulsów z prędkościomierza */
ISR(INT0_vect) {
	_int0_imp++;
}

/* Przerwanie od impulsów z obrotomierza */
ISR(INT1_vect) {
	_int1_imp++;	
}

ISR(ADC_vect) {
	uint8_t chan = ADMUX & 0x0F;
	uint8_t i, tmp;
	/* Temperatura / 10 => Wartość ADC począwszy od 40C */
	static uint8_t temp2adc[] = {
		254, /* 40C */
		204, /* 50C */
		130, /* 60C */
		89,  /* 70C */
		64,  /* 80C */
		48,  /* 90C */
		37,  /* 100C */
		29,  /* 110C */
		23,  /* 120C */
		19,  /* 130C */
		16,  /* 140C */
	};
	
	/* Sprawdzamy który kanal byl probkowany */
	if (chan == TEMPIN_ADC) {
		tmp = ADCH;
		for(i = 0; i < sizeof(temp2adc) / sizeof(uint8_t); i++) {
			if (temp2adc[i] <= tmp) {
				break;
			}
		}
		
		/* Błąd */
		if (i == 0) { /* Silnik bardzo zimny / błąd */
			_temp_input = 0;
		}
		if (i >= sizeof(temp2adc) / sizeof(uint8_t)) { /* Silnik bardzo gorący / błąd */
			_temp_input = 0xFF;
		}
		else {
			uint8_t a, b;
			a = tmp - temp2adc[i];
			b = temp2adc[i - 1] - tmp;
			_temp_input = 
			_temp_input = (((i * b) + ((i - 1) * a)) * 10) / (a + b) + 40;
		}
		
		chan = FUELIN_ADC;
	}
	else if (chan == FUELIN_ADC) {
		_fuel_input = ADCH;
		chan = TEMPIN_DDR;
	}
	else {
		/* Nigdy nie powinnismy tu trafic */
	}
	
	/* Rozpoczynamy kolejne probkowanie */
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << ADLAR) | chan;
	ADCSRA |= (1 << ADSC);
}

static int printf_helper(char c, FILE * stream) {
	if (c == '\n') 	{
		_delay_ms(1);
		while(!(UCSRA & (1 << UDRE)));
		UDR = '\r';
	}
	_delay_ms(1);
	while(!(UCSRA & (1 << UDRE)));
	UDR = c;

	return 0;
}

static FILE _stdout = FDEV_SETUP_STREAM(printf_helper, NULL, _FDEV_SETUP_WRITE);

void adc_start(uint8_t chan) {
	
}

void init(void) {
	/* Debugowanie po USART */
	UBRRH = (unsigned char)(USART_UBR >> 8);
	UBRRL = (unsigned char)USART_UBR;
	UCSRB = (1 << TXEN) | (1 << RXEN);
	UCSRC = (1 << URSEL) | (3 << UCSZ0);
	stdout = &_stdout;
	
	/* Wyjścia prędkościomierza i obrotomierza */
	TACHOOUT_DDR |= (1 << TACHOOUT_PINNO);
	TACHOOUT_PORT &= ~(1 << TACHOOUT_PINNO);
	
	SPEEDOUT_DDR |= (1 << SPEEDOUT_PINNO);
	SPEEDOUT_PORT &= ~(1 << SPEEDOUT_PINNO);
	
	/* PWM dla temperatury i poziomu paliwa */
	TEMPOUT_DDR |= (1 << TEMPOUT_PINNO);
	FUELOUT_DDR |= (1 << FUELOUT_PINNO);	
	TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11) | (1 << WGM10);
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); 
	
	/* Lampka kontrolna oleju */
	OILOUT_DDR |= (1 << OILOUT_PINNO);
	
	/* Wejście czujnika ciśnienia oleju */
	OILIN_DDR &= ~(1 << OILIN_PINNO);
	OILIN_PORT |= (1 << OILIN_PINNO);
	
	/* Przerwania wejściowe dla impulsów prędkościomierza i obrotomierza */
	SPEEDIN_DDR &= ~(1 << SPEEDIN_PINNO);
	SPEEDIN_PORT |=  (1 << SPEEDIN_PINNO);
	TACHOIN_DDR &= ~(1 << TACHOIN_PINNO);
	TACHOIN_PORT |=  (1 << TACHOIN_PINNO);
	MCUCR |= (1 << ISC11) | (1 << ISC10) | (1 << ISC01) | (1 << ISC00); /* Aktywacja zboczem narastającym */
	GICR |= (1 << INT1) | (1 << INT0);
	GIFR |= (1 << INTF1) | (1 << INTF0); /* Asekuracyjnie zerujemy flagi przerwań */
	
	/* Wejście czujnika paliwa i temperatury */
	TEMPIN_DDR &= ~(1 << TEMPIN_PINNO);
	FUELIN_DDR &= ~(1 << FUELIN_PINNO);
	
	/* Inicjujemy ADC i rozpoczynamy pierwsza konwersje */
	ADMUX = (1 << REFS1) | (1 << REFS0) | (1 << ADLAR) | TEMPIN_ADC;
	ADCSRA = (1 << ADEN) | (1 << ADIF) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
		
	/* Timer 0 - generowanie przebiegów dla prędkościomierza i obrotomierza */
	TCCR0 |= (0 << CS02) | (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
	TIFR |= (1 << TOV0);
	
	/* Timer 2 - odm,ierzanie czasu  */
	TCCR2 |= (1 << CS22) | (1 << CS21) | (1 << CS20);
	TIMSK |= (1 << TOIE2);
	TIFR |= (1 << TOV2);
	
	sei(); /* Globalna aktywacja przerwań */
	
	ADCSRA |= (1 << ADSC); /* uruchamiamy pierwsza konwersje na ADC */
}

void setout_rpm(uint16_t rpm) {
	if (!rpm) {
		_rpm_val = 0xFFFF;
	}
	else {
		uint32_t a = (rpm / 250); 
		uint32_t b = (rpm % 250);
		_rpm_val = (uint32_t)(((960 / (a + 1)) * b) + ((960 / a) * (250 - b))) / 250;
	}
}

void setout_speed(uint8_t speed) {
	if (!speed) {
		_speed_val = 0xFFFF;
	}
	else {
		uint32_t a = speed / 5;
		uint32_t b = speed % 5;
		_speed_val = (((1452 / (a + 1) * b) + ((1452 / a) * (5 - b))) / 5);
	}
}

void setout_fuel(uint8_t level) {
	/* Poziom paliwa w % co 25% */
	static uint16_t fuel2val[] = {
		0x1A0, /* 0 */
		0x136, /* 25% */
		0xFE,  /* 50% */
		0xD6,  /* 75% */
		0xB0,  /* 100 */
		0xB0,  /* 100 */
	};

	if (level > 100)
		level = 100;
	
	uint32_t a = level / 25;
	uint32_t b = level % 25;
	FUELOUT_OCR = ((fuel2val[a + 1] * b) + (fuel2val[a] * (25 - b))) / 25;
}

void setout_temp(uint8_t temp) {
	/* Temperatura / 10 => wartość PWM. Minimalna wartość 50C */
	static uint16_t temp2val[] = {
		0x20, /* 50C */
		0x38, /* 60C */
		0x40, /* 70C */
		0x48, /* 80C */
		0x80, /* 90C */
		0x88, /* 100C */
		0x8E, /* 110C */
		0x96, /* 120C */
		0x9E, /* 130C */
		0x9E, /* 130C */
	};
	
	if (temp < 50) {
		temp = 50;
	}
	else if (temp > 130) {
		temp = 130;
	}
		
	temp -= 50;
	uint32_t a = temp / 10;
	uint32_t b = temp % 10;
	TEMPOUT_OCR = ((temp2val[a + 1] * b) + (temp2val[a] * (10 - b))) / 10;
}

int main(void) {
	init();
	
	setout_speed(0);
	setout_fuel(50);
	setout_temp(0);
	setout_rpm(0);
	uint16_t oldrpm = 0;
	uint8_t oldspeed = 0;
	
	
	while(1) {
		if (oldrpm != _rpm_input) {
			oldrpm = _rpm_input;
			setout_rpm(oldrpm);
		}
		
		if (oldspeed != _speed_input) {
			oldspeed = _speed_input;
			setout_speed(_speed_input);
		}
		
		/* Czujnik cisnienia oleju odczytujemy tylko jak silnik pracuje - wejście negujemy */
		if (_rpm_input > 1450) {
			OILOUT_WRITE(OILIN_READ());
			//OILOUT_WRITE(1);
		}
		else {
			OILOUT_WRITE(0);
		}
		
		if ((_temp_input == 0) || (_temp_input == 0xFF)) {
			setout_temp(0);
		}
		else {
			setout_temp(_temp_input);
		}
		printf("rpm=%u,speed=%u,temp=%u,fuel=%u,oil=%u\n", _rpm_input, _speed_input, _temp_input, _fuel_input, OILIN_READ());
	}
	return 0;
}
