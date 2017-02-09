#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define DURA_NUMBER 1

// LEDS
#define BLUE   2
#define RED    4
#define GREEN  3

typedef enum State
{
    START,
    LEFT,
    RIGHT
}CurrentState;

unsigned char v1;
unsigned char v2;
unsigned char dir1;
unsigned char dir2;
unsigned char robotNumber;
unsigned int ovfCounter;
bool valid = true;
CurrentState state = START;

ISR(TIMER1_OVF_vect)
{
    PORTC = 0;
}

ISR(TIMER2_OVF_vect)
{
    if((++ovfCounter > 15) && bit_is_set(PIND, 6))
    {
        if(state != START)
        {
            PORTD ^= _BV(BLUE);
        }
        state = START;
        robotNumber = 0;
        valid = true;
    }
    if(ovfCounter > 100)
    {
        v1 = 0;
        v2 = 0;
    }
}

ISR(ANA_COMP_vect)
{
    static unsigned char stop;
    static unsigned int pulseWidthL;
    static unsigned int pulseWidthR;
    // if start
    if(bit_is_clear(ACSR, ACO))
    {
        unsigned int validWidth;
        PORTD &= ~_BV(1);
        switch(state)
        {
        case START:
            state = LEFT;
            robotNumber++;
            break;

        case RIGHT:
            state = LEFT;
            robotNumber++;
            validWidth = (unsigned int)TCNT2 + (unsigned int)ovfCounter * 256;
            if((validWidth < 314) || (validWidth > 326))
            {
                valid = false;
                PORTD ^= _BV(GREEN);
            }
            break;

        case LEFT:
            state = RIGHT;
            validWidth = (unsigned int)TCNT2 + (unsigned int)ovfCounter * 256;
            if((validWidth < 314) || (validWidth > 326))
            {
                valid = false;
                PORTD ^= _BV(GREEN);
            }
            break;
        }
        TCNT2 = 0;
        ovfCounter = 0;
        TIFR = _BV(TOV2);
    }
    else
    {
        stop = TCNT2;
        PORTD |= _BV(1);
        if(bit_is_set(TIFR, TOV2) && (stop < 128))
        {
            ovfCounter++;
        }
        if(state == LEFT)
        {
            if((robotNumber == DURA_NUMBER) && (valid == true))
            {
                pulseWidthL = (unsigned int)stop + (unsigned int)ovfCounter * 256;
            }
        }
        else if(state == RIGHT)
        {
            if((robotNumber == DURA_NUMBER) && (valid == true))
            {
                pulseWidthR = (unsigned int)stop + (unsigned int)ovfCounter * 256;
                if((pulseWidthL < 182) && (pulseWidthL > 136))
                {
                    v1 = 0;
                    PORTD |= _BV(GREEN);
                }
                else if(pulseWidthL < 137)
                {
                    // Backward
                    if(pulseWidthL < 91)
                    {
                        v1 = 250;
                    }
                    else if(pulseWidthL < 114)
                    {
                        v1 = 236;
                    }
                    else
                    {
                        v1 = 216;
                    }
                    dir1 = 1;
                    OCR1AH = 0;
                    OCR1AL = 255 - v1;
                    PORTD &= ~_BV(GREEN);
                }
                else
                {
                    // Forward
                    if(pulseWidthL > 227)
                    {
                        v1 = 250;
                    }
                    else if(pulseWidthL > 204)
                    {
                        v1 = 236;
                    }
                    else
                    {
                        v1 = 216;
                    }
                    dir1 = 0;
                    OCR1AH = 0;
                    OCR1AL = 255 - v1;
                    PORTD &= ~_BV(GREEN);
                }
                if((pulseWidthR < 182) && (pulseWidthR > 136))
                {
                    v2 = 0;
                }
                else if(pulseWidthR < 137)
                {
                    if(pulseWidthR < 91)
                    {
                        v2 = 251;
                    }
                    else if(pulseWidthR < 114)
                    {
                        v2 = 241;
                    }
                    else
                    {
                        v2 = 216;
                    }
                    dir2 = 1;
                    OCR1BH = 0;
                    OCR1BL = 255 - v2;
                }
                else
                {
                    if(pulseWidthR > 227)
                    {
                        v2 = 251;
                    }
                    else if(pulseWidthR > 204)
                    {
                        v2 = 241;
                    }
                    else
                    {
                        v2 = 216;
                    }
                    dir2 = 0;
                    OCR1BH = 0;
                    OCR1BL = 255 - v2;
                }
            }
        }
    }
    PORTD ^= _BV(RED);
}

ISR(TIMER1_COMPA_vect)
{
    if(v1 > 100)
    {
        if(dir1 == 0)
        {
            PORTC |= _BV(1);
        }
        else
        {
            PORTC |= _BV(2);
        }
    }
}

ISR(TIMER1_COMPB_vect)
{
    if(v2 > 100)
    {
        if(dir2 == 0)
        {
            PORTC |= _BV(4);
        }
        else
        {
            PORTC |= _BV(3);
        }
    }
}

int main(void)
{
    // LEDs
    DDRD = _BV(GREEN) | _BV(BLUE) | _BV(RED);
    // Motors
    DDRC = _BV(1) | _BV(2) | _BV(3) | _BV(4);

    // For Debug
    DDRD |= _BV(1);

    // Fast PWM TOP - 0xFF, Prescaler = 8
    TCCR1A = _BV(WGM10);
    TCCR1B = _BV(WGM12) | _BV(CS11);
    OCR1AH = 0x00;
    OCR1AL = 255;
    OCR1BH = 0x00;
    OCR1BL = 255;

    TCCR2 = _BV(CS22);
    TIMSK = _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) | _BV(TOIE2);

    //Analog Comparator
    ACSR = _BV(ACIE);

    sei();

    while(1)
    {
    }
}

