/*
 * 
 *
 * 
 * 
 */ 

#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define RX  (1<<4)
#define TX  (1<<3)
#define TE  (1<<5)
#define RE  (1<<7)

int x1 = 150, y1 = 100, z1 = 100,ho = 100; 
int ct1, ct2, rt1, rt2, rtt1 =2.5, rtt2 =2.5, f =0.8, i;  //

void uart0_init()
{
  UCSR0B = 0x00;              //disable while setting baud rate
  UCSR0A = 0x00;
  UCSR0C = 0x06;
  UBRR0L = 0x5F;              //9600BPS at 14745600Hz
  UBRR0H = 0x00;
  UCSR0B = 0x98;
  //UCSR0C = 3<<1;              //setting 8-bit character and 1 stop bit
  //UCSR0B = RX | TX;
}

void adc_init() 
{
  ADCSRA|=((1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
}

char uart_rx()
{
  while(!(UCSR0A & RE));            //waiting to receive
  return UDR0;
}

void uart_tx(char data)
{
  while(!(UCSR0A & TE));            //waiting to transmit
  UDR0 = data;
}

void magnet_pin_config()
{
  DDRH =0xFF;
  DDRA =0xFF;
  PORTH =0x00;
  PORTA =0x00;
}

void motor_pin_config()
{
   DDRH =0xFF;
   PORTH =0x00;
   DDRA = 0xFF;
   DDRB = 0xFF;
   PORTA = 0x00;
}

void magnet_on()
{
  PORTH =0x01 ;
}

void magnet_off()
{
  PORTH =0x00 ;
}

void forward()
{
  PORTA =0b10100000;
}

void backward()
{
  PORTA =0b01010000 ;
}

void left()
{
	ct1=0;
	EIMSK=(1<<INT0);
	stop_();
	_delay_ms(500);
	sei();
	while (rt1 <= rtt1)
	{
		PORTA =0b10010000;
	}
	cli();
  
}

void right()
{
  ct1=0;
  EIMSK=(1<<INT0);
  stop_();
  _delay_ms(100);
  sei();
  while (ct1 <= 50)
  {
	  PORTA =0b10010000;
  }
  cli();
  stop_();
}

void soft_left()
{
  PORTA =0b00010000;
}

void soft_right()
{
  PORTA =0b01000000 ;
}

void stop_()
{
  PORTA =0x00 ;
}

void deg_right()
{
	ct1 = 0;
	stop_();
	_delay_ms(100);
	EIMSK=(1<<INT0);
	sei();
	while (rt1 <= rtt1)
	{
		PORTA =0b01000000 ;
		OCR0A=50;
	}
	cli();
}

void deg_left()
{
	stop_();
	ct2 =0;
	_delay_ms(100);
	EIMSK=(1<<INT1);
	sei();
	while (rt2 <= rtt2)
	{
		PORTA =0b00010000;
		OCR0A=50;
	}
	cli();
}

void beep_on()
{
  PORTA=0x01;
}

void beep_off()
{
	PORTA=0x00;
}

int adc_read()
{
  ADCSRA|=(1<<ADSC);
  while(!(ADCSRA&(1<<ADIF)));
  ADCSRA|=(1<<ADIF);
  return ADCH;
}


int main(void)
{
  motor_pin_config();
  magnet_pin_config();
  adc_init();
  int x,y,z,l,c;  
  char data;
  EICRA|=(1<<ISC11)|(1<<ISC10)|(1<<ISC01)|(1<<ISC00);
  TCCR0A = (1<<COM0A1)|(1<<WGM01)|(1<<WGM00);
  TCCR0B = (1<<CS02) | (1<<CS00);
  TCCR2A = (1<<COM2A1)|(1<<WGM21)|(1<<WGM20);
  TCCR2B = (1<<CS22) | (1<<CS20);
  OCR0A=60;
  OCR2A=100;
  while (1) 
    {
    ADMUX=0b01100000;
    x=adc_read();
    ADMUX=0b01100010;
    y=adc_read();
    ADMUX=0b01100100;
    z=adc_read();
    if (x>=x1 && y<=y1 && z<=z1)
    {
      soft_right();
    }
    else if (x<=x1 & y>=y1 & z<=z1)
    {
      forward();
    }
    else if ((x<=x1) && (y<=y1) && (z>=z1))
    {
      soft_left();
    }	
    else if(((x<=x1) && (y>=y1) && (z>=z1)) || ((x>=x1) && (y>=y1) && (z<=z1))|| ((x>=x1) && (y>=y1) && (z>=z1)))
     {
      uart_tx("a");                      // 'a' is transmitted to indicate the node is reached
	  data =  uart_rx();
      if(data=='r'){right();}            // 'r' from the PC signifies turn right      
      else if (data =='l'){left();}      // 'l' indicates left
      else if (data == 'i')
	   {
		  deg_right();
		  stop_();
		  _delay_ms(50);
		  sei();
		  ct1 = 0;
		  while (rt1 <= f)
		  {
			  forward();
		  }
		  cli();
		  c++;
		  l=1;
	   }
	  else if (data == 'e')
	   {
		  deg_left();
		  stop_();
		  _delay_ms(50);
		  sei();
		  ct1 = 0;
		  while (rt1 <= f)
		  {
			  forward();
		  }
		  cli();	
		  c++;
		  l=1;
	    }
		else if (data == 'b')
		{
			beep_on();
			_delay_ms(5000);
			beep_off();
			PORTA=0x00;
			PORTH=0x00;
			while (1)
			{
				uart_tx("c");
			}
		}		
	   if (c%2 == 0 && l==1)
	    {
		   magnet_on();
		   beep_on();
		   _delay_ms(1000);
		   beep_off();
		   l=0;
	    }
	   else if (c%2 == 1 && l==1)
	    {
		    magnet_off();
		    beep_on();
			_delay_ms(1000);
		    beep_off();
			l=0;			
	    }
		
	 }	  
   }	  	      
}
