#include <tiny2313.h>
#include <i2c.h>
#include <delay.h>

#define ADR 0xD0

#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE (1<<RXC)
#define FRAMING_ERROR (1<<FE)
#define PARITY_ERROR (1<<UPE)
#define DATA_OVERRUN (1<<DOR)

// USART Transmitter buffer
#define TX_BUFFER_SIZE 8
char tx_buffer[TX_BUFFER_SIZE];

#if TX_BUFFER_SIZE <= 256
unsigned char tx_wr_index=0,tx_rd_index=0;
#else
unsigned int tx_wr_index=0,tx_rd_index=0;
#endif

#if TX_BUFFER_SIZE < 256
unsigned char tx_counter=0;
#else
unsigned int tx_counter=0;
#endif

// USART Transmitter interrupt service routine
interrupt [USART_TXC] void usart_tx_isr(void)
{
if (tx_counter)
   {
   --tx_counter;
   UDR=tx_buffer[tx_rd_index++];
#if TX_BUFFER_SIZE != 256
   if (tx_rd_index == TX_BUFFER_SIZE) tx_rd_index=0;
#endif
   }
}

#ifndef _DEBUG_TERMINAL_IO_
// Write a character to the USART Transmitter buffer
#define _ALTERNATE_PUTCHAR_
#pragma used+
void putchar(char c)
{
while (tx_counter == TX_BUFFER_SIZE);
#asm("cli")
if (tx_counter || ((UCSRA & DATA_REGISTER_EMPTY)==0))
   {
   tx_buffer[tx_wr_index++]=c;
#if TX_BUFFER_SIZE != 256
   if (tx_wr_index == TX_BUFFER_SIZE) tx_wr_index=0;
#endif
   ++tx_counter;
   }
else
   UDR=c;
#asm("sei")
}
#pragma used-
#endif

// Standard Input/Output functions
#include <stdio.h>

void write(unsigned char reg, unsigned char data)
{
     i2c_start(); 
     i2c_write(ADR); 
     i2c_write(reg); 
     i2c_write(data);
     i2c_stop();
}

unsigned char read(unsigned char reg)
{
    unsigned char data;
    i2c_start(); 
    i2c_write(ADR); 
    i2c_write(reg);
    i2c_start();
    i2c_write(ADR|1); 
    data=i2c_read(0);
    i2c_stop();
    return data-127;
}

void main(void)
{
#define N 10

int index = 0;
int oldXes[N], oldYes[N] = {0};

// Crystal Oscillator division factor: 1
#pragma optsize-
CLKPR=(1<<CLKPCE);
CLKPR=(0<<CLKPCE) | (0<<CLKPS3) | (0<<CLKPS2) | (0<<CLKPS1) | (0<<CLKPS0);
#ifdef _OPTIMIZE_SIZE_
#pragma optsize+
#endif

DDRA.0 = 1;
DDRA.1 = 1;
PORTA.0 = 0;
PORTA.1 = 1;

// USART initialization
// Communication Parameters: 8 Data, 1 Stop, No Parity
// USART Receiver: Off
// USART Transmitter: On
// USART Mode: Asynchronous
// USART Baud Rate: 9600
UCSRA=(0<<RXC) | (0<<TXC) | (0<<UDRE) | (0<<FE) | (0<<DOR) | (0<<UPE) | (0<<U2X) | (0<<MPCM);
UCSRB=(0<<RXCIE) | (1<<TXCIE) | (0<<UDRIE) | (0<<RXEN) | (1<<TXEN) | (0<<UCSZ2) | (0<<RXB8) | (0<<TXB8);
UCSRC=(0<<UMSEL) | (0<<UPM1) | (0<<UPM0) | (0<<USBS) | (1<<UCSZ1) | (1<<UCSZ0) | (0<<UCPOL);
UBRRH=0x00;
UBRRL=0x33;

// Timer/Counter 0 initialization
// Clock source: System Clock
// Clock value: 31,250 kHz
// Mode: Fast PWM top=0xFF
// OC0A output: Non-Inverted PWM
// OC0B output: Non-Inverted PWM
// Timer Period: 8,192 ms
// Output Pulse(s):
// OC0A Period: 8,192 ms Width: 0 us
// OC0B Period: 8,192 ms Width: 0 us
TCCR0A=(1<<COM0A1) | (0<<COM0A0) | (1<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (1<<WGM00);
TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (0 <<CS00);
TCNT0=0x00;
OCR0A=0x00;
OCR0B=0x00;

// Timer/Counter 1 initialization
// Clock source: System Clock
// Clock value: 31,250 kHz
// Mode: Fast PWM top=0x00FF
// OC1A output: Non-Inverted PWM
// OC1B output: Non-Inverted PWM
// Noise Canceler: Off
// Input Capture on Falling Edge
// Timer Period: 8,192 ms
// Output Pulse(s):
// OC1A Period: 8,192 ms Width: 0 us
// OC1B Period: 8,192 ms Width: 0 us
// Timer1 Overflow Interrupt: Off
// Input Capture Interrupt: Off
// Compare A Match Interrupt: Off
// Compare B Match Interrupt: Off
TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (1<<COM1B1) | (0<<COM1B0) | (0<<WGM11) | (1<<WGM10);
TCCR1B=(0<<ICNC1) | (0<<ICES1) | (0<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
TCNT1H=0x00;
TCNT1L=0x00;
ICR1H=0x00;
ICR1L=0x00;
OCR1AH=0x00;
OCR1AL=0x00;
OCR1BH=0x00;
OCR1BL=0x00;;

DDRB.2 = 1;
DDRB.3 = 1;
DDRB.4 = 1;
DDRD.5 = 1;

// Bit-Banged I2C Bus initialization
// I2C Port: PORTB
// I2C SDA bit: 0
// I2C SCL bit: 1
// Bit Rate: 100 kHz
// Note: I2C settings are specified in the
// Project|Configure|C Compiler|Libraries|I2C menu.
i2c_init();
#asm("sei")

while (1)
{
    #define FL OCR0B
    #define FR OCR1B
    
    #define RL OCR0A
    #define RR OCR1A
    
    char ACC = 0x3B;
    char GYR = 0x43;
    int X,Y,Z,gX,gY,gZ = 0;
    long totalX = 0, totalY = 0;
    int i = 0;
    write(0x6B,0x00); //Power management 1 (gyro)
    write(0x6C,0xC0); //Power management 2 (frequency 40Hz)
    write(0x1A,0x01); //Filtering config
    write(0x1B,0x00); //Gyro config
    write(0x1C,0x00); //Accelerometer config
    
    X = read(ACC);
    Y = read(ACC+2);
    Z = read(ACC+4);
        
    gX = read(GYR);
    gY = read(GYR+2);
    gZ = read(GYR+4);
      
    oldXes[index]=X;
    oldYes[index]=Y;
    index++;
    if(index>N)
        index=0;
        
    for(i=0;i<N;i++)
    {
        totalX += oldXes[i];   
        totalY += oldYes[i];
    }                      
    X = totalX/N;
    Y = totalY/N;
    
    X-=130;  
    Y-=130;
    
    if(X<0)
    {
        X = 0-X;
    }
    
    if(X>50)
    {    
        X=50;
    }
    
    if(Y<0)
    {
        Y = 0-Y;
    }
    
    if(Y>50)
    {    
        Y=50;
    }
    
    putchar('X');
    putchar(X);
    putchar('Y');
    putchar(Y);
    putchar('Z');
    putchar(Z);
    
    putchar('A');
    putchar(gX);
    putchar('B');
    putchar(gY);
    putchar('C');
    putchar(gZ);
    
    X=0;    
    RR=X;
    FL=X;
    
    putchar('\r');
    delay_ms(3);
}
}