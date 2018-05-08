#define TRUE 1
#define FALSE 0
#define SCL_CLOCK 100000L // I2C clock in Hz
#define ADDR_W 0xEF // Module address write mode
#define ADDR_R 0xEF // Module address read mode
#define CMD_RESET 0x1E // ADC reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command
#define CMD_ADC_D1 0x00 // ADC D1 conversion
#define CMD_ADC_D2 0x10 // ADC D2 conversion
#define CMD_ADC_256 0x00 // ADC OSR=256
#define CMD_ADC_512 0x02 // ADC OSR=512
#define CMD_ADC_1024 0x04 // ADC OSR=1024
#define CMD_ADC_2048 0x06 // ADC OSR=2048
#define CMD_ADC_4096 0x08 // ADC OSR=4096
#define CMD_PROM_RD 0xA0 // Prom read command
//_____ I N C L U D E S
#include <stdio.h>
#include <util/delay.h>
#include <util/twi.h>
#include <math.h>
//_____ D E F I N I T I O N S
unsigned char i2c_start(unsigned char address);
void i2c_stop(void);
unsigned char i2c_write( unsigned char data );
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);
void cmd_reset(void);
unsigned long cmd_adc(char cmd);
unsigned int cmd_prom(char coef_num);
unsigned char crc4(unsigned int n_prom[]); 

//********************************************************
//! @brief send I2C start condition and the address byte
//!
//! @return 0
//********************************************************
unsigned char i2c_start(unsigned char address)
{
 unsigned char twst;
TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN); // send START condition
 while(!(TWCR & (1<<TWINT))); // wait until transmission completed
 twst = TW_STATUS & 0xF8;// check value of TWI Status Register. Mask prescaler bits.
 if ( (twst != TW_START) && (twst != TW_REP_START)) return 1;
 TWDR = address; // send device address
 TWCR = (1<<TWINT) | (1<<TWEN);
// wait until transmission completed and ACK/NACK has been received
 while(!(TWCR & (1<<TWINT)));
 twst = TW_STATUS & 0xF8; 
 // check value of TWI Status Register. Mask prescaler bits.
 if ( (twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) return 1;
return 0;
} 

//********************************************************
//! @brief send I2C stop condition
//!
//! @return none
//********************************************************
void i2c_stop(void)
{
/* send stop condition */
 TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
 // wait until stop condition is executed and bus released
 while(TWCR & (1<<TWSTO)); //THIS MAKES PROBLEM FOR IS2402
} 

//********************************************************
//! @brief send I2C stop condition
//!
//! @return 0
//********************************************************
unsigned char i2c_write(unsigned char data)
{
 unsigned char twst;
 TWDR = data; // send data to the previously addressed device
 TWCR = (1<<TWINT) | (1<<TWEN);
 while(!(TWCR & (1<<TWINT))); // wait until transmission completed
 twst = TW_STATUS & 0xF8; // check value of TWI Status Register. Mask prescaler bits
 if( twst != TW_MT_DATA_ACK) return 1;
 return 0;
}
//********************************************************
//! @brief read I2C byte with acknowledgment
//!
//! @return read byte
//********************************************************
unsigned char i2c_readAck(void)
{
 TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
while(!(TWCR & (1<<TWINT)));
 return TWDR;
}
//********************************************************
//! @brief read I2C byte without acknowledgment
//!
//! @return read byte
//********************************************************
unsigned char i2c_readNak(void)
{
 TWCR = (1<<TWINT) | (1<<TWEN);
 while(!(TWCR & (1<<TWINT)));
 return TWDR;
} 

//********************************************************
//! @brief send command using I2C hardware interface
//! 
//! @return none
//********************************************************
void i2c_send(char cmd)
{
 unsigned char ret;
 ret = i2c_start(ADDR_W); // set device address and write mode
 if ( ret )
 {//failed to issue start condition, possibly no device found */
 i2c_stop();
 }
 else
 {// issuing start condition ok, device accessible
 ret=i2c_write(cmd);
 i2c_stop();
 }
}
//********************************************************
//! @brief send reset sequence
//!
//! @return none
//********************************************************
void cmd_reset(void)
{
 i2c_send(CMD_RESET); // send reset sequence
 _delay_ms(3); // wait for the reset sequence timing
}
//********************************************************
//! @brief preform adc conversion
//!
//! @return 24bit result
//********************************************************
unsigned long cmd_adc(char cmd)
{
 unsigned int ret;
 unsigned long temp=0;
 i2c_send(CMD_ADC_CONV+cmd); // send conversion command
 switch (cmd & 0x0f) // wait necessary conversion time
 {
 case CMD_ADC_256 : _delay_us(900); break;
 case CMD_ADC_512 : _delay_ms(3); break;
 case CMD_ADC_1024: _delay_ms(4); break;
 case CMD_ADC_2048: _delay_ms(6); break;
 case CMD_ADC_4096: _delay_ms(10); break;
 }
 i2c_send(CMD_ADC_READ);
 ret = i2c_start(ADDR_R); // set device address and read mode
 if ( ret )
 {//failed to issue start condition, possibly no device found
 i2c_stop();
 }
 else
 {//issuing start condition ok, device accessible
 ret = i2c_readAck(); // read MSB and acknowledge
 temp=65536*ret;
 ret = i2c_readAck(); // read byte and acknowledge
 temp=temp+256*ret;
 ret = i2c_readNak(); // read LSB and not acknowledge
 temp=temp+ret;
 i2c_stop(); // send stop condition
 } 
return temp;
} 

//********************************************************
//! @brief Read calibration coefficients
//!
//! @return coefficient
//********************************************************
unsigned int cmd_prom(char coef_num)
{
 unsigned int ret;
 unsigned int rC=0;

 i2c_send(CMD_PROM_RD+coef_num*2); // send PROM READ command
 ret = i2c_start(ADDR_R); // set device address and read mode
 if ( ret )
 {//failed to issue start condition, possibly no device found
 i2c_stop();
 }
 else
 {//issuing start condition ok, device accessible
 ret = i2c_readAck(); // read MSB and acknowledge
 rC=256*ret;
 ret = i2c_readNak(); // read LSB and not acknowledge
 rC=rC+ret;
 i2c_stop();
 }
 return rC;
}
//********************************************************
//! @brief calculate the CRC code
//!
//! @return crc code
//********************************************************
unsigned char crc4(unsigned int n_prom[])
{
 int cnt; // simple counter
 unsigned int n_rem; // crc reminder
 unsigned int crc_read; // original value of the crc
 unsigned char n_bit;
 n_rem = 0x00;
 crc_read=n_prom[7]; //save read CRC
 n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
 for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
 {// choose LSB or MSB
if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
 else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
 for (n_bit = 8; n_bit > 0; n_bit--)
 {
 if (n_rem & (0x8000))
 {
 n_rem = (n_rem << 1) ^ 0x3000;
 }
 else
 {
 n_rem = (n_rem << 1);
 }
 }
 }
 n_rem= (0x000F & (n_rem >> 12)); // final 4-bit reminder is CRC code 
n_prom[7]=crc_read; // restore the crc_read to its original place
 return (n_rem ^ 0x0);
}
//********************************************************
//! @brief main program
//!
//! @return 0
//********************************************************
void setup(){} 
void loop ()
{
 unsigned long D1; // ADC value of the pressure conversion
 unsigned long D2; // ADC value of the temperature conversion
 unsigned int C[8]; // calibration coefficients
 double P; // compensated pressure value
 double T; // compensated temperature value
 double dT; // difference between actual and measured temperature
 double OFF; // offset at actual temperature
 double SENS; // sensitivity at actual temperature
 int i;
 unsigned char n_crc; // crc value of the prom

 // setup the ports
 DDRA = 0xFE;
 DDRB = 0x0F; //SPI pins as input
 DDRC = 0x03; // I2C pins as output
 DDRD = 0x82; // RS out and tx out;

 PORTA = 0x1F; // I2C pin high
 PORTB = 0xF0;
 PORTC = 0x01;
 PORTD = 0x00;

 // initialize the I2C hardware module
 TWSR = 0; // no prescaler
 TWBR = ((F_CPU/SCL_CLOCK)-16)/2; // set the I2C speed

 D1=0;
 D2=0;
 cmd_reset(); // reset IC
 for (i=0;i<8;i++){ C[i]=cmd_prom(i);} // read coefficients
 n_crc=crc4(C); // calculate the CRC
 for(;;) // loop without stopping
 {
 D2=cmd_adc(CMD_ADC_D2+CMD_ADC_4096); // read D2
 D1=cmd_adc(CMD_ADC_D1+CMD_ADC_4096); // read D1
 // calcualte 1st order pressure and temperature (MS5607 1st order algorithm)
 dT=D2-C[5]*pow(2,8);
 OFF=C[2]*pow(2,17)+dT*C[4]/pow(2,6);
 SENS=C[1]*pow(2,16)+dT*C[3]/pow(2,7);

 T=(2000+(dT*C[6])/pow(2,23))/100;
 P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15))/100;
 // place to use P, T, put them on LCD, send them trough RS232 interface...
 }

} 

 





 
