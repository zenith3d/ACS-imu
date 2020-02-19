#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <string.h>

#include "led.h"
#include "spi.h"
#include "fmt1030.h"


/*** CAN IDs ***/
#define CAN_ID_IMU      0X20    // The CAN data identifier


/** LEDS DECLARATION **/
Led redLed(&PORTB, 6, 0);       // The RED Led
Led yellowLed(&PORTB, 5, 0);    // The RED Led


/*** GLOBAL VARIABLES ***/
Quaternion attitude;            // []       The actual body orientation
Vector rateofturn;              // [rad/s]  The actual body rate of turn

MtsspConfiguration notification;
Fmt1030 imu;                    // The IMU object
uint8_t can_buff[8];            // The CAN buffer used to send data


void initCANBus(){
    /** 
	 * Function that initializes the CANBus according to the wiring and the needed speed (500kb/s).
	 * It also enable the CAN module interruptions
	**/

	// Enable CAN transmission
	DDRC |= 0x80;			        // Configure the STANDBY pin as an output
	PORTC &= 0x7F; 			        // Activate the MCP2562 (STANDBY to 0)

	// CAN General CONtrol register (see p.158)
	CANGCON = (1<<SWRES);		    // Reset the CAN module
	CANGCON = 0x02; 		        // Set the CAN module to 'enable' mode

	// CAN General Interrupt Enable register (see p.161)
	CANGIE |= (1<<ENIT) | (1<<ENRX);// Enable all interrupts, receive & transmit interrupts

	// CAN Bit Timing registers (see p.172/163)
	CANBT1 = 0x06;			        // Baud rate prescaler (500kb/s)
	CANBT2 = 0x04;			        // Propagation time segment
	CANBT3 = 0x13;  		        // Compensate phase edge errors & filter noise (using 3 points sampling)

	// CAN Highest Priority MOb register
	CANHPMOB = 0x00; 		        // Set no priority  
}

void sendCANdata(uint8_t id, uint8_t dlc, uint8_t* data){
    CANPAGE  = 0x00;    //Selection of MOB 0
    CANIDT4 = 0x00;     // Config as data (rtr = 0)
    CANIDT3 = 0x00;
    CANIDT2 = (uint8_t)( (id & 0x00F)<< 5 ); // set the identifier
    CANIDT1 = (uint8_t)( id >> 3 );

    for(uint8_t i=0; i< dlc; i++){
	    CANMSG = data[i];
	    CANPAGE = 0x00 + 1 + i;
    }

    CANCDMOB = 0x40 | dlc;  // send the message using the MOB 0 - DLC = 8
    while ( !(CANSTMOB & (1 << TXOK)));
}

int main(void) {
    cli();

    // Enable SPI level adaptation
    DDRC |= 0x40;
    PORTC |= 0x40;

    // CAN Bus initialization (500Kb/s)
    initCANBus();

    // IMU initialization
    imu.fmt1030_init();
    _delay_ms(200);
    imu.goToConfig();
    _delay_ms(100);
    imu.setMyOutputConfiguration(); // Using a specific method rather than the general one because it doesn't handle multiple args
    //imu.setOutputConfiguration(2, XDI_QUATERNION, FREQ_50Hz, XDI_RATEOFTURN, FREQ_50Hz);
    _delay_ms(100);
    imu.gotoMeasurement();
    _delay_ms(100);

    // Blink the LEDs
    for(int i=0; i<4; i++){
        redLed.blink(50);
        yellowLed.blink(50);
    }

    sei();

    while(1){
        // Read the number of bytes to be read in the measurement register
        notification = imu.read_pipe_status();
        imu.readMesure(notification.measurementMessageSize);

        // Measurement register reading
        attitude = imu.getQuaternion(); 
        rateofturn = imu.getRateOfTurn();

        // Send data on CAN bus
        // Each float is send every 2ms, so that the total refresh rate is 1/(7*2ms) = 50Hz
        redLed.on();    // Toggle the red led for debugging

        can_buff[0] = 0; memcpy(&can_buff[1], &(attitude.w), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
        _delay_ms(2);

        can_buff[0] = 1; memcpy(&can_buff[1], &(attitude.x), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
        _delay_ms(2);

        can_buff[0] = 2; memcpy(&can_buff[1], &(attitude.y), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
	    _delay_ms(2);

        can_buff[0] = 3; memcpy(&can_buff[1], &(attitude.z), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
	    _delay_ms(2);

        can_buff[0] = 4; memcpy(&can_buff[1], &(rateofturn.x), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
	    _delay_ms(2);

        can_buff[0] = 5; memcpy(&can_buff[1], &(rateofturn.y), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
	    _delay_ms(2);

        can_buff[0] = 6; memcpy(&can_buff[1], &(rateofturn.z), sizeof(float));
        sendCANdata(CAN_ID_IMU, 5, can_buff);
	    _delay_ms(2);

        redLed.off();
    }
}
