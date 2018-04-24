#include "mcc_generated_files/mcc.h"

/*
 * protocol will be 1 command byte folowed by one data byte:
 * commands:
 *     p: set pan position (type int8_t)
 *     t: set tilt position (type int8_t)
 *     s: set slide speed (type int8_t)
 *     h: goto home (no data byte)
 *     l: goto left (no data byte)
 *     r: goto right (no data byte)
 *     H: set home (no data byte)
 *     L: set left (no data byte)
 *     R: set right (no data byte)
 */

#define YAW_MIN         1106
#define YAW_MAX         4900
#define PWM_PERIOD      40000

#define HIGH            1
#define LOW             0

void setYawServo(uint16_t);

uint16_t yawPosition = 3000;
uint8_t encoderA;
uint8_t encoderB;
int32_t encoderPosition = 0;


void main (void) {
    char cmd, cmd2;
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    SYSTEM_Initialize();
   // T0CONbits.TMR0ON = 1;

	for(;;) {
		if (EUSART2_DataReady) {			// wait for incoming data on USART
            cmd = EUSART2_Read();
			switch (cmd) {
                case '?':
                    printf("Connect RA1 to Servo Data\r\n");
                    printf("Connect RB0 to Encoder A\r\n");
                    printf("Connect RA4 to Encoder B\r\n");
                    break;
                case 'S':
                    setYawServo(yawPosition+100);
                    printf("Incrementing Servo Position: %u\r\n",yawPosition);
                    break;
                case 's':
                    setYawServo(yawPosition-100);
                    printf("Decrementing Servo Position: %u\r\n",yawPosition);
                    break; 
                case 'E':
                    while(!EUSART2_DataReady){
                        printf("Encoder Position: %ld\r\n",encoderPosition);
                    }
                    EUSART2_Read();
                    break;
                case 'o':
                    printf(" k\r\n");
                    break;
                case 'u':
                    while (!EUSART2_DataReady) {
                        if (EUSART1_DataReady) {
                            cmd2 = EUSART1_Read();
                            if (cmd2 == 'o') {
                                printf("ok\r\n");
                            }
                            else {
                                printf("Recieved %u from EUSART1\r\n", cmd2);
                            }
                        }
                    }
                    EUSART2_Read();
                    break;
                case 'z':
                    printf("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
                    break;
                case 'Z':
                    RESET();
                    break;
                default:
                    printf("Unknown key %c\r\n",cmd);
                    break;
			} 

		}
	} 
}

void setYawServo(uint16_t pos){
   
    if(pos < YAW_MIN){
        pos = YAW_MIN;   
    }
    else if(pos > YAW_MAX){
        pos = YAW_MAX;
    }
    
    yawPosition = pos;
}


//Yaw Servo ISR
void TMR0_DefaultInterruptHandler(void){
    
    static uint8_t pulseHigh = false;
    if(!pulseHigh){
        YAW_SERVO_PIN_SetHigh();
        TMR0_WriteTimer(0xFFFF -yawPosition ); 
        pulseHigh = true;
    }
    else{
       YAW_SERVO_PIN_SetLow();
       TMR0_WriteTimer(0xFFFF -PWM_PERIOD ); 
       pulseHigh = false;
    }
    
    
    INTCONbits.TMR0IF = 0;
   

} // end

void CCP4_CallBack(uint16_t capturedValue)
{
    if(CCP4CONbits.CCP4M == 0b0101){ //on Rising Edge
            CCP4CONbits.CCP4M = 0b0100; //set to falling edge
            encoderA = HIGH; //zero timer
            if(encoderB==LOW){
                encoderPosition++;
            }
            else{
                encoderPosition--;
            }
    }
    else{
        CCP4CONbits.CCP4M = 0b0101; //set to rising edge
        encoderA = LOW;
    }

    PIR4bits.CCP4IF = 0;
}

void CCP5_CallBack(uint16_t capturedValue)
{
    if(CCP5CONbits.CCP5M == 0b0101){ //on Rising Edge
            CCP5CONbits.CCP5M = 0b0100; //set to falling edge
            encoderB = HIGH; 
            
    }
    else{
        CCP5CONbits.CCP5M = 0b0101; //set to rising edge
        encoderB = LOW;
    }

    PIR4bits.CCP5IF = 0;
}
