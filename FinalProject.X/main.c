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

#define YAW_MIN             1100
#define YAW_MAX             4900
#define YAW_CENTER          3000

#define PITCH_MIN           1100
#define PITCH_MAX           4900
#define PITCH_CENTER        3000

#define MOTOR_MAX           3500
#define MOTOR_MIN           2500

#define STOP                3000

#define PWM_PERIOD          40000

#define HIGH                1
#define LOW                 0

#define ManualControl       0
#define FeedbackControl     1

#define DEADZONE            100

void setYawServo(uint16_t);
void setPitchServo(uint16_t);
void setMotorSpeed(uint16_t);
uint16_t parseInt(void);

uint16_t yawPosition = YAW_CENTER;
uint16_t pitchPosition = 3000;
uint16_t motorSpeed = 3000;

uint32_t motorSetPoint = 0;
uint8_t motorControlMode = ManualControl;

uint8_t encoderA;
uint8_t encoderB;
int32_t encoderPosition = 0;

int32_t home = 0;
int32_t leftLimit = -100000;
int32_t rightLimit = 100000;



void main (void) {
    char cmd, cmd2;
    INTERRUPT_GlobalInterruptEnable();
    INTERRUPT_PeripheralInterruptEnable();
    
    SYSTEM_Initialize();
    printf("Connect RA1 to Yaw Servo Data\r\n");
    printf("Connect RA0 to Pitch Servo Data\r\n");
    printf("Connect RA2 to the Motor Controller Data\r\n");
    printf("Connect RB0 to Encoder A\r\n");
    printf("Connect RA4 to Encoder B\r\n");
	for(;;) {
		if (EUSART2_DataReady) {			// wait for incoming data on USART
            cmd = EUSART2_Read();
			switch (cmd) {
                case 't':
                    setPitchServo(pitchPosition - 100);
                    printf("Incrementing Pitch Servo Position: %u\r\n",pitchPosition);
                    break;
                case 'T':
                    setPitchServo(pitchPosition + 100);
                    printf("Decrementing Pitch Servo Position: %u\r\n",pitchPosition);
                    break;
                case '?':
                    printf("Debug Console Help\r\n");
                    printf("? Show Help Menu\r\n");
                    printf("S/s move Yaw Servo\r\n");
                    printf("T/t move Tilt Servo\r\n");
                    printf("M/m Increase/Decrease Motor Speed \r\n");
                    printf("F/f Move Motor 10000/-10000 ticks \r\n");
                    printf("E print encoder values to console \r\n");
                    printf("o print encoder values to console \r\n");
                    printf("u print incoming EUSART1 data \r\n");
                    printf("z clear terminal\r\n");
                    printf("Z Reset Pic\r\n");
                    
                    
                    break;
                case 'S':
                    setYawServo(yawPosition+100);
                    printf("Incrementing Yaw Servo Position: %u\r\n",yawPosition);
                    break;
                case 's':
                    setYawServo(yawPosition-100);
                    printf("Decrementing Yaw Servo Position: %u\r\n",yawPosition);
                    break; 
                case 'M':
                    motorControlMode = ManualControl;
                    setMotorSpeed(motorSpeed+100);
                    printf("Incrementing Motor Speed: %u\r\n",motorSpeed);
                    break;
                case 'm':
                    motorControlMode = ManualControl;
                    setMotorSpeed(motorSpeed-100);
                    printf("Decrementing Motor Speed: %u\r\n",motorSpeed);
                    break; 
                case 'F':
                    motorControlMode = FeedbackControl;
                    motorSetPoint += 10000;
                    printf("Incrementing Motor Setpoint: %ld\r\n",motorSetPoint);
                    break;
                case 'f':
                    motorControlMode = FeedbackControl;
                    motorSetPoint -= 10000;
                    printf("Decrementing Motor Setpoint: %ld\r\n",motorSetPoint);
                    break;
                case 'E':
                    while(!EUSART2_DataReady){
                        printf("Encoder Position: %ld   SetPoint:%ld\r\n",encoderPosition,motorSetPoint);
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
                                printf("Received %u from EUSART1\r\n", cmd2);
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
        if (EUSART1_DataReady) {
            
            cmd = EUSART1_Read();
            switch (cmd) {
                case 'p':
                    printf("");
                    uint16_t pos = parseInt()<<4;
                    setYawServo(pos +YAW_MIN);
                break;
                case 't':
                    printf("");
                    uint16_t pos = parseInt()<<4;
                    //printf("Data Bits t %i\r\n",pos);
                    setPitchServo(pos + PITCH_MIN);
                break;
                case 's':
                    motorControlMode = ManualControl;
                    printf("");
                    uint16_t speed = parseInt();
                    speed = (speed <<4) - speed + YAW_MIN;
                    //printf("Data Bits s %i\r\n",speed);
                    setMotorSpeed(speed);
                break;
                case 'H':
                    home = encoderPosition;
                    break;
                case 'L':
                    leftLimit = encoderPosition;
                break;
                case 'R':
                    rightLimit = encoderPosition;
                break;
                case 'h':
                     motorControlMode = FeedbackControl;
                     motorSetPoint = home;
                    break;
                case 'l':
                    motorControlMode = FeedbackControl;
                    motorSetPoint = leftLimit;
                break;
                case 'r':
                    motorControlMode = FeedbackControl;
                    motorSetPoint = rightLimit;
                break;
                default:
                    printf("Unknown or Invalid Command %c\r\n",cmd);
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
void setPitchServo(uint16_t pos){
   
    if(pos < YAW_MIN){
        pos = YAW_MIN;   
    }
    else if(pos > YAW_MAX){
        pos = YAW_MAX;
    }
    
    pitchPosition = pos;
}
void setMotorSpeed(uint16_t speed){
    
   
    if(speed< MOTOR_MIN){
        speed = MOTOR_MIN; 
       
    }
    else if(speed > MOTOR_MAX){
        speed = MOTOR_MAX;
    }
    
    motorSpeed = speed;
}

uint16_t parseInt(){
    while(!EUSART1_DataReady);
    uint16_t position = EUSART1_Read();
    return position;
    

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
    if(motorControlMode == FeedbackControl){
        printf("%ld,    %ld\r\n",motorSetPoint,encoderPosition);
    }
    INTCONbits.TMR0IF = 0;
   

} // end

//Pitch Servo ISR
void TMR1_DefaultInterruptHandler(void){
    static uint8_t pulseHigh = false;
    if(!pulseHigh){
        PITCH_SERVO_PIN_SetHigh();
        TMR1_WriteTimer(0xFFFF -pitchPosition ); 
        pulseHigh = true;
    }
    else{
       PITCH_SERVO_PIN_SetLow();
       TMR1_WriteTimer(0xFFFF -PWM_PERIOD ); 
       pulseHigh = false;
    }
    
    
    PIR1bits.TMR1IF = 0;
}

//Feedback Motor Limits and Control ISR
void TMR2_DefaultInterruptHandler(void){
    if(motorControlMode == FeedbackControl){
        int32_t error = (encoderPosition - motorSetPoint);
        if(error > 500){
            error = 500;
        }
        if(error < -500){
            error = -500;
        }
        if(error >DEADZONE || error < -DEADZONE){
            setMotorSpeed(STOP - error);
        }
        else{
            setMotorSpeed(STOP);
        }
        
    }
    if(encoderPosition <leftLimit && motorSpeed <3000){
             setMotorSpeed(STOP);
        }
    if(encoderPosition >rightLimit&& motorSpeed >3000){
            setMotorSpeed(STOP);
        }
    
    TMR2_WriteTimer(0); 
    PIR1bits.TMR2IF = 0;
}

//Motor Control ISR
void TMR5_DefaultInterruptHandler(void){
    static uint8_t pulseHigh = false;
    if(!pulseHigh){
        MOTOR_PIN_SetHigh();
        TMR5_WriteTimer(0xFFFF - motorSpeed ); 
        pulseHigh = true;
    }
    else{
       MOTOR_PIN_SetLow();
       TMR5_WriteTimer(0xFFFF - PWM_PERIOD ); 
       pulseHigh = false;
    }
    
    
     PIR5bits.TMR5IF = 0;
}

//Encoder Channel A ISR
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

//Encoder Channel B ISR
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
