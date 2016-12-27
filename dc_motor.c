// by Benjamin Hahn and Edward Brewin

#include <xc.h>
#include "dc_motor.h"

void initPWM(){ 
    // initialise pin settings
    TRISB = 0;
    LATBbits.LATB0 = 1;
    LATBbits.LATB1 = 0;
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 0;
    
    PTCON0 = 0b00000000; //free running mode, 1:1 prescaler = 0.5us
    PTCON1 = 0b10000000; //enable pwm timer
    
    PWMCON0 = 0b01101111; //pwm0/1 and 3/4 enabled, all interdependant mode
    PWMCON1 = 0x00; //special features, all 0 (default)
    //PTPER needs to be 199, calculated from Tpwm
    //199 in binary is 11000111
    PTPERL = 0b11000111; //base PWM period, low byte
    PTPERH = 0b00000000;//base PWM period, high byte
        
    // set initial duty cycle to 0% for both PWM outputs
    PDC0L = 0;
    PDC0H = 0; //if PDC0L is 32 or higher, the bit shift means that PDC0H will need to have 1's in it
    PDC1L = 0;
    PDC1H = 0;
}

// function to set PWM output from the values in the motor structure
void setMotorPWM(struct DC_motor *m)
{
    int PWMduty; //tmp variable to store PWM duty cycle

    PWMduty = (m->power*m->PWMperiod)/100;  //calculate duty cycle (value between 0 and PWMperiod)
    
    if (m->direction) //if forward direction
    {
        LATB=LATB|(1<<(m->dir_pin)); //set dir_pin high in LATB
		PWMduty=m->PWMperiod-PWMduty; //need to invert duty cycle as direction is high (100% power is a duty cycle of 0)
    }
    else //if reverse direction
    {
        LATB=LATB&(~(1<<(m->dir_pin))); //set dir_pin low in LATB
    }

	//write duty cycle value to appropriate registers
    *(m->dutyLowByte)=PWMduty<<2;
    *(m->dutyHighByte)=PWMduty>>6;
}

//increases a motor to full power over a period of time
void setMotorFullSpeed(struct DC_motor *m)
{
    for (m->power; (m->power)<=60; m->power++){ //increase motor power until 100
            setMotorPWM(m);	//pass pointer to m to setMotorSpeed function (not &m)
            __delay_ms(1);	//delay of 1 ms (100 ms from 0 to 100 full power)
    }
}

//function to stop a motor gradually 
void stopMotor(struct DC_motor *m)
{
	// a loop to slow the motor power down to zero
    for (m->power; (m->power)>0; m->power--)
    {
        setMotorPWM(m);	//pass pointer to m to setMotorSpeed function (not &m)
        __delay_ms(1);
    }
}

//function to stop the robot gradually 
void stop(struct DC_motor *mL, struct DC_motor *mR)
{
	// a loop to slow both motors down to zero power
    stopMotor(mL);
    stopMotor(mR);
}

//function to make the robot turn left 
void turnLeft(struct DC_motor *mL, struct DC_motor *mR)
{
        // stop robot first
        stop(mL,mR);
    
        // run motors in opposite directions
        mL->direction = 0;
        mR->direction = 1;  
              
        setMotorFullSpeed(mL);
        setMotorFullSpeed(mR);
}

//function to make the robot turn right 
void turnRight(struct DC_motor *mL, struct DC_motor *mR)
{
        // stop robot first
        stop(mL,mR);
    
        // run motors in opposite directions
        mL->direction = 1;
        mR->direction = 0;  
      
        setMotorFullSpeed(mL);
        setMotorFullSpeed(mR);
}

//function to make the robot go straight
void fullSpeedAhead(struct DC_motor *mL, struct DC_motor *mR) 
{
        // stop robot first
        stop(mL,mR);
    
        // run motors in the same direction
        mL->direction = 1;
        mR->direction = 1;  
      
      // gradually increase motor speed
        setMotorFullSpeed(mL);
        setMotorFullSpeed(mR);
}

//function to make the robot go backwards
void fullSpeedBackwards(struct DC_motor *mL, struct DC_motor *mR) 
{
    // stop robot first
    stop(mL,mR);

    // run motors in the same direction
    mL->direction = 0;
    mR->direction = 0;  

    // gradually increase motor speed
    setMotorFullSpeed(mL);
    setMotorFullSpeed(mR);
}

void initMotorsLR(struct DC_motor *mL, struct DC_motor *mR)
{
    // PWM cycle
    int PWMcycle = 199;
    
    // initialise left motor
    mL->power=0; //zero power to start
    mL->direction=1;//set default motor direction
    mL->dutyLowByte=(unsigned char*)(&PDC0L);//store address of PWM duty low byte
    mL->dutyHighByte=(unsigned char*)(&PDC0H);//store address of PWM duty high byte
    mL->dir_pin=0;//pin RB0/PWM0 controls direction
    mL->PWMperiod=PWMcycle;//store PWMperiod for motor
    
    // initialise right motor
    mR->power=0; //zero power to start
    mR->direction=1;//set default motor direction
    mR->dutyLowByte=(unsigned char*)(&PDC1L);//store address of PWM duty low byte
    mR->dutyHighByte=(unsigned char*)(&PDC1H);//store address of PWM duty high byte
    mR->dir_pin=2;//pin RB0/PWM0 controls direction
    mR->PWMperiod=PWMcycle;//store PWMperiod for motor
}