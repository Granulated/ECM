// This code controls a robot used to detect an IR beacon, read an RFID code and return back to the base safely.
// by Benjamin Hahn and Edward Brewin

#include <xc.h>
#include <stdio.h>
#include "LCD.h"
#include "dc_motor.h"

#pragma config OSC = IRCIO, MCLRE = OFF, LVP=OFF // internal oscillator
#define _XTAL_FREQ 8000000


//IR and RFID global variables
volatile unsigned int IRreading1 = 1; // global variable to store IR readings from sensor 1
volatile unsigned int IRreading2 = 1; // global variable to store IR readings from sensor 2
char RFIDbuf[11] = {0}; // global variable to store entire RFID message
int RFIDcheckSum[2] = {0};
int stopReadRFID = 1;
int beginReadRFID = 0;
int RFIDcounter = 0;

// position variables
unsigned int moves[100] = 0;          // 'moves' stores the time it took to do the rotations and translations the robot makes
int moveDictionary[100] = 0; // 'moveDictionary' contains a 0 if the move was a rotation and a 1 if the move was a translation
char numberOfMoves = 0; // variable to store the number of moves, used to reference current array
int previousMoveType; // saves the type of the previous movement
unsigned int movementTime = 0; // variable to keep track how long each movement takes

char searchingForBeacon = 1; // equals 1 if still searching for the beacon and equals 0 if the RFID code has been received and the device is navigating back towards the base
char firstMoveDone = 0; // flag to check whether a first movement has been done
char returnedHome = 0;

struct DC_motor motorL, motorR; //declare 2 motor structures. global motor setup


// Function to convert RFID ASCII characters to Decimal
char ASCIItoDecimal(int n)
{
    if (n < 58 && n > 47) {
        return n - '0'; // ASCII '0' is decimal 48
    } else if (n >= 65 && n <= 70){
        return n - 'A' + 10; // ASCII 'A' is decimal 65
    }
}


// function to then convert ASCII number to hex and use longitudinal parity checksum algorithm
int checkSum(char input[10])
{
    int output[10];
    for (int i=0; i<10; i++)
    {
        output[i] = ASCIItoDecimal(input[i]);
    }
    
    int checksum = ((output[0] << 4) + output[1]); // initialise checksum variable to the first two, converted, hex digits
    
    
    for (int i=1; i<5; i++)
    {
        checksum = checksum ^ ((output[2*i] << 4) + output[2*i+1]); // XOR each checksum value with the next hex pair until all have been read
    }
    
    return checksum;
}


//convert specific ASCII checksum bytes in message to hex
int ASCIIchecksumToHex(int input[2])
{
    int checksum1, checksum2;
    checksum1 = ASCIItoDecimal(input[0]);
    checksum2 = ASCIItoDecimal(input[1]);
    return ((checksum1 << 4) + checksum2);
}


// initialise communication for RFID
void initSerialComs() 
{
    // enable background resources and pins used
    SPBRG = 203; //set baud rate to 9600 using SPBRG = 204 or use SPBRG = 102 for a baud rate of 19'200
    SPBRGH = 0;
    BAUDCONbits.BRG16 = 1; //set baud rate scaling to 16 bit mode
    TXSTAbits.BRGH = 1; //high baud rate select bit
    RCSTAbits.CREN = 1; //continuous receive mode
    RCSTAbits.SPEN = 1; //enable serial port, other settings default
    
    TRISCbits.RC7 = 1; // set pins RC6 and RC7 as inputs
    PORTCbits.RC7 = 0;
    TRISCbits.RC6 = 1; // set pins RC6 and RC7 as inputs
    PORTCbits.RC6 = 0;
}


//Initialise IR sensor timer and pins
void init_IR()
{
    T5CON = 0b00001001; //initialise timer 5. pg139
    CAP1CON = 0b01000110; // sets CAP1 to read PWM pulse width
    CAP1BUFL = 0;
    CAP1BUFH = 0;
    
    CAP2CON = 0b01000110; // sets CAP2 to read PWM pulse width
    CAP2BUFL = 0;
    CAP2BUFH = 0;   
    
    DFLTCON = 0b00011000; // enable noise filter for CAP1 and CAP2
}

// initialise all interrupts necessary
void init_Interrupts()
{
    // default interrupt priority registers to zero
    IPR1 = 0;
    IPR2 = 0;
    IPR3 = 0;
    
    // enable interrupts - global
    INTCONbits.GIEH = 1; // Global interrupt Enable bit
    INTCONbits.GIEL = 1; // Peripheral interrupt enable bit
    
    // enable interrupts - IR sensors
    PIE3bits.IC1IE = 1; // enable CAP1 interrupt
    PIE3bits.IC2QEIE = 1; // enable CAP2 interrupt
    RCONbits.IPEN = 1; // Interrupt Priority Enable bit, 1 = on
    IPR3bits.IC1IP = 0; // set CAP1 interrupt priority to low
    IPR3bits.IC2QEIP = 0; // set CAP2 interrupt priority to low
    //IPR3bits.TMR5IP = 0; // Timer 5 priority to low
    
    // enable interrupts - RFID
    PIE1bits.RCIE = 1; // interrupt enable for serial input
    IPR1bits.RC1IP = 1; // set EUSART receive interrupts to high priority
    
//    // enable interrupt - button
//    TRISCbits.RC3 = 1; // set RC3 as input
//    INTCON3bits.INT1IE = 1; //INT1 External Interrupt Enable bit for button on RC3
//    INTCON3bits.INT1IP = 1; // set INT1 to high priority
    
}


//initialise timer for for btracking position buffer used by return journey 
void initTimer() {
 //timer setup
 T0CONbits.TMR0ON=1; //turn on timer0
 T0CONbits.T016BIT=0; // 16bit mode
 T0CONbits.T0CS=0; // use internal clock (Fosc/4)
 T0CONbits.PSA=0; // enable prescaler
 T0CONbits.T0PS=0b111; // set prescaler value
 }


// rotate robot for a specified time in a specified direction. 
void turnByAngle(unsigned int durationOfRotation, int direction)
{
    // reset timer back to zero to get a clean reading
    TMR0H = 0;
    TMR0L = 0;
    __delay_us(50);
    
    // keep track of time elapsed, initialise to zero
    movementTime = 0; 
    
    // start the turn in the correct direction
    // the directions of the rotations are the exact opposite on the return journey compared to the journey to the beacon. 
    // Thus, where durationOfRotation is negative, signifying that the robot turned left on the way to the beacon, the robot
    // will now turn right, and vice versa.
    if (direction == 1) // in journey to beacon, left direction was -1, therefore now should be +1
    {
    turnLeft(&motorL, &motorR);
    }
    else if (direction == -1)// in journey to beacon, right direction was +1, therefore now should be -1
    {
    turnRight(&motorL, &motorR);    
    }
      
    while (movementTime <= durationOfRotation) // turn as long as it takes
    {
        // get timer value telling us how long the rotation has been taking
        movementTime = TMR0L;
        movementTime += (TMR0H << 8);
    }
    
    // stop once rotation has finished
    stop(&motorL, &motorR);
}    
    
// move robot forwards for a certain amount of time
void moveDistance(unsigned int DurationOfTranslation)
{

    // reset timer back to zero to get a clean reading
    TMR0H = 0;
    TMR0L = 0;
    
    // keep track of time elapsed, initialise to zero
    movementTime = 0; 
    
   //move backwards
    fullSpeedBackwards(&motorL, &motorR);
    
    while (movementTime <= DurationOfTranslation) // turn as long as it takes
    {
        // get timer value telling us how long the rotation has been taking
        movementTime = TMR0L;
        movementTime += (TMR0H << 8);
    }
    
    // stop once translation has finished 
    stop(&motorL, &motorR);
}

// repeat the moves done to return to the starting point
void returnToBase()
{
        //start reverse journey, running stored forward movement operations in reverse
    // iterate over the array containing previous movements backwards
    for (int i = numberOfMoves; i >= 0; i--)
    {
        if ((moveDictionary[i] == 1) || (moveDictionary[i] == -1)) // if the move was a rotation
        {
            turnByAngle(moves[i], moveDictionary[i]);
        }
        else if (moveDictionary[i] == 0) // if the move was a translation
        {
            moveDistance(moves[i]);
        }
    }
    
    returnedHome = 1; // set returned home value to 1. This stops robot. Generic 'stop' dc motor function didnt seem to run correctly here.
}


// high priority interrupt to read RFID value
void high_priority interrupt InterruptHandlerHigh() {
    if (PIR1bits.RCIF) {
        
//        stop the motor so the robot doesn't plough through the beacon
            if (RCREG == 0x02 && beginReadRFID == 0) { //if start byte received, and no previous message being received
                beginReadRFID = 1; // begin read
                stopReadRFID = 0;
            }
            else if (RCREG == 0x03) { // if end byte received
                stopReadRFID = 1; // stop reading
                beginReadRFID = 0;
                }
            else if (beginReadRFID == 1 && stopReadRFID == 0) { // while reading is in operation
                if (RFIDcounter < 10) { // if in the main RFID message, write to RFID array
                    RFIDbuf[RFIDcounter] = RCREG; // write each read in value to corresponding entry in array
                    
                } else if (RFIDcounter == 10){
                    RFIDcheckSum[0] = RCREG; // for final two bytes, write to separate checksum array
                }
                else if (RFIDcounter == 11)
                {
                    RFIDcheckSum[1] = RCREG;
                }

                RFIDcounter++; // increment counter to next RFID array position
            }
        
        // stop the search for the beacon after the RFID has been fully read
        if (searchingForBeacon == 1 && RFIDcounter == 11)
        {
            searchingForBeacon = 0;
        }
        PIR1bits.RCIF = 0; // clear the interrupt flag
        // turn off the IR interrupts
        PIE3bits.IC1IE = 0; // disable CAP1 interrupt to prevent IR readings altering movement of robot
        PIE3bits.IC2QEIE = 0; // disable CAP2 interrupt to prevent IR readings altering movement of robot
    }
}

 //low priority interrupt to catch changes in IR value
void low_priority interrupt InterruptHandlerLow() {
if (PIR3bits.IC1IF) // read left IR sensor value
    {
        IRreading1 = CAP1BUFL;
        IRreading1 += ((unsigned int) CAP1BUFH << 8); // high and low byte amalgamated into a 16 bit number
        PIR3bits.IC1IF = 0; // clear the interrupt flag
    }
    if (PIR3bits.IC2QEIF) //read right IR sensor value
    {       
        IRreading2 = CAP2BUFL;
        IRreading2 += ((unsigned int) CAP2BUFH << 8);
        PIR3bits.IC2QEIF = 0; // clear the interrupt flag
    }
}

// check the two IR readings to determine the direction of the IR beacon and move accordingly. Simultaneously record
// the moves and rotations in an array which will be used for the return journey.
void moveToBeacon(int IR1, int IR2) {
    int comparisonParameter = 400; // value used to determine what difference in IR values between the two sensors
    
    // turn left if beacon is to the left and not already turning left
    if (((IR1 - IR2) > comparisonParameter) && !(motorL.direction == 0 && motorR.direction == 1)) {
        //check if the first move has been done, since only then can we record the time it took to complete the first move
        if (firstMoveDone) {
            // get timer value telling us how long the previous movement took and store it in an array
            int curTime = TMR0L;
            curTime += (TMR0H << 8);
            moves[numberOfMoves] = curTime;

            // save the type of the last move - left rotation (=-1), right rotation (=1) or translation (=0) -
            // in the corresponding array element of moveDictionary
            moveDictionary[numberOfMoves] = previousMoveType;
            
            numberOfMoves++; // increment the number of moves
        } 
        else 
        {
            firstMoveDone = 1; // skip updating the array the first time
        }
        previousMoveType = -1; // set previousMoveType to -1, corresponding to a rotation to the left

        // reset timer back to zero to avoid overflow
        TMR0H = 0;
        TMR0L = 0;

        // turn the robot
        turnLeft(&motorL, &motorR);
    }
    
    // go straight ahead if beacon is more or less straight ahead and the robot is not already going straight ahead
    else if (((IR1 - IR2) < comparisonParameter && (IR1 - IR2) > -comparisonParameter) && !(motorL.direction == 1 && motorR.direction == 1)) 
    {
        //check if the first move has been done, since only then can we record the time it took to complete the first move
        if (firstMoveDone) {
            // get timer value telling us how long the previous movement took and store it in an array
            int curTime = TMR0L;
            curTime += (TMR0H << 8);
            moves[numberOfMoves] = curTime;
            
            // save the type of the last move - left rotation (=-1), right rotation (=1) or translation (=0) -
            // in the corresponding array element of moveDictionary
            moveDictionary[numberOfMoves] = previousMoveType;
            
            numberOfMoves++; // increment the number of moves
        } 
        else 
        {
            firstMoveDone = 0; // skip updating the array the first time
        }

        previousMoveType = 0; // set previousMoveType to 0, corresponding to a translation

        // reset timer back to zero to avoid overflow
        TMR0H = 0;
        TMR0L = 0;

        // have the robot go straight ahead
        fullSpeedAhead(&motorL, &motorR);
    }        
    
    // turn right if the beacon is to the right and the robot isn't already going to the right
    else if (((IR1 - IR2) < -comparisonParameter) && !(motorL.direction == 1 && motorR.direction == 0)) 
    {
        //check if the first move has been done, since only then can we record the time it took to complete the first move
        if (firstMoveDone) {
            
            // get timer value telling us how long the previous movement took and store it in an array
            int curTime = TMR0L;
            curTime += (TMR0H << 8);
            moves[numberOfMoves] = curTime;
            
            // save the type of the last move - left rotation (=-1), right rotation (=1) or translation (=0) -
            // in the corresponding array element of moveDictionary
            moveDictionary[numberOfMoves] = previousMoveType;
            
            numberOfMoves++; // increment the number of moves
        } 
        else 
        {
            firstMoveDone = 1; // skip updating the array the first time
        }

        previousMoveType = 1; // set previousMoveType to 1, corresponding to a rotation to the right

        // reset timer back to zero to avoid overflow
        TMR0H = 0;
        TMR0L = 0;

        // turn the robot
        turnRight(&motorL, &motorR);
    }
}

// Initialise all prerequisite settings on the PIC18F4331. Pressing the button starts the program. 
// During runtime, the robot searches for the IR beacon and attempts to navigate towards it. Once it has been
// located, the RFID code is read and the checksum calculated. Then the robot runs the same path in reverse through 
// the use of an array storing the times and directions.
int main(void) {
    OSCCON = 0b11110010; // internal oscillator, 8MHz
    while (!OSCCONbits.IOFS); //Wait for OSC to become stable

    ANSEL0 = 0; // set all cap pins to digital input instead of analogue
    ANSEL1 = 0;
    
    LCD_Init(); // initialise LCD screen
    
    // wait until button is pressed to start the search routine 
    LCD_String("press button, go");
    TRISAbits.RA0 = 1; // set pin connected to button as input
    while (PORTAbits.RA0 == 0); // wait until the button has been pressed
    
    init_Interrupts(); // initialise all interrupts
    init_IR(); // initialise IR sensors
    initTimer(); // initialise timer0
    
    TRISAbits.RA2 = 1; // set CAP1 (=RA2) as input
    TRISAbits.RA3 = 1; // set CAP2(=RA3) as input

    // initialise motors
    initPWM(); //initialise PWM settings
    initMotorsLR(&motorL, &motorR);
    
    //initialise RFID
    initSerialComs(); // initialise serial communication
    
    char IRbuf[16]; // variable to store IR values
    unsigned char checkSumBuf[12]; // variable to store checksum
   
    // Before starting the main search routine, rotate robot until beacon is straight ahead
    char initialSearch = 1;
    while (initialSearch)
    {
        if (((IRreading2 - IRreading1) > 300) && ((IRreading1 - IRreading2) < - 300)&& IRreading1 < 10000 && IRreading2 < 10000)
        {
        turnRight(&motorL, &motorR);
        __delay_ms(80);
        }
        else
        {
            initialSearch = 0;
               stop(&motorL, &motorR);

        }
    }
    
    
    while (1) {
        //if returnToBase has completed, stop
        if (returnedHome) 
        {
            stop(&motorL, &motorR);
        }
        
        // print RFID values to line 2 on LCD screen only when a 
        // complete message has been received
        if (beginReadRFID == 0 && stopReadRFID == 1 && RFIDcounter > 10) {
                SetLine(2);
                sprintf(checkSumBuf, "%s, %d", RFIDbuf, checkSum(RFIDbuf));
//              sprintf(checkSumBuf, "%d, %d  ", checkSum(RFIDbuf), ASCIIchecksumToHex(RFIDcheckSum)); // use this to display 
                                                                                                       // the received and the calculated checksum instead
                LCD_String(checkSumBuf);
        }
        
        // check if beacon has been found
        if (searchingForBeacon == 1) {
            // find the direction of the beacon and move towards it
            moveToBeacon(IRreading1, IRreading2);
            SetLine(1);
            sprintf(IRbuf, "%d|%d          ", IRreading1, IRreading2); // read IR and format as string
            LCD_String(IRbuf); // print IR values to screen    

        } 
        else if (searchingForBeacon == 0 && returnedHome == 0) // if the beacon has been found, 
        {
            // since the last movement was stopped by the high priority interrupt and the robot won't turn another
            // time (it has found the beacon, after all), we need to record the time of the last move manually here 
            // to ensure it will be saved as well.
            int curTime = TMR0L;
            curTime += (TMR0H << 8);
            moves[numberOfMoves] = curTime;
            numberOfMoves++; // increment the number of moves

            // save the type of the last move - left rotation (=-1), right rotation (=1) or translation (=0) -
            // in the corresponding array element of moveDictionary
            moveDictionary[numberOfMoves] = previousMoveType;
            
            // return back to the starting position
            SetLine(1);
            LCD_String("going home   ");
            returnToBase();
        }
    }
}

