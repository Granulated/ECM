#include "simpleTrig.h"

// The following functions are part of an alternative approach for returning back to the starting point
// by continuously updating the position and global angle of the robot after each change of direction. Using 
// Taylor approximations of trigonometric functions, a direct path back to the starting point can be found.
// Despite multiple attemps at code optimization the PIC18F4331 does not have sufficient memory to store the use
// of four trigonometric functions (our implementation used around 12'000 bytes vs. the 8Kbytes available). 
// Based upon code from http://www.ganssle.com/articles/atrig.htm

// by Benjamin Hahn and Edward Brewin

// global variables
float linearSpeed = 2.7; // [m/s] experimentally measured over 10m
float angularSpeed = 1.235; // [rad/s], experimentally measured over 10 rotations
float xCoord, yCoord, globalAngle;


// taylor expansion of cos and sin
float trig(float x, char sincos)
{
    float p1,p2,p3,p4,y,t,absx,frac,pi2; // declare necessary variables
                                         // p1, p2, p3 and p4 are the taylor expansion coefficients
    char quad; // stores the quadrant
    pi2=1.570796326794896; // pi/2 
    
    absx=x; // get the absolute value of x
    if (x<0) absx=-absx; 	     
    
    quad=(int) (absx/pi2);  // quad can be 0 - 3
    frac= (absx/pi2) - quad;  // fractional part of the input
    if(quad==0) t=frac * pi2; // t corresponds to the relative angle in the quadrant
    if(quad==1) t=(1-frac) * pi2;
    if(quad==2) t=frac * pi2;
    if(quad==3) t=(frac-1) * pi2;
    
    // use the taylor coefficients for sine
    if (sincos == 's')
    {
        p1=-0.166666666666; // 1/6
        p2= 0.008333333333; // 1/120
        p3=-0.000198412666; // 1/5'040
        p4= 0.000002755731; // 1/362'880
        
        if (quad == 1 || quad == 3) t=-t; // change sign of quad if we're calculating sine
        
        y = t * (1 + (p1*t*t) + (p2*t*t*t*t) + (p3*t*t*t*t*t*t) + (p4*t*t*t*t*t*t*t*t)); // calculate 7th order taylor expansion of sine
    }
    else if(sincos == 'c') // use taylor coefficients for cosine
    {
        p1=-0.5; // 1/2
        p2= 0.0416666666666; // 1/24
        p3=-0.0013888888888; // 1/720
        p4= 0.0000248015873; // 1/40320
        t = t*t; // reduce computational load
        y=1 + (p1*t) + (p2*t*t) + (p3*t*t*t) + (p4*t*t*t*t); // calculate 8th order taylor expansion of cosine
    }
    if(quad==2 || quad==1) y=-y;  // change sign of quad if we're calculating cosine
    return(y);
}

//taylor expansion of arctan - works for small angles only
float atan(float x) {
    float p0, p1, p2, p3, p4, y, t, absx, frac, quad, pi2;
    
    // taylor expansion coefficients for arctan
    p0 = 1;
    p1 = -0.333333333333;
    p2 = 0.2;
    p3 = -0.142857142857;
    p4 = 0.111111111111;
    pi2 = 1.570796326794896; 
    float t2 = t * t; // calculate t squared to reduce computational load
    y = t * (p0 + (p1 * t2) + (p2 * t2 * t2) + (p3 * t2 * t2 * t2) + (p4 * t2 * t2 * t2 * t2)); // eight order taylor expansion of arctan
    return (y);
}

// update global position of the robot
void updatePosition()
{
    xCoord += trig(globalAngle,'c') * linearSpeed * movementTime;
    yCoord += trig(globalAngle, 's') * linearSpeed * movementTime;
}

void turnByAngle(float angle)
{
    float durationOfTurn = angle / angularSpeed; // get the time it will take to conduct
                                                 // the rotation
    
    // reset timer back to zero to get a clean reading
    TMR0H = 0;
    TMR0L = 0;
    
    // keep track of time elapsed, initialise to zero
    movementTime = 0; 
    
    // start the turn 
    turnLeft(&motorL, &motorR);
    
    while (movementTime <= durationOfTurn) // turn as long as it takes
    {
        // get timer value telling us how long the rotation has been taking
        movementTime = TMR0L;
        movementTime += (TMR0H << 8);
    }
    
    // stop once rotation has finished 
    stop(&motorL, &motorR);
}    
    
void moveDistance(float distance)
{
    float durationOfMovement = distance / linearSpeed; // get the time it will take to move to the base
    
    // reset timer back to zero to get a clean reading
    TMR0H = 0;
    TMR0L = 0;
    
    // keep track of time elapsed, initialise to zero
    movementTime = 0; 
    
    // go straight ahead
    fullSpeedAhead(&motorL, &motorR);
    
    while (movementTime <= durationOfMovement) // go straight ahead as long as it takes
    {
        // get timer value telling us how long the translation has been taking
        movementTime = TMR0L;
        movementTime += (TMR0H << 8);
    }
    
    // stop once translation has finished 
    stop(&motorL, &motorR);
}


// function called once the beacon has been found to return robot to the base
void returnToBase()
{
    float angleOfBase = M_PI - atan(yCoord) + globalAngle; // CHANGE ATAN ARGUMENT
    float distanceToBase = xCoord/trig(angleOfBase, 'c');

    
    turnByAngle(angleOfBase); // turns robot to orientate it towards the base
    moveDistance(distanceToBase); // move robot by a specified distance
    
    // robot should arrive back where it started from
}