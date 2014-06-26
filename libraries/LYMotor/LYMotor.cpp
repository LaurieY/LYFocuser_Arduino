//LYMotor using Pololu A4983 Stepper Motor Driver Carrier with Voltage Regulators 
//But based on the 
// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

// Modded June 16 2014 to add delay after enabling & make the step pulse 10microsecs only

#include <avr/io.h>
//#include "WProgram.h"
 #if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
#include "LYMotor.h"

static uint8_t latch_state;





/******************************************
               MOTORS
******************************************/


/******************************************
               STEPPERS
******************************************/
void LY_Stepper::init(uint16_t steps) {
 // delay(5);

	// MC.enable();
#ifdef MOTORDEBUG
 //   Serial.print("In LY_Stepper ");
#endif


 
   /* latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B) &
      ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // all motor pins to 0
    MC.latch_tx();*/
    
    // enable both H bridges
  /**  pinMode(11, OUTPUT);
    pinMode(3, OUTPUT);
    digitalWrite(11, HIGH);
    digitalWrite(3, HIGH);
	**/
	//Enable A4983
	  revsteps = steps;
  // usteps= 8;
  //steppernum = num;
 // currentstep = 0;
	
pinMode(MOTORNOTENABLE,OUTPUT);
   digitalWrite(MOTORNOTENABLE, HIGH); 

	pinMode(MOTORSTEP,OUTPUT);
	digitalWrite(MOTORSTEP,LOW);
		pinMode(MOTORDIR,OUTPUT);
	digitalWrite(MOTORDIR,LOW);
pinMode(MOTORDIR,OUTPUT);
  	pinMode(MOTORNOTSLEEP,OUTPUT);
   digitalWrite(MOTORNOTSLEEP, HIGH);
      	pinMode(MOTORNOTRESET,OUTPUT);
   digitalWrite(MOTORNOTRESET, HIGH);
   //NOW SET Microstep to Full
   pinMode(MOTORMS1,OUTPUT);
   digitalWrite(MOTORMS1, LOW);  
  pinMode(MOTORMS2,OUTPUT);
   digitalWrite(MOTORMS2, LOW);
   pinMode(MOTORMS3,OUTPUT);
   digitalWrite(MOTORMS3, LOW);  
 
   
 //usteps= 8;
 // currentstep = 0;
 //  setSpeed(600); //DEFAULT
//   setMicroSteps(8); //DEFAULT
   
   // NOW set the default sense of the directions FORWARD and REVERSE via FORWARDLEVEL 0 and BACKWARDLEVEL 1
forward_Level = FORWARDLEVEL;
 backward_Level = BACKWARDLEVEL;
}
LY_Stepper::LY_Stepper(uint16_t steps,uint16_t motor ) {
init(steps);
}
LY_Stepper::LY_Stepper(uint16_t steps) { // steps is basic # of full steps
init(steps); 
 }

void LY_Stepper::enable( ) { //default setup of stepper motor, sets speed to 60 and microsteps to 8
}

 void LY_Stepper::setMicroSteps(uint16_t micsteps){
 //set MS1,MS2,MS3 for the microstep values of Full, Half, Quarter,Eighth, Sixteen, given as 1,2,4,8,16
 digitalWrite(MOTORMS1, LOW);
  digitalWrite(MOTORMS2, LOW);
  digitalWrite(MOTORMS3, LOW);
  usteps= micsteps;
  //Serial.print("microstep (usteps)value = = : "); Serial.println(usteps);
 switch (usteps) {
    case 1:
       break;
	   case 2:
	   digitalWrite(MOTORMS1, HIGH);
	   break;
	   case 4:
	   digitalWrite(MOTORMS2, HIGH);
	   break;
	   case 8:
	   digitalWrite(MOTORMS1, HIGH);
	   digitalWrite(MOTORMS2, HIGH);
	   break;
	   case 16:
	   digitalWrite(MOTORMS1, HIGH);
	   digitalWrite(MOTORMS2, HIGH);
	   digitalWrite(MOTORMS3, HIGH);
	   break;

 }
 }
void LY_Stepper::setSpeed(uint16_t rpm) {
//unsigned long fred = (long) revsteps * (long) rpm;
//Serial.print("steps per min = : "); Serial.println(fred);
  usperstep = (unsigned long) 60000000 / (( long)revsteps * (long) rpm* usteps); //ignoring microstepp settings  valu is in microseconds
//  Serial.print("rev step set to: "); Serial.println(revsteps, DEC);
 //  Serial.print("Microsteps (usteps) set to: "); Serial.println(usteps, DEC);
 //   Serial.print("rpm set to: "); Serial.println(rpm, DEC);
 // Serial.print("current step (usperstep) set to: "); Serial.println(usperstep);
 // Serial.println("---");

//delay(3000);
  steppingcounter = 0;  // an absolute counter of position
 //  digitalWrite(MOTORNOTENABLE,LOW); // ?June 2014 Why enable NOW?
}

void LY_Stepper::release(void) {
 digitalWrite(MOTORNOTENABLE, HIGH);
  //digitalWrite(MOTORNOTSLEEP, LOW);
  digitalWrite(MOTORNOTSLEEP, HIGH); // just use the enable pin
   delay(2);
}
void LY_Stepper::unrelease(void) {
 digitalWrite(MOTORNOTENABLE, LOW);
  digitalWrite(MOTORNOTSLEEP, HIGH);
  delay(2);
 
}
void LY_Stepper::swapDirection(uint8_t dir)  {
// alters the sense of forward & reverse
// a supplied value of 0 sets the sense back to default as per #define FORWARDLEVEL 0 & #define BACKWARDLEVEL 1
// the default is set at initialisation
//a supplied value of 1 reverses these for the duration of the run

if (dir ==0)
{forward_Level = FORWARDLEVEL;
 backward_Level = BACKWARDLEVEL;
} else {
forward_Level = BACKWARDLEVEL;
 backward_Level = FORWARDLEVEL;
}
}
void LY_Stepper::step(uint16_t steps, uint8_t dir,  uint8_t delay_mult) { // 
// Step number of microsteps or steps using the microstep value of 8
//Serial.print("current step STILL: "); Serial.println(usperstep);
//Serial.print("microsteps (style) : "); Serial.println(style);
//setMicroSteps(style);
//enable();
  uint32_t uspers = usperstep;
 //Serial.print("uspers step : "); Serial.println(uspers);
 //uspers=2000;
  uint8_t ret = 0;


  /**********
#ifdef MOTORDEBUG
 //   Serial.print("steps = "); Serial.println(steps, DEC);
#endif
 
 ********/
 //Serial.print("stepsize = "); Serial.println(usperstep, DEC);
  while (steps--) {
    ret = onestep(dir,  delay_mult);
   delayMicroseconds(delay_mult*uspers/2); // in micros half a cycle, the other half was inside onestep
//	delay(uspers/2000);
  //  steppingcounter += (uspers % 1000);
  //  if (steppingcounter >= 1000) {
   //   delay(1);
  //    steppingcounter -= 1000;
  //  }
  }
 
}

void LY_Stepper::step(uint16_t steps, uint8_t dir) {
//uint8_t micsteps = usteps;
step(steps, dir, 1);
}

uint8_t LY_Stepper::onestep(uint8_t dir, uint8_t delay_mult) {  // delay_mult, default 1 multiply delay between step action by this to slow down moves

  uint32_t uspers = usperstep;
   digitalWrite(MOTORNOTENABLE, LOW);
  digitalWrite(MOTORNOTSLEEP, HIGH);
 
 //uspers=2000;
  // next determine what sort of stepping procedure we're up to
 


    if (dir == FORWARD) {
	digitalWrite(MOTORDIR,forward_Level);
      currentstep++;
    } else {
      // BACKWARDS
	  	digitalWrite(MOTORDIR,backward_Level);
      currentstep--;
    }

 delayMicroseconds(2); //short delay after setting direction


#ifdef MOTORDEBUG
 // Serial.print("current step time: "); Serial.println(uspers, DEC);
 //Serial.print("current delay: "); Serial.println(uspers/2000, DEC);
#endif


//Serial.print("current uspers: "); Serial.println(uspers, DEC);
//Serial.print("current delay_mult: "); Serial.println(delay_mult, DEC);

digitalWrite(MOTORSTEP,HIGH);
//delayMicroseconds(delay_mult*uspers/2); // wait for half of the cycle time in microseconds so min value is 2 msecs
//getting overflow change to a delay call 
delayMicroseconds(10); // long enough, its the leading edge that's the step trigger
unsigned long freddy= delay_mult*uspers/(2*1000);
//Serial.print("freddy size : "); Serial.println(freddy, DEC);
//delay((unsigned long) delay_mult*uspers/(2*1000));

//	delay(uspers/2000);
digitalWrite(MOTORSTEP,LOW);
delay(freddy);  //add the speed/performance delay AFTER the end of the STEP

  //Serial.println(step, DEC);

  return currentstep;
}
 

