


/* 
 * Source for an Arduino+Adafruit motorshield based 
 * focus controller.
 *
 * $Source: \\orion\users\sander\cvsroot/open_focuser/open_focuser/open_focuser.pde,v $
 * $Revision: 1.2 $
 * $Date: 2011/03/03 23:15:49 $
 */
 /* Modded by LaurieY to replace AFMotor with LYMotor
  * Source for an Arduino+pololu stepper driver based focus controller.
  * $Source: "\electronics\Arduino\Projects\focuser\LaurieY-OpenFocuser\open_focuser_LY\open_focuser_LY.pde" $
 * $Revision: 1.3 $
 * $Date: 2011/03/15 21:22:49 $
 
 **  June 15 2014
 Included commands c rpm  sets rpm and
 and 
 d delay_mult for fast moves ( can be used to slow down the fast moves ) 1 is fast 8 is the same as the slowdown speeds)
 Both are saved in eeprom
 
 ** also fixed a few bits where comparisons were between int and unsigned ints
 
 Fixed the step pulse width and delay between pulses where the slow (long delay) pulses were not really slow because of overflow in the delayto microseconds call
 Changed it to delay  (in milliseconds) not as accurate but not important
 *  Changed move_focuser to be quicker by stepping by unints of 800
2011-April-15
Changed to receive and transmit across serial interface steps in 1/200th  so for a MICROSTEP of 8 need to mult the requested difference  by 8 
  
  Changed to slow down when within 1/4 turn of the target.
  23 June 2011 added simple button controls
  Initially just keep moving by 5 steps while the button is pressed
2011-June 24
Update to provide a mechanism to upload startup params for eg, move rates, move direction, slow button move rates & others
Use code w - for upload followed by a number for the swap direction 
Code b for button move steps ( more steps faster) default 10 need a timer
& then I can implement a release motor after no activity timer set inactivity to >3 secs
activity timer record the time for any move action, speed up on button moves set to >2 secs

May 2012
Added a serial command k nnnn
forces the position and target position to be a value send from the driver.
Used to set the position positive when switched on, needed since I don't save the position in EEprom between sessions 
& since I remove the camera it isn't going to be where I left it


June 2012, reverse Fwd & Back settings 
alter the sensitivity.  Now all moves are in microsteps 
alter the sensitivity.  Now all moves are in microsteps 
alter the sensitivity.  Now all moves are in microsteps 
alter the sensitivity.  Now all moves are in microsteps 
with default microstepping of 8 there are now 1600 steps per rev

Also uncommented the EEPROM code & changed things so it only writes to EEPROM wome time after a move & only once
On init EEPROM position is read into the position variable, so it remembers it position over sessions

July 2012
Mod to ensure that the position count does not get > 32k (32768)
July 0 2012

Change back to single stepping  ie 1/200 rev per step, 
because unless I keep the motor energised the stepper falls back to the nearest full step 
when I re-energise so the step count becomes incorrect
This gives me about 5mm per full rev of 200 steps so about 25 microns per step
So for autofocus I should use 2 -> 4 steps per exposure with a focus zone of about 100 microns

Aug 2012 
Added a command 'e' to set the time to retain energise, so I can trial the potential to keep motor energinsed
**/

#include <LYMotor.h>
//#include <stdarg.h>
void p(char *fmt, ... ){
        char tmp[128]; // resulting string limited to 128 chars
        va_list args;
        va_start (args, fmt );
        vsnprintf(tmp, 128, fmt, args);
        va_end (args);
        Serial.println(tmp);
}
#include <EEPROM.h>
// LEY  **** */
#include <avr/eeprom.h> 



uint16_t position_e_a __attribute__((section(".eeprom"))) = 0;
uint16_t rpm_e_a __attribute__((section(".eeprom"))) = 4;
uint16_t init_e_a __attribute__((section(".eeprom"))) = 2;
uint16_t fast_delay_mult_e_a __attribute__((section(".eeprom"))) = 6;

#define INIT_EEPROM_VALUE  0xFFFF // 0x3287

#define MAXLEN 50
#define BLACKBUTTON 18 // manual button controls
#define REDBUTTON 19
#define BUTTONMOVEVALUE 1 // how much to move for a button press
char inByte = 0;         // incoming serial byte
int index = 0;
char  in_buffer[MAXLEN];
int  temperature = 234;
long	position = 4000;
long ee_position = position;  // the last value to be written to EEPROM
int current_black,current_red =1;
long	target_position = 0;
 long target_difference =0;
 unsigned long moveTime, buttonTime;
 unsigned long energiseTime = 3;
 boolean buttonPressed=false;
 unsigned int stepsize=1;
int stop = 0;
int currentButtonMoveValue = BUTTONMOVEVALUE;
int defaultButtonMoveValue = BUTTONMOVEVALUE;
int defaultButtonMoveSpeed = 120; // RPM for button fast moves
int currentMoveSpeed =2;
int buttonSpeedupValue = 3;
int fast_delay_mult =1;

int temp1;




#define VER "LY Open_Focuser_1.02 Apr 2014"
#define LY_MIC 1 // microsteps are 8 
//#define LY_MIC_MOVE 2 // Not used
#define LY_STEPS_REV 200  //steps per rev
#define LY_STEPS_SLOW_REV LY_STEPS_REV/10 // keep slow for only 20 steps
#define RPM 20
int rpm = RPM;
LY_Stepper motor(LY_STEPS_REV);


void setup()
{delay(1000);
	unsigned int temp;
	// start serial port at 9600 bps:
	Serial.begin(115200);
	delay(500);
//	Serial.println(VER);
motor.enable();
motor.setMicroSteps(LY_MIC);

	//motor.release();
  position=4000;
rpm = RPM;
temp = eeprom_read_word(&init_e_a);
	if (temp != INIT_EEPROM_VALUE) {
		eeprom_write_word(&init_e_a, INIT_EEPROM_VALUE);
		Serial.println("Position initialized");
                ee_position = position;
		write_status();
	} else {
		position=eeprom_read_word(&position_e_a);
   /*              if ((temp >32767) or (temp <-32768) ){temp=position;}
                 else
                 {position=temp;}  */
		target_position = position; // no movement right out of the gates
		ee_position = position;
			while (!eeprom_is_ready());
		  cli();
		rpm = eeprom_read_word((uint16_t*)rpm_e_a);
			sei();
			while (!eeprom_is_ready());
		  cli();
		fast_delay_mult= eeprom_read_word((uint16_t*)fast_delay_mult_e_a);
		sei();
// report eeprom values while initialising 
 p("Starting position = %i", position);
  p("Starting rpm = %i", rpm);
   p("Starting fast_delay_mult = %i", fast_delay_mult);
	}
/****/

motor.setSpeed(rpm);
currentMoveSpeed =rpm;
  target_position = position; // no movement right out of the gates
    pinMode(BLACKBUTTON, INPUT);
    pinMode(REDBUTTON, INPUT);
    digitalWrite(BLACKBUTTON, HIGH); //turn on 20k pullup resistors to simplify switch input
    digitalWrite(REDBUTTON, HIGH);
	Serial.println("Ready");
	//p("PPP READSY");
}

void loop()
{	 if ((millis() - moveTime)/1000 >10) {
	if (ee_position != position ) {write_status();
				//p("updated eeprom to in loop %d",position);
		ee_position = position;
	}}// only write to EEPROM if position has changed and at after 10 secs
	
	// if we get a valid byte, read analog ins:
	if (Serial.available() > 0) {
		// get incoming byte:
		inByte = Serial.read();
		in_buffer[index++] = inByte;
		if (inByte == 10 || index == MAXLEN) {
			// LF read
			// chop CR LF off the command string
			in_buffer[index - 2] = 0;
			//Serial.print('>');
			//Serial.print(in_buffer);
			//Serial.println('<');
			process_cmd(in_buffer);
			// go again
			index = 0;
		}

} else  {
  // ********************nothing incoming from serial so check buttons
  //****************  BUTTONS  ***********************
  
        current_black = digitalRead(BLACKBUTTON);
		
        current_red = digitalRead(REDBUTTON);
		//p("Current RED= %d ",current_red);
        if (current_black!=current_red) {  //one button is pressed, they are not equal
       if (!buttonPressed)  { buttonTime=millis(); //mark 1st time button press is seen
       buttonPressed=true;
        currentButtonMoveValue = defaultButtonMoveValue;
     //p("button 1st pressed at %d",buttonTime);
       }
       else {//button was already pressed so look at how long & increase speed if >3 secs
       
      
      //p(" button already pressed and elapsed time is %d - %d",millis(),buttonTime);
         if (((millis()-buttonTime)/1000)>2) {currentButtonMoveValue = defaultButtonMoveValue*100; // 30 step moves ensures all fast button moves would be fast
									currentMoveSpeed = defaultButtonMoveSpeed; //move at 120 RPM
									 //p("Upping motorspeed to %d",currentMoveSpeed); 
									 //p("currentMoveValue= %d",currentButtonMoveValue);
									motor.setSpeed(currentMoveSpeed); }
         }
        if (current_black ==0) { //black button move  increase position by BUTTONMOVEVALUE
                target_position = position+currentButtonMoveValue;
                 stop=0; // gets the move actioned
              } else {//red button move RED is REVERSE  ie reduce position
    target_position = position-currentButtonMoveValue;
    stop=0;
              }
              while (  target_position != position) {
              move_focuser(currentButtonMoveValue);

                delay(10);
              }
  delay(100);
  //buttonTime=millis();
        }else {buttonPressed = false;
		if (currentMoveSpeed !=rpm) motor.setSpeed(rpm); // reset speed to normal if had been changed
		}
        }
        
 //*************   END OF BUTTONS  *******************
	// read temperature
	// adjust focuser if needed based on temp
	// nothing to read, see if the focuser needs to be moved
	move_focuser(1);
}

void process_cmd(char* cmd) {
	if (strcmp(cmd, "p") == 0) {
		// get position
		Serial.print("P ");

		Serial.println(position,DEC);
	} else if (strncmp(cmd, "m", 1) == 0) {
		// move
		//p(" In MOVE");
		
		 long temp = atol(cmd+2);
		Serial.print("M ");
		Serial.println(temp);
		target_position = temp;
		//	p("start time in msec %u",millis());
          stop=0; //LEY
	} else if (strncmp(cmd, "w ", 2) == 0) {
		// ** swap direction **
          // first is direction swap normal  =0, set to 1 for opposite, 
              // then button move speed, then button move delay in secs before speedup
		 char* u1;   
                u1= strtok(cmd+2," ,:");
		p("W %s ",u1);
                 motor.swapDirection(atoi(u1));

	}else if (strncmp(cmd, "b ", 2) == 0) {
	        // then button move RPM, 
		 char* u2;   
                u2= strtok(cmd+2," ,:");
		p("B %s ",u2);
                defaultButtonMoveValue = atoi(u2);

	}
else if (strncmp(cmd, "y ", 2) == 0) {
	        //  button speedup  delay in secs before speedup needs timer sorting 
//**** NOT implemented ***
		 char* u3;   
                u3= strtok(cmd+2," ,:");
		p("Y %s ",u3);
                buttonSpeedupValue = atoi(u3);

	}



else if (strcmp(cmd, "i") == 0) {
		// is focuser moving?
		Serial.print("I ");
		if (stop == 1 || (target_position == position))
			// no
			Serial.println(0);
		else
			// yes
			Serial.println(1);
	} else if (strcmp(cmd, "t") == 0) {
		// get temperature
		Serial.print("T ");
		Serial.println(temperature);
	} else if (strcmp(cmd, "a") == 0) {
		// get target position
		Serial.print("A ");
		Serial.println(target_position);
	} else if (strcmp(cmd, "h") == 0) {
		// halt (ASCOM style)i
		Serial.println("H");
		target_position = position;
	} else if (strcmp(cmd, "r") == 0) {
		// release
		Serial.println("R");
		stop = 2;
	} else if (strcmp(cmd, "s") == 0) {
		// stop
		Serial.println("S");
		stop = 1;
	} else if (strcmp(cmd, "v") == 0) {
		Serial.println(VER);
	} else if (strcmp(cmd, "z") == 0) {
		// set current as zero, resets target to zero as well
		Serial.println("Z");
		target_position = 0;
		position = 0;
		//write_status();
	} else if (strncmp(cmd, "k ", 2) == 0){ 
		// set current as a value, default 10k, resets target to 10k as well
		// allows for position to be always positive if switched on at focus position
		Serial.print("K ");
		long setPos = atol(cmd+2);

		Serial.println(setPos);
		target_position = setPos;
		position = setPos;
		//write_status();
	}else if (strncmp(cmd, "c ", 2) == 0){   		// sets rpm as value
		
		Serial.print("C ");
		rpm  = atoi(cmd+2);

		Serial.println(rpm);
		 while (!eeprom_is_ready());
		  cli();
			if(eeprom_read_word((uint16_t*)rpm_e_a) != rpm) {
			eeprom_write_word((uint16_t*)rpm_e_a, rpm);
			}
			sei();
//		eeprom_write_word(&rpm_e_a, (long)rpm);
	p("set rpm  to %i", rpm);
	motor.setSpeed(rpm);
		//write_status();
	}
	else if (strncmp(cmd, "d ", 2) == 0){ 		
	// sets fast delay_mult.  Can set to 8 to make all moves slower as value, slow moves always use 8, the fast did default to 1
		
		Serial.print("D ");
		fast_delay_mult  = atoi(cmd+2);
			Serial.println(fast_delay_mult);
			while (!eeprom_is_ready());
		  cli();
		  temp1= eeprom_read_word((uint16_t*)fast_delay_mult_e_a);
			if(temp1 != fast_delay_mult) {
			eeprom_write_word((uint16_t*)fast_delay_mult_e_a, fast_delay_mult);
			}
			sei();
	//	eeprom_write_word(&fast_delay_mult_e_a, (long)fast_delay_mult);
	//p("temp value from eeprom read %i", temp1);
		p("set fast_delay_mult to %i", fast_delay_mult);
		
		
		//write_status();
	}
	else if (strncmp(cmd, "e ", 2) == 0){ 
		// set time to remain energized in seconds (defalt stays at 3

		Serial.print("E ");
	        energiseTime = atol(cmd+2);

		Serial.println(energiseTime);

		//write_status();
	} else {
		Serial.print('>');
		Serial.print(cmd);
		Serial.println("< not understood.");
	}
}

void move_focuser(int moveSteps) {
	if (stop == 1) {
		motor.release();
		return;
	}
if (stop ==2) {
  motor.release();
  return;
}
//moveSteps=1;

stepsize=moveSteps;
// Make moves in microsteps units
// if move difference < 1/4 turn then do it slowly at 1/10 the normal RPM rate do this by adding delays between each step
if (target_position != position) {

motor.unrelease(); // allows motor to be energized for 3 secs after the move (for extra stability without heating
target_difference = target_position - position;
int delay_mult =fast_delay_mult;
if (abs(target_difference) <= LY_STEPS_SLOW_REV)  delay_mult =8;// button move fast is  100 steps so would be fast!!!

//p("Position %d",position);
//p("Target Position %d",target_position);

 /* Serial.print("Position ");
Serial.println(position);
  Serial.print("Target Position ");
Serial.println(target_position);
  Serial.print("Target Diff ");
Serial.println(target_difference);*/
 
//int one_second_steps=RPM*LY_STEPS_REV/(60);
 /*Serial.print("one second steps ");
Serial.println(one_second_steps, DEC);*/

 //if (abs(target_difference) >one_second_steps)  stepsize=LY_STEPS_HALF_REV*LY_MIC;
	if (target_difference > 0) {

/*Serial.print("Stepsize ");
Serial.println(stepsize);
*/


    
		position = position+(stepsize);
		motor.step(stepsize, FORWARD, delay_mult);
// p("step OUT to : %d",position);
//p("step delaymult : %d",delay_mult);
		//write_status();
	} else if (target_difference<0) {

		position = position-(stepsize);
		motor.step(stepsize, BACKWARD, delay_mult);
 //p("step IN  = : %",position);
		//write_status(); 
}
moveTime = millis();  // record the last time a move was made so that after a period the motor can be released

//write_status(); //only write position to EEprom at end of move
	}
else {//release if no action for 3 secs
		if ((millis() - moveTime)/1000 >=energiseTime) {
			// p("end time () in msec %u",millis());
  		motor.release(); 
	                } else {
	               
                }
 /*               if ((millis() - moveTime)/1000 >10) {
		if (ee_position != position ) {write_status(); 
	//		Serial.print("updated eeprom to ");
					
	//				Serial.println(position);
		ee_position = position;
		}// only write to EEPROM if position has changed and at after 10 secs

	}*/
  }
}



void write_status() {
cli();
	 
eeprom_write_word(&position_e_a, position);
//p(" writing to EEPROM: %p ",position);
sei();
}


