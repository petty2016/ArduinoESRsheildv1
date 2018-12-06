/*
 ESR_CODE
 Reads an analog input on pin 5, converts it to a Equivalent Series Resistance in ohms.
 Prints the result to a LCD.
 To be used with ESR Shield Rev 1 and Arduino Uno R3 with 1.1 volt internal reference.
 
 Arduino Pins Used:
     
    A0:  A0 is used by the SainSmart analog buttons. Since I am using the 1.1 volt internal reference for A5 the
         only buttons that will work are the UP, DOWN, and Right. They all call the Calibrate method.
    
    A5:  A5 is used to read the rectified voltage output by the second op amp.
    
    D2:  D2 is assigned as an external interrupt. This method sets a flag to display calibration detail on the lcd
         and can also be used to call the calibration method should an LCD be used that doesn't have buttons.

    The ESR Shield must be manually spanned using trim pots R9 and R5 such that the output ESRpin (A5) reads:
      Calibration:
      Leads shorted:  Volt should = 700 mV
      10 Ohms:        Volt should = 295 mV

      Calibration Procedure:
        Remove LCD shield if installed.
        Center the precision trim pots R5 (Attenuation), R6 (Frequency) and R9 (Load).
            
        Frequency Calibration:
            With the ESR shield installed on the Arduino Uno connect a DMM (set to Hz), Frequency Counter 
            or an Oscilloscope to pin 1 of the op amp on the ESR shield and adjust trim pot R6 to set the 
            desired frequency, typically 100khz.
        
        Voltage Output Calibration:    
            Short the ESR shield test leads together.
            Using a voltmeter, measure the DC voltage across pin A5 of the ESR shield and ground.
            Adjust trim pot R9 until the voltmeter reads 700 mV.
            Remove the test leads and put them across a tested 10 ohm 1% (or better) precision resistor.
            Adjust trim pot R5 until the voltmeter reads 295 mV.
            Short the leads together again and check the voltmeter.
            Adjust trim pot R9 until the voltmeter reads 700 mV.
            Continue going back and forth until the readings are stable @ 700mV shorted and 295mV @ 10 ohms.
            A few mV one way or the other is fine, it doesn't have to be perfect.
            We will trim it with the calibration factors, just get it close.
          
        Connect the LCD shield if installed.

 By Dennis Hill
 03/05/2013
 */

// include the library code:
#include <LiquidCrystal.h>

//  -= Using SainSmart LCD Shield =-
// LCD Pins RS, En, D4, D5, D6, D7
// Create an instance of the LiquidCrystal class and name it lcd.
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ESR Shield R1 uses the Arduino Uno analog pin 5 (019) to measure voltage 
// and then converts the measured value to resistance.
const int ESRpin = A5;            

// Analog pin 0 used by the SainSmart LCD shield buttons. We will use these to request
// calibration.
const int calPin = A0;

// This value will be used to set the base value of the ADC noise floor.
const byte noiseFloor = 50;

// This is the pin used to setup an external interrupt to poll the show 
// calibration request button on digital pin 2.
const int showCalValPin = 2;            

// This constant is used to convert the ADC value into voltage.
// This value is used when the ADC reference value is set to 1.1 volts.
// Change this value to fine tune your Arduino.
const float voltageStep = 0.00103;

// Variable used to hold value read from ADC.
unsigned  int adcVal = 0;        

// Variable used to accumulate ADC values.
unsigned long adcTemp = 0;           

// Variable used to hold calculated voltage value from ADC.
float voltage = 0;                    

// Variable to hold calculated resistance measurement derived from ADC value.
float ohms = 0;                       

// Variable used to hold the ADC calibration factor.
int adcCorrection = 0;            

// Interrupt variable used to signal calibration function is requested.
volatile boolean calFlag = false;     

// Interrupt variable used to signal the display of calibration values is requested.
volatile boolean showCalFlag = false; 

// Variable used to toggle display of the calibration values.
boolean showCal = false;

// Not implemented in this version.
// Variable to adjust LCD back-light intensity.
// byte BackLightPWM = 0;

// Measured ADC values at specified resistance.
const float _1ohm = 622; 
const float _2ohm = 569; 
const float _3ohm = 521; 
const float _4ohm = 478; 
const float _5ohm = 439; 
const float _6ohm = 403; 
const float _7ohm = 371; 
const float _8ohm = 341; 
const float _9ohm = 314; 
const float _10ohm = 290; 
const float _11ohm = 267; 
const float _12ohm = 246; 
const float _13ohm = 227; 
const float _14ohm = 210;
const float _15ohm = 193; 
const float _16ohm = 178; 
const float _17ohm = 164; 
const float _18ohm = 151; 
const float _19ohm = 139; 
const float _20ohm = 128; 
const float _21ohm = 118; 

/* -============================ SETUP ============================- */
void setup() {
    // Initialize the LCD; 16 Columns x 2 Lines.
    lcd.begin(16, 2);
    
    // Configure pin (A0) as an input for requesting calibration.
    pinMode(calPin, INPUT);	   
    
    // Turn on the internal pull-up resistor on pin (A0).
    digitalWrite(calPin, HIGH);
   
   // Run the method to initialize the pinChange interrupt in order to poll calPin (A0)
   // for calibration requests.   
   initLCDbuttonInterrupt();
   
   // Uno only!!!
   // Set the ADC voltage reference to the internal 1.1 volt reference.
   analogReference(INTERNAL);

   // Configure digital pin showCalValPin (2) as an input.
   pinMode(showCalValPin,INPUT);

   // Turn on the internal pull-up resistor on showCalValPin (2).
   digitalWrite(showCalValPin,HIGH);

   // Attach external interrupt 0 to showCalValPin (pin 2), trigger on the falling edge
   // and call the interruptESRshieldButton method when the event fires.
   attachInterrupt(0, interruptESRshieldButton, FALLING);
    
 }
/* -============================ end SETUP end ============================- */



/* -============================ LOOP ============================- */
void loop() {
    /*
    Purpose:
        Checks calFlag each time through and calls Calibrate routine if needed.
        Checks showCalFlag and toggles the display of calibration values if needed.
        Then reads 64 samples of the ADC on analog pin 5.
        Calculates a mean ADC value and determines the voltage.
        Calls the esrCalc function that interpolates the resistance from the ADC mean value.
    */

    if (calFlag)            // Check to see if user requested a calibration.
    {						// i.e., User pressed up button on LCD/Button shield.
        Calibrate();        // Call the Calibration procedure.
        calFlag = false;    // Reset the request flag.
    }
    
    if (showCalFlag)           // Check to see if user requested display of calibration values.
    {						   // User pressed the button on the ESR Shield.
        showCal = ~showCal;    // Toggle the display flag of the calibration values.
        showCalFlag = false;   // Reset the request flag.
        lcd.clear();           // Clear the text on the LCD.
    }
    
    adcTemp = 0;                       // Initialize the variable to hold the ADC sample values.
    for (int i=0; i <= 64; i++)        // Measure the voltage on the ESRpin 64 times.
    {
        adcVal = analogRead(ESRpin);   // Read voltage on analog pin 5
        adcTemp = adcTemp + adcVal;    // Save and accumulate sample.
        delay(5);                      // Added to stabilize the LCD display flicker
    }								   // and allow the ADC some recovery time to reduce noise.

    // The 64 sample values are shifted right 6 binary places which actually divides the accumulated
    // values by 64 giving us a mean value.
    adcVal = adcTemp >> 6;  // Divide accumulated sample value by 64, the number of sample loops.

    // If the resulting ADC value is above noiseFloor of the ADC, the correction factor will be added to
    // the mean ADC value. The correction factor may be either positive or negative.
    if (adcVal > noiseFloor) adcVal += adcCorrection;    

    // Calculate the voltage measurement from the mean ADC value.
    // 10 bit ADC has 1023 steps relative to a 1.1 volt reference.
    // 1.1 / 1023 = 0.0010752 which gives us a 1.0752mV resolution.
    voltage = adcVal * voltageStep;         // Convert the analog reading for Uno with 1.1V Ref
                                            // This is set to a custom value for my ADC.
                                            // Change voltageStep as needed.

    // Call the esrCalc function to calculate the resistance from the mean ADC value.
    ohms = esrCalc(adcVal);    

    // We can now display the results
    lcd.setCursor(0,0);                // Reset the cursor to the 1st column on the first line.
    lcd.print("ESR         ");         // Print identifier with spaces to clear previous values.
    lcd.setCursor(4,0);                // Then reset cursor to column 5, line 1, for the value.

    if (adcVal > noiseFloor)           // If the ADC mean value is greater than the noise floor 
    {                                  // we will print the resistance value.

		lcd.print(ohms,2);                 // Print the resistance value with 2 decimal place precision.
		lcd.print((char) 0xF4);            // Print the Omega character that is stored in the LCD ROM.
    }
    else lcd.print("OL");              // If we are below the noise floor of the ADC print the
                                       // Open Loop (OL) abbr.
   
   if (showCal)                        // Check the showCal flag and see if the user wants to view
   {	                               // the calibration values.
        lcd.setCursor(11,0);           // Set the cursor to column 12, line 1. 
        lcd.print("CF:");              // CF = ADC Correction Factor.
        lcd.print(adcCorrection);      // Print the ADC Correction Factor value.

        lcd.setCursor(0,1);            // Move the cursor to column 1, line 2.
        lcd.print("ADC:            "); // Print ADC with spaces to clear previous values.
        lcd.setCursor(5,1);            // Set cursor to column 6, line 1.
        lcd.print(adcVal);             // Print the ADC value.
        lcd.print("  ");               // Print a couple of spaces.
        lcd.print(voltage,3);          // Then print the calculated voltage with 3 decimal precision.
        lcd.print("V");                // Then print the identifier.
   }
   delay(50);						   // Just another wait state to let the LCD catch up.
}
/* -============================ end LOOP end ============================- */


/* -============================ Calibrate ============================- */
void Calibrate(void)
{
    /*
    PURPOSE:
    Calibrates the ADC by reading in 64 samples, averaging them, and then subtracting the measured value
    from 680 which is the target ADC value for zero resistance.

    NOTE:
    The test leads on the ESR shield must be shorted together in order to preform this procedure.
    */

    unsigned int adcVal;            // Variable used to hold value read from ADC.
    unsigned long tempADCval = 0;   // Variable used to accumulate ADC values.
    
    lcd.clear();                    // Clears the LCD display.
    lcd.print(" CALIBRATING ");     // Inform user calibration is in progress.
    
    for (int i=0; i <= 64; i++)            // Sample ESRpin 64 times.
    {
        adcVal = analogRead(ESRpin);       //Read voltage on analog pin 5
        tempADCval = tempADCval + adcVal;  // Save and accumulate sample.
        delay(5);                          // Added to stabilize the LCD display flicker
    }                                      // and allow the ADC some recovery time to reduce noise.

    adcVal =  tempADCval >> 6;		 // Divide accumulated sample value by 64, the number of sample loops.

    adcCorrection = 680 - adcVal;    // 680 is the target value for the ADC (700 mV)
                                     // Correction factor is the difference
    delay(500);        // Wait about half a second so the user can see the calibrating text on the display.
    lcd.clear();       // Then clear the LCD display.

}
/* -============================ end Calibrate end ============================- */


/* -============================ esrCalc ============================- */
float esrCalc(unsigned int adc)
{
    /*
    PURPOSE:
    Interpolates resistance value from input ADC measurement. Resistance values were measured through resistors
    at assumed values of 1 through 20 ohms at 1 ohm intervals.

     */
	if (adc > 621) return mapf(adc, 680, _1ohm, 0, 1);		// 0 - 1 Ohms

	else if (adc > 568) return mapf(adc, _1ohm, _2ohm, 1, 2);	// 1 - 2 Ohms
	else if (adc > 520) return mapf(adc, _2ohm, _3ohm, 2, 3);	// 2 - 3 Ohms
	else if (adc > 477) return mapf(adc, _3ohm, _4ohm, 3, 4);	// 3 - 4 Ohms
	else if (adc > 438) return mapf(adc, _4ohm, _5ohm, 4, 5);	// 4 - 5 Ohms
	else if (adc > 402) return mapf(adc, _5ohm, _6ohm, 5, 6);	// 5 - 6 Ohms
	else if (adc > 370) return mapf(adc, _6ohm, _7ohm, 6, 7);	// 6 - 7 Ohms
	else if (adc > 340) return mapf(adc, _7ohm, _8ohm, 7, 8);	// 7 - 8 Ohms
	else if (adc > 313) return mapf(adc, _8ohm, _9ohm, 8, 9);	// 8 - 9 Ohms
	else if (adc > 289) return mapf(adc, _9ohm, _10ohm, 9, 10);	// 9 - 10 Ohms
	else if (adc > 266) return mapf(adc, _10ohm, _11ohm, 10, 11);	// 10 - 11 Ohms
	else if (adc > 245) return mapf(adc, _11ohm, _12ohm, 11, 12);	// 11 - 12 Ohms
	else if (adc > 226) return mapf(adc, _12ohm, _13ohm, 12, 13);	// 12 - 13 Ohms
	else if (adc > 209) return mapf(adc, _13ohm, _14ohm, 13, 14);	// 13 - 14 Ohms
	else if (adc > 192) return mapf(adc, _14ohm, _15ohm, 14, 15); // 14 - 15 Ohms
	else if (adc > 177) return mapf(adc, _15ohm, _16ohm, 15, 16);	// 15 - 16 Ohms
	else if (adc > 163) return mapf(adc, _16ohm, _17ohm, 16, 17); // 16 - 17 Ohms
	else if (adc > 150) return mapf(adc, _17ohm, _18ohm, 17, 18); // 17 - 18 Ohms
	else if (adc > 138) return mapf(adc, _18ohm, _19ohm, 18, 19); // 18 - 19 Ohms
	else if (adc > 127) return mapf(adc, _19ohm, _20ohm, 19, 20); // 19 - 20 Ohms
	else return mapf(adc, _20ohm, _21ohm, 20, 21); // 20 Ohms to Noise Floor.

}
/* -============================ end esrCalc end ============================- */


/* -============================ esrCalc ============================- */
    /*
    PURPOSE:
	Just a version that accepts floating number values instead of the Arduino map function that uses long integers.
	*/
float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/* -============================ esrCalc ============================- */



/* -============================ ***INTERRUPT*** ============================- */

/* 
 -============================ interruptESRshieldButton ============================- 
 This is the external interrupt handler for external interrupt 0 on digital pin 2.
 This procedure was assigned to interrupt 0 in the setup routine by the 
 "attachInterrupt(0, interruptESRshieldButton, FALLING)" method.
 This is the interrupt routine is called when the button on the ESR shield is pressed.
*/
void interruptESRshieldButton(void)        
{
    showCalFlag = true;  // Set the show calibration values request flag.
    //calFlag = true;    // This can also be used to request calibration for LCD shields without buttons.
}
/* -============================ end interruptESRshieldButton end ============================- */


/* 
 -============================ initLCDbuttonInterrupt ============================- 
 This method initializes the Pin Change Interrupt -=ISR(PCINT1)=-.
 This method is called only once by the setup routine.
 We will use this polling method to detect when the SainSmart LCD shield buttons are pressed.
 The LCD shield buttons are connected to analog pin 0 on the Arduino. When pressed, the buttons
 pull-down the voltage on analog pin 0 by connecting the other side of the button switch to ground
 through a resistor. The button pressed can then be identified by the voltage read on analog pin 0.
 Using the 1.1 volt ADC internal reference for this project limits the buttons that can be read.
 The UP, DOWN and RIGHT buttons are the only buttons detected as a result. Any of which will 
 fire the pin change event and set the calibration request flag, calFlag.
 */
void initLCDbuttonInterrupt(){
  cli();		// Switch interrupts off while messing with their settings.  
  PCICR =0x02;          // Enable PCINT1 interrupt.
  PCMSK1 = 0b00000001;  // Set the calPin (A0) Pin Change enable bit.
  sei();		// Turn interrupts back on.
}
/* -============================ end initLCDbuttonInterrupt end ============================- */

/* 
 -============================ ***ISR(PCINT1_vect)*** ============================- 
 This is the Pin Change Interrupt Handler Method for -=ISR(PCINT1)=-.
 This method is called when one of LCD shield UP, DOWN or Right buttons are pressed.
 This method simply sets a flag that is checked in the main loop to determine if calibration is requested.
 */
ISR(PCINT1_vect) {    // Interrupt service routine.
	calFlag = true;
}
/* -============================ end --ISR(PCINT1_vect)-- end ============================- */

/* -============================ end ***INTERRUPT*** end ============================- */

//END ESR_CodeV1





