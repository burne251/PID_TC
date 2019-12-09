/*	$TC_PID: main.ino,v 1.013 2019/12/08 20:08:59 burne251 Exp $	*/

/*
 * Copyright (c) 2019 Charlie Burnett <burne251@umn.edu>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */


#include <Adafruit_MAX31856.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <PID_v1.h>

/* definitions of pins for readability's sake */
#define CS_PIN 10
#define DI_PIN 11
#define DO_PIN 12
#define CLK_PIN 13
#define LED_PIN 3
#define BUTTON_PIN 2
#define LIGHT_PIN 5
#define FAN_PIN 6
/* define the time for the heating element to cycle, and the temp setpoint */
#define CYCLE_TIME_MS 1000
#define THERMO_SETPOINT 25
/* 
 * Experimentally determined optimal gain values with Ziegler-Nicholls method. 
 * Will vary from setup to setup.
 */
#define KP_GAIN 9
#define KI_GAIN 3.6
#define KD_GAIN 5.625
/* 
 * Recommended to keep the baud rate at 9600, thermocouple has weird issues 
 * otherwise that I don't really have time to figure out admittedly.
 */
#define BAUD_RATE 9600

static void control();

/* Device setup */
Adafruit_MAX31856 thermocouple = Adafruit_MAX31856(CS_PIN, DI_PIN, DO_PIN, 
    						   CLK_PIN);
Adafruit_7segment matrix = Adafruit_7segment();
double pv, setpoint, output;
/* Initializes PID with defined gain values */
PID bulbcontrol(&pv, &output, &setpoint, KP_GAIN, KI_GAIN, KD_GAIN, DIRECT);

void 
setup() 
{
	/* Sets baud rate to 9600, thermocouple acts up at higher rates */
	Serial.begin(9600);
	
	/* Set pins to desired modes */
	pinMode(LED_PIN, OUTPUT);
	pinMode(BUTTON_PIN, INPUT);
	pinMode(LIGHT_PIN, OUTPUT);
	pinMode(FAN_PIN, OUTPUT);

	/* Set target value for PID to indicated degrees Celsius */
	setpoint = THERMO_SETPOINT;
  	
	/* 
	 * Initialize thermocouple, declare type (should work for other 
	 * thermocouples using the MAX31856.
	 */
	thermocouple.begin();
	thermocouple.setThermocoupleType(MAX31856_TCTYPE_E);
	/* Set PID loop to automatic control */
	bulbcontrol.SetMode(AUTOMATIC);
  
	/* Initialize LED Matrix */
	matrix.begin(0x70);
}

void 
loop() 
{
	/*
	 *  Required "Watchdog LED" loop, exited by activating the button. 
	 *  Flashes LED on for half a second, then off for half a second, and 
	 *  checks for a button press at each loop. Turns light/fan off too.
	 */
	while (!digitalRead(BUTTON_PIN)){
		digitalWrite(FAN_PIN, HIGH);
		digitalWrite(LIGHT_PIN, HIGH);
		digitalWrite(LED_PIN, HIGH);
		delay(500);
		digitalWrite(LED_PIN, LOW);
		delay(500); 
	}

	/* When kicked out of loop, turns on cooler and turns off LED */
	digitalWrite(FAN_PIN, LOW);
	digitalWrite(LED_PIN, HIGH);
	
	for(int i = 0; i < 60; i++){  
		/* 
		 * Shoots off an error message to any attached serial monitors
		 * in the event of a fault.
		 */
		uint8_t fault = thermocouple.readFault();
		if (fault) {
    			if (fault & MAX31856_FAULT_CJRANGE) 
				Serial.println("Cold Junction Range Fault");
    			if (fault & MAX31856_FAULT_TCRANGE) 
				Serial.println("Thermocouple Range Fault");
    			if (fault & MAX31856_FAULT_CJHIGH)  
				Serial.println("Cold Junction High Fault");
    			if (fault & MAX31856_FAULT_CJLOW)
				Serial.println("Cold Junction Low Fault");
    			if (fault & MAX31856_FAULT_TCHIGH)  
				Serial.println("Thermocouple High Fault");
    			if (fault & MAX31856_FAULT_TCLOW)   
				Serial.println("Thermocouple Low Fault");
    			if (fault & MAX31856_FAULT_OVUV)    
				Serial.println("Over/Under Voltage Fault");
    			if (fault & MAX31856_FAULT_OPEN)    
				Serial.println("Thermocouple Open Fault");
  		} else {
			/* 
			 * Assigns current thermo reading to 
			 * global process variable value. 
			 */
			pv = thermocouple.readThermocoupleTemperature();
			
			/* PID magic determines output value */
    			bulbcontrol.Compute();
			
			/* control() applies the output to the heater */
    			control();
			
			/* Write to display LED matrix */
        		matrix.print(pv);
	    		matrix.writeDisplay();
		
			/* 
			 * Debugging stuff, outputs output and PV to a serial 
			 * monitor 
			 */
			Serial.println(output);
		    	Serial.print("Thermocouple Temp: "); 
    			Serial.println(pv);
		}
	}
  
}

static void 
control()
{
	/* 
	 * Divides output by output range of the PID loop. The defined cycle 
	 * time is then multiplied by this value. Arduino will kick on the 
	 * heating element for the calculated value, and then off for the 
	 * remainder of the cycle time. Ideally a 12v bulb is used to help 
	 * visually demonstrate what exactly the loop is doing. 
	 */
	int onTime = (output / 255) * CYCLE_TIME_MS;
  	digitalWrite(LIGHT_PIN, LOW);
  	delay(onTime);
  	digitalWrite(LIGHT_PIN, HIGH);
  	delay(CYCLE_TIME_MS - onTime);
}
