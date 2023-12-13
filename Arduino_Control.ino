#include <Adafruit_NeoPixel.h>
// #include <rp2040_pio.h>

#include <Wire.h>
#include <AutoPID.h>
#include <Adafruit_MotorShield.h>
#include "TimerOne.h"

int motor_pin = A3;
int potentiometer_pin = 2;
double current_pos = 0;
// double kp = 1500, ki = 500, kd = 0;
double kp = 500, ki = 30, kd = 100;
double setpointValue = 0;
double out_min = -165;
double out_max = 165;
double outputValue = 0;
double objective = 0;


#define PIN 7
#define NUM_LEDS 12
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB);
uint32_t color = strip.Color(75, 250, 100);
int receivedValue = 0;
String inputString = "";
int strip_lst[] = {6, 7, 8, 9, 10, 11, 0, 1, 2, 3};

      

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motor = AFMS.getMotor(3);
AutoPID Controller(&current_pos, &setpointValue, &outputValue, out_min, out_max, kp, ki, kd);



void update_position() {
	double exponent = 5;
	current_pos = analogRead(potentiometer_pin) / 1023.0;
	current_pos = (pow(current_pos + 0.1, 1.0 / exponent) - pow(0.1, 1.0 / exponent)) / (1 - pow(0.1, 1.0 / exponent));
}

void drive_motor(double pwr) {
	if (pwr < 0) {
		motor->run(BACKWARD);
		motor->setSpeed(-pwr + 90);
	} else {
		motor->run(FORWARD);
		motor->setSpeed(pwr + 90);
	}
}

void motor_to_setpoint(double target) {
	setpointValue = target;
	long start_timer = millis(); 
	while (abs(current_pos - target) >= 0.001) {
		if (millis() - start_timer > 8000) {
			break;
		}
		Controller.run();
		Serial.print("current pos: ");
		Serial.print(current_pos);
		Serial.print("\tpwr: ");
		Serial.println(outputValue);
		drive_motor(outputValue);
	}
	start_timer = millis();
	while (millis() - start_timer < 1000) {

		Controller.run();
		Serial.print("current pos: ");
		Serial.print(current_pos);
		Serial.print("\tpwr: ");
		Serial.println(outputValue);
		drive_motor(outputValue);
	}
	motor->run(RELEASE);
}


 // Declare the object outside setup() and loop()

void setup() {
	Serial.begin(115200); 
	AFMS.begin(); // Initialize the motor shield
	Timer1.initialize(10000); // 0.1s
	Timer1.attachInterrupt(update_position);
	Controller.setTimeStep(100);
	Controller.setBangBang(0.1);
	// motor_to_setpoint(0.8);
	strip.begin();
  	strip.show();            // Initialize all pixels to 'off'
  	strip.setBrightness(40); // 40/255 brightness (about 15%)
		  
}

void loop() {
Serial.print("input from 1-10:");
	while (Serial.available() == 0) {
    // Do nothing, keep waiting for input
  	}
	
	receivedValue = Serial.parseInt();
	Serial.print("Received value: ");
	Serial.println(receivedValue);
	for (int i = 10; i > 0; i--) {
		strip.setPixelColor(strip_lst[i], 0);
		delay(50);
		strip.show();
	}
	for (int i = 0; i < receivedValue; i++) {
		strip.setPixelColor(strip_lst[i], color);
		delay(100);
		strip.show();
	}
	delay(50);

	Serial.print("input from 0-1:");
	while (Serial.available() == 0) {
    // Do nothing, keep waiting for input
  	}
	objective = Serial.parseFloat();
	motor_to_setpoint(objective);
	for (int i = 0; i < 10; i++) {
		Serial.print("current pos: ");
		Serial.println(current_pos);
		delay(200);
	}

}


