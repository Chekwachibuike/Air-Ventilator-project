#include <LiquidCrystal.h>
#include <Wire.h>
#include <Stepper.h>
#include <AccelStepper.h>
#include <Adafruit_MPRLS.h>
#include <MAX30105.h>
#include "heartRate.h"

// Constants
#define STEPS 200 // Number of steps for the stepper motor
#define BUTTON_PIN 2 // Pin for the button input
#define LED_PIN 13 // Pin for the LED output

// Initialize objects for LCD, Stepper, and Sensors
LiquidCrystal lcd(12, 11, 6, 5, 4, 3); // Pins for the LCD display
int buttonState = 0; // Current state of the button
int lastButtonState = 0; // Previous state of the button
int buttonCounter = 0; // Counts button presses
bool isDisplayOn = false; // Tracks if the display is currently on
int potValue = 0; // Potentiometer value
float air_pressure = 0; // Air pressure reading
Adafruit_MPRLS mpr = Adafruit_MPRLS(); // Pressure sensor
AccelStepper myStepper(AccelStepper::DRIVER, 27, 28); // Stepper motor setup
MAX30105 particleSensor; // Particle sensor for heart rate
const byte RATE_SIZE = 4; // Size of the array for storing heart rate readings
byte rates[RATE_SIZE]; // Array to store heart rate readings
byte rateSpot = 0; // Index for storing heart rate readings
long lastBeat = 0; // Time of the last detected heartbeat
float beatsPerMinute; // Calculated beats per minute
int beatAvg; // Average beats per minute

void setup() {
  // Setup the pin modes for the button and LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  // Initialize the LCD and serial communication
  lcd.begin(20, 4);
  Serial.begin(9600);
  
  // Initialize the pressure sensor
  mpr.begin();
  
  // Setup the stepper motor with max speed and acceleration
  myStepper.setMaxSpeed(200);
  myStepper.setAcceleration(50);
  
  // Initialize the particle sensor for heart rate
  particleSensor.begin(Wire, I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A); // Setting pulse amplitude for red LED
  particleSensor.setPulseAmplitudeGreen(0); // Turning off the green LED
  
  // Check if the pressure sensor is connected properly
  if (!mpr.begin()) {
    Serial.println("Could not find a valid MPRLS sensor, check wiring!");
    while (1);
  }
}

void loop() {
  // Track the current time in milliseconds
  unsigned long currentMillis = millis();
  
  // Read the current state of the button
  buttonState = digitalRead(BUTTON_PIN);

  // Detect button press and debounce it
  if (buttonState != lastButtonState && buttonState == LOW) {
    buttonCounter++;
    lastButtonState = buttonState;
    delay(50); // Debounce delay
  }

  // Handle different button presses
  switch (buttonCounter) {
    case 1:
      // First button press: Display project name and instructions
      if (!isDisplayOn) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("DAVID'S PROJECT");
        lcd.setCursor(0, 2);
        lcd.print("Press any key");
        digitalWrite(LED_PIN, HIGH); // Turn on the LED
        isDisplayOn = true;
      }
      break;

    case 2:
      // Second button press: Prompt for blood oxygen reading
      if (isDisplayOn) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Take your Blood");
        lcd.setCursor(0, 1);
        lcd.print("Oxygen reading");
        isDisplayOn = false;
      }
      break;

    case 3:
      // Third button press: Display heart rate and air pressure
      if (!isDisplayOn) {
        lcd.clear();
        
        // Get the infrared value from the particle sensor
        long irValue = particleSensor.getIR();

        // Check if a heartbeat is detected
        if (checkForBeat(irValue)) {
          long delta = currentMillis - lastBeat;
          lastBeat = currentMillis;
          beatsPerMinute = 60 / (delta / 1000.0);
          
          // Store the beats per minute if it's within a valid range
          if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;
            beatAvg = 0;
            
            // Calculate the average heart rate
            for (byte x = 0; x < RATE_SIZE; x++)
              beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
          }
        }

        // Display the heart rate on the LCD
        int heartRate = beatAvg;
        lcd.setCursor(0, 0);
        lcd.print("BPM:");
        lcd.print(heartRate);

        // Read and display air pressure from the sensor
        air_pressure = mpr.readPressure();
        lcd.setCursor(0, 1);
        lcd.print("Air Pressure:");
        lcd.print(air_pressure);
        isDisplayOn = true;
      }

      // Read the potentiometer value and adjust stepper motor speed accordingly
      potValue = analogRead(A0);
      int stepperSpeed = map(potValue, 0, 1023, 0, 1000);
      myStepper.setMaxSpeed(stepperSpeed);
      myStepper.setSpeed(60);
      myStepper.setAcceleration(stepperSpeed / 2);
      myStepper.runSpeed();

      // Print sensor readings to the serial monitor for debugging
      Serial.print("IR=");
      Serial.print(particleSensor.getIR());
      Serial.print(", BPM=");
      Serial.print(beatsPerMinute);
      Serial.print(", Avg BPM=");
      Serial.print(beatAvg);
      if (particleSensor.getIR() < 50000) {
        Serial.print(" No finger?");
      }
      Serial.println();
      break;

    case 4:
      // Fourth button press: Reset the display and turn off the LED
      buttonCounter = 0;
      lcd.clear();
      digitalWrite(LED_PIN, LOW); // Turn off the LED
      isDisplayOn = false;
      potValue = 0; // Reset potentiometer value
      break;
  }

  delay(10); // Small delay for loop stability
}
