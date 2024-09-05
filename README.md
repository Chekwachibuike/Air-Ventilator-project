This project is a prototype for a medical air ventialtor. It uses an LCD for display, a pressure sensor for air pressure readings, a pulse sensor for heart rate monitoring, and a stepper motor for mechanical motion control. The device is operated using a push button, and the user interface is displayed on a 20x4 LCD screen.

It makes use of an Arduino Board,
20x4 LCD Display
Adafruit MPRLS Pressure Sensor,
MAX30105 Pulse Sensor,
Stepper Motor,
AccelStepper Library,
Push Button,
Potentiometer,
LED and a power source or battery.


This project operates in different modes, which are controlled using a push button:

Welcome Mode: Displays the project title and an initial message. The LED is turned on to indicate that the device is ready.

Blood Oxygen Reading Prompt: Prompts the user to take a blood oxygen reading.

Heart Rate and Air Pressure Monitoring: Displays the heart rate (in beats per minute) and air pressure readings on the LCD. The stepper motor’s speed is adjustable using a potentiometer.

Reset Mode: Resets the display, turns off the LED, and prepares the device for a new session.

It is wired as follows:

LCD Display: Connect the pins 12, 11, 6, 5, 4, and 3 on the Arduino to the corresponding pins on the LCD.

Pressure Sensor (Adafruit MPRLS): Connect to the I2C pins on the Arduino.

Pulse Sensor (MAX30105): Connect to the I2C pins on the Arduino.

Stepper Motor: Connect the control pins to pins 27 and 28 on the Arduino.

Button: Connect to pin 2 on the Arduino with an internal pull-up resistor enabled.

LED: Connect to pin 13 on the Arduino.

Potentiometer: Connect one end to 5V, the other to GND, and the middle pin to A0 on the Arduino.

Instructions:

Power On: Connect the Arduino to a power source.

Button Navigation: Use the push button to navigate through the different modes:

Press 1: Displays the project title and a welcome message.

Press 2: Prompts you to take a blood oxygen reading.

Press 3: Displays heart rate and air pressure. Adjust the stepper motor speed with the potentiometer.

Press 4: Resets the device to the initial state.

Serial Monitor: Open the serial monitor at 9600 baud to see detailed sensor readings and debug information.
