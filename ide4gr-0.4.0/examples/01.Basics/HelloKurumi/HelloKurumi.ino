/*
  HelloKurumi
 
  This example code is in the public domain.
 */

// Pin 22,23,24 are assigned to RGB LEDs.
int led_red   = 22; // LOW active
int led_green = 23; // LOW active
int led_blue  = 24; // LOW active

// the setup routine runs once when you press reset:
void setup() {
  //setPowerManagementMode(PM_STOP_MODE, 0, 1023); //Set CPU STOP_MODE in delay()
  //setOperationClockMode(CLK_LOW_SPEED_MODE); //Set CPU clock from 32MHz to 32.768kHz

  // initialize the digital pin as an output.
  Serial.begin(9600);
  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);
  pinMode(led_blue, OUTPUT);

  // turn the LEDs on, glow white.
  digitalWrite(led_red, LOW);
  digitalWrite(led_green, LOW);
  digitalWrite(led_blue, LOW);
    
}

// the loop routine runs over and over again forever:
void loop() {
  Serial.println("Hello");
  digitalWrite(led_red, HIGH);   // turn the RED LED off, glow sky blue.
  delay(200);                   // wait 200ms

  digitalWrite(led_red, LOW);    // turn the RED LED on
  digitalWrite(led_green, HIGH); // turn the GREEN LED off, glow pink.
  delay(200);                   // wait 200ms

  digitalWrite(led_green, LOW);  // turn the GREEN LED on
  digitalWrite(led_blue, HIGH);  // turn the BLUE LED off, glow yellow.
  delay(200);                   // wait for a second

  digitalWrite(led_blue, LOW);   // turn the BLUE LED on
}
