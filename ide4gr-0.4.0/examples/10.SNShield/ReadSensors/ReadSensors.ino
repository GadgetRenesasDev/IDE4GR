/*
  ReadSensors
  This sample read sensors on Sensor Network Shield,
  then display the data.

  Note: This sample needs to import below libraries.
        - SNShield
        - SPI
        - Wire
        
 
  This example code is in the public domain.
 */

SNShield kurumi;

void setup() {
  kurumi.begin();
  Serial.begin(9600);
}

void loop() {
  char ttt[100];
  
  kurumi.getAll();
  sprintf(ttt,"Temp: %.2f Lux: %.2f X: %.2f Y: %.2f Z: %.2f",kurumi.temp,kurumi.lux,kurumi.accx,kurumi.accy,kurumi.accz);
  Serial.println(ttt);
  delay(200);
  
}
