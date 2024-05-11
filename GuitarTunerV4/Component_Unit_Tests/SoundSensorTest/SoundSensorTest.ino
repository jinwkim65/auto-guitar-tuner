/*
Program to test that the KY-037 Sound Sensor Works
*/
int digitalPin = 11;   // KY-037 digital interface
int analogPin = A0;   // KY-037 analog interface
int ledPin = 10;      // Arduino LED pin
int digitalVal;       // digital readings
int analogVal;        // analog readings
void setup()
{
  pinMode(digitalPin,INPUT); 
  pinMode(analogPin, INPUT);
  pinMode(ledPin,OUTPUT);      
  Serial.begin(9600);
}
void loop()
{
  digitalWrite(ledPin, LOW);
  // Read the digital inteface
  digitalVal = digitalRead(digitalPin); 
  
  if(digitalVal == 0) // Soudn detection = 0
  {
    digitalWrite(ledPin, HIGH); // Turn ON Arduino's LED
    delay(500);
  }

  analogVal = analogRead(analogPin);
  // Print analog value to serial
  if (analogVal >= 500){
    Serial.println(analogVal);
  }
}