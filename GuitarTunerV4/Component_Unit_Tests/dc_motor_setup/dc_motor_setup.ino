#define AIN1_PIN 4
#define AIN2_PIN 5
#define PWMA_PIN 6

// Define motor speed and step duration
const int motorSpeed = 255;  // Max PWM value (0 to 255)
const int stepDuration = 25;  // Duration of each step in milliseconds

void setup() {
  // Set motor control pins as outputs
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
}

void loop() {
  // Move the motor forward for a short duration CLOCKWISE
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, motorSpeed);  // Start the motor at the defined speed

  delay(stepDuration);  // Wait for the step to complete

  // Stop the motor
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, 0);  // Stop sending PWM signal

  // Wait some time before the next step
  delay(1000);  // Wait for 1 second
}