/*
We originally use a rotary encoder and an I2C Display for the user
to select the note that needs to be tuned. However, we ran into 
issues with Arduino memory as our final program with all the components
took up too much space, and the I2C display would not work. We elected to
use LEDs to indicate which string was being tuned as the I2C display was
more aesthetic purposes and less for functionality.
We are including this program in our unit tests to show that our I2C display
and rotary encoder work together to select the note.
*/

#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define DEBOUNCE_DELAY 50 // milliseconds
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define outputA 3
#define outputB 2
#define SCL A5 
#define SDA A4
#define ENC_RD PIND
#define BUTTON_PIN 7


// Set up display
Adafruit_SSD1306 disp(128, 64, &Wire);

int noteSelected = 0;

// Guitar notes
int currentNote = 0;
// "E", "A", "D", "G", "B", "E" correspond to 0, 1, 2, 3, 4, 5
String notes[] = {"E2", "A2", "D3", "G3", "B3", "E4"};
double frequencies[] = {82.41, 110.00, 146.83, 196.00, 246.94, 329.63};


// Rotary encoder value
int rotaryValue = 0;
unsigned long lastDebounceTime = 0;

void setup() {
  Serial.begin(9600);

  DDRD &= ~(1 << outputA); //Set outputA as input
  PORTD |= (1 << outputA); //Enable pull-up resistor on outputA
  DDRD &= ~(1 << outputB); // Set outputB as input
  PORTD |= (1 << outputB); // Enable pull-up resistor on outputB
  

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(outputA), updateRotation, CHANGE);
  attachInterrupt(digitalPinToInterrupt(outputB), updateRotation, CHANGE);

  // Display Setup
  disp.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  disp.clearDisplay();
  disp.display();
}


void loop() {
  // Map rotary value to a note
  mapNote();
  Serial.println(rotaryValue);

  // Draw note
  disp.clearDisplay();
  displayNote();
  // disp.display();

  delay(10);
}

void displayNote() {
  disp.setTextSize(2); //Set text size. You can adjust this for different display sizes
  disp.setTextColor(SSD1306_WHITE); // Set font color to white
  disp.setCursor(0, 20); // Set cursor to start at top-left corner of the display

  // Print current note
  disp.println(notes[currentNote]);

  // Optionally, display the frequency as well
  disp.setTextSize(1); //Smaller text for the frequency
  disp.setCursor(0, 50); // Move cursor to lower on the display
  disp.print("Freq: ");
  disp.print(frequencies[currentNote]);
  disp.println(" Hz");

  // Show the changes on the display
  disp.display();
}

// Encoder interrupt routine influenced by https://github.com/mo-thunderz/RotaryEncoder/blob/main/Arduino/ArduinoRotaryEncoder/ArduinoRotaryEncoder.ino
void updateRotation() {
  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {
    static uint8_t old_AB = 3; //Lookup table index
    static int8_t encval = 0; // Encoder value
    // Encoder lookup table
    static const int8_t enc_states [] PROGMEM = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; 
    old_AB <<=2; // Remember previous state
    old_AB |= (( ENC_RD >> 2 ) & 0x03 );
    encval += pgm_read_byte(&(enc_states[( old_AB & 0x0f )]));

    // change rotary value by encval
    rotaryValue += encval;
    encval = 0;

    /*
    Wrap rotary value within the range 0-128
    We do this in the following steps:
    1. rotaryValue % 129 ensures that any rotaryValue above 128 wraps back to 0
    2. rotaryValue % 129 + 129 ensures that a negative rotaryValue will become positive
    3. (rotaryValue % 129 + 129) % 129 ensures that both possibilities result in a number [0, 28]
    */
    rotaryValue = (rotaryValue % 129 + 129) % 129;
  }
}

// Maps the rotary encoder value to one of the six guitar notes
void mapNote() {
  // Divide the range 0-128 into six segments, each segment corresponding to a note
  currentNote = rotaryValue / 22; // 128 / 6 approximately equals 22, rounding up to handle overflow

  // Cap note at 5
  if (currentNote > 5) currentNote = 5;
}