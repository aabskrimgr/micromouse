
const int btn = 2;  // Pin where the button is connected
const int ledPin = 13;  // Pin where the LED is connected

// Debounce function
bool debounce() {
  static uint16_t state = 0; // Keeps track of the button state history
  state = (state << 1) | digitalRead(btn) | 0xfe00; // Shift state, read new button input, and mask
  return (state == 0xff00); // Return true if the button is pressed and stable
}

void setup() {
  pinMode(btn, INPUT);   // Set button pin as input
  pinMode(ledPin, OUTPUT);  // Set LED pin as output
  digitalWrite(ledPin, LOW); // Initialize LED as off
  Serial.begin(9600);    // Initialize serial communication
}

void loop() {
  if (debounce()) {  // Call debounce to check stable button press
    digitalWrite(ledPin, HIGH); // Turn LED on when button is pressed
    Serial.println("Button Pressed - Debounced");
  } else {
    digitalWrite(ledPin, LOW);  // Turn LED off when button is not pressed
  }
}
