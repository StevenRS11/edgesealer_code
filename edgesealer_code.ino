const int buttonPin = 3; // Pin connected to the button
const int outputPins[] = {51, 50, 49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36}; // Array of output pins
//relay map               1,  2,  3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 16
/**
1- 4 in piston
2- 24 in piston
3- grabber
4- alignment
5- main piston
6- ejection piston 1
7- ejection piston 2
8- ejection immobilize 1
9 ejection immobilize 2
10 ejection immobilize 3
11 ejection immobilize 4
12
13
14
15
16
**/
const int numOutputs = sizeof(outputPins) / sizeof(outputPins[0]); // Number of output pins

// Define the patterns for each state
const int numStates = 20; // Number of states

//ready to accpet paddle, aligment up, grabber fully retracted and open, main press open, ejectors open
const byte state1[] =  {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 1

//debug state all high
//const byte state1[] =  {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; // Pattern for State 1

//short piston extended into aligned paddle
const byte state2[] =  {LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 2

//paddle grabbed, alignment still up
const byte state3[] =  {LOW, HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 3

//dropped alignment
const byte state4[] =  {LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 4

//fully extend insertion pistons, insert paddle into press
const byte state5[] =  {LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 5

//clamp paddle in press with ejectors
const byte state6[] =  {LOW, LOW, LOW, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 6

//release paddle grabber
const byte state7[] =  {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 7

//retract long insertion piston to pull grabber out of press
const byte state8[] =  {LOW, HIGH, HIGH, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 8

//activate main press piston (start main press cycle)
const byte state9[] =  {LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 9

//imobilize ejectors
const byte state10[] =  {LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 10

//release and retract ejectors after full pressure is reached breifly
const byte state11[] =  {LOW, HIGH, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 11

//reclamp ejectors on paddle
const byte state12[] =  {LOW, HIGH, HIGH, LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 12

//retract main piston (finished main press cycle)
const byte state13[] =  {LOW, HIGH, HIGH, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 13

//extend long insertion piston
const byte state14[] =  {LOW, LOW, HIGH, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 14

//grab paddle
const byte state15[] =  {LOW, LOW, LOW, LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 15

//release paddle ejectors
const byte state16[] =  {LOW, LOW, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 16

//retract long insertion piston
const byte state17[] = {LOW, HIGH, LOW, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 17

//raise alignment
const byte state18[] = {LOW, HIGH, LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 18

//release paddle
const byte state19[] = {LOW, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 19

//retract short insertion piston, back to starting conditions
const byte state20[] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // Pattern for State 20


//const byte state20[] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; // Pattern for State 20


//const byte state21[] = {LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW}; // Pattern for State 21




const byte* states[numStates] = {state1, state2, state3, state4, state5, state6, state7, state8, state9, state10, state11, state12, state13, state14, state15, state16, state17, state18, state19, state20}; // Array of patterns for each state

// Define the delays between each state (in milliseconds)
const unsigned long stateDelays[numStates] = {1000, 2000, 1000, 1000, 5000, 2000, 1000, 4000, 7000, 1000 ,10000 ,7000, 10000, 5000 ,1000 ,2000 ,10000, 1000, 1000, 5000}; // Delay for each state
//                                              1     2     3     4     5     6     7     8     9     10    11    12      13    14   15    16   17     18     19    20   
bool buttonState = false; // Current state of the button
bool lastButtonState = false; // Previous state of the button
unsigned long lastDebounceTime = 0; // Last time the button was toggled
unsigned long debounceDelay = 50; // Delay time to debounce the button

void setup() {
 // Serial.begin (9600);
  setState(0);


  pinMode(buttonPin, INPUT_PULLUP); // Set button pin as input with internal pull-up resistor
  for (int i = 0; i < numOutputs; i++) {
    pinMode(outputPins[i], OUTPUT); // Set all output pins as output

  }
  setState(0);
}

void loop() {

  // Read the state of the button
  int reading = digitalRead(buttonPin);

  // Check if the button state has changed
  if (reading != lastButtonState) {
    lastDebounceTime = millis(); // Reset the debounce timer
  }

  // If a certain time has passed since the last change of the button state, consider it as stable
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // If the button state has changed and is low (pressed)
    if (reading != buttonState) {
      buttonState = reading; // Update the button state

      // If the button is pressed, start the sequence
      if (buttonState == LOW) {
        startSequence();
      }
    }
  }

  // Save the current button state for comparison in the next iteration
  lastButtonState = reading;
}

// Function to toggle the output pins sequentially with a delay between each
void startSequence() {
  for (int stateIndex = 0; stateIndex < numStates; stateIndex++) {
    // Set the output states according to the current pattern
    for (int pinIndex = 0; pinIndex < numOutputs; pinIndex++) {
      digitalWrite(outputPins[pinIndex], states[stateIndex][pinIndex]);
     // Serial.print("StateIndex= ");
    //  Serial.print(stateIndex);
     // Serial.print(" PinIndex= ");
    //  Serial.println(pinIndex);
    }
    
    // Delay for the specified time for the current state
    delay(stateDelays[stateIndex]);
  }
}

void setState(int stateIndex) {
    //sets the output state to the given stateIndex
    for (int pinIndex = 0; pinIndex < numOutputs; pinIndex++) {
      digitalWrite(outputPins[pinIndex], states[stateIndex][pinIndex]);
     // Serial.print("StateIndex= ");
     // Serial.print(stateIndex);
     // Serial.print(" PinIndex= ");
     // Serial.println(pinIndex);
    }

}