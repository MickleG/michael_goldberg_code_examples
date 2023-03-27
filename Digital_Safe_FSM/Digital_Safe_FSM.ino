// Set flag for displaying the current safe status
const boolean VERBOSE = false;

// Define integer values for encoder rotation
const int CW = 1;
const int CCW = 0;

// Define state names and associated integer values
const int STAGE_0 = 100;
const int STAGE_1 = 101;
const int STAGE_2 = 102;
const int STAGE_3 = 103;
const int UNLOCK  = 104;

// Define input pin numbers
const int CHA_PIN = 2;
const int CHB_PIN = 3;
const int ESW_PIN = 4;

// Define output pin
const int RED_LED = 13;

// Define switch bounce delay (in milliseconds)
const int DELAY = 1;

// Define state variable and initialize
int state = STAGE_0;

// Define encoder direction variables;
int enc_dir = CW;
int last_enc_dir = CW;

// Define encoder count and initialize
int enc_count = 0;
int last_state;

// Define flag to be set if encoder rotation is detected
boolean rot_flag = false;

//boolean buttonFlag = true;
char buffer[100];

void setup() {
  // Declare input pins
  pinMode(CHA_PIN, INPUT);
  pinMode(CHB_PIN, INPUT);
  pinMode(ESW_PIN, INPUT);

  // Set initial state output
  pinMode (13, OUTPUT);

  // Start serial port
  Serial.begin(9600);

  // Wait for serial port to be established
  delay(100);

  // Announce startup message
  if (VERBOSE) {
    Serial.println("Welcome to the ME 588 digital safe");
    Serial.println("----------------------------------");
    Serial.println();
    Serial.println("In STAGE_0");
  }
}

void loop() {
  // 1. READ DEVICE INPUTS
  int esw_state = debounceSwitch(ESW_PIN, DELAY);
  int enc_change = readEncoder(CHA_PIN, CHB_PIN);

  if (enc_change != 0) {    //  did rotation occur?
    rot_flag = true;
    if (enc_change == 1) {  // if so, in what direction? (either 1 or -1)
      enc_dir = CW;
    } else {
      enc_dir = CCW;
    }
  }

  // 2. COMPUTE ENCODER COUNT
  if (rot_flag) {   // has encoder rotated?
    if (enc_dir != last_enc_dir) {  // has direction changed?
      if (enc_dir == CW) {          // if so, reset count
        enc_count = 1;
      } else {
        enc_count = -1;
      }
    } else {           // continued rotation in same direction
      enc_count = enc_count + enc_change;
      //Serial.println("TESTING");
    }
    if (VERBOSE) {
      // Serial.print("Count: ");
      // Serial.println(enc_count);
      sprintf(buffer, "Count: %d", enc_count);
      Serial.println(buffer);
    } else {
      Serial.println("***");
    }
    
  }

  // 3. DETERMINE ADDITIONAL INPUTS FOR FSM
  boolean cw4 = (enc_count == 4);
  boolean ccw7 = (enc_count == -7);
  boolean cw6 = (enc_count == 6);
  boolean cw10 = (enc_count == 10);
  boolean ccw10 = (enc_count == -10);

  // 4. EVALUATE FINITE STATE MACHINE
  //    Boolean Inputs: esw_state, enc_dir, cw4, ccw7, cw6, rot_flag
  switch (state) {
    case STAGE_0:
      enc_count = 0;
      digitalWrite(13, HIGH);
      if (esw_state) {
        state = STAGE_1;
        enc_dir = CW;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      } else {
        state = STAGE_0;
      }
      break;
    case STAGE_1:
      digitalWrite(13, HIGH);
      if (cw4) {
        state = STAGE_2;
        enc_dir = CCW;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      } else if (enc_dir == CW || rot_flag == false) {
        state = STAGE_1;
        enc_dir = CW;
      } else {
        state = STAGE_0;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      }
      break;
    case STAGE_2:
      digitalWrite(13, HIGH);
      if (ccw7) {
        state = STAGE_3;
        enc_dir = CW;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      } else if (enc_dir == CCW || rot_flag == false) {
        state = STAGE_2;
        enc_dir = CCW;
      } else {
        state = STAGE_0;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      }
      break;
    case STAGE_3:
      digitalWrite(13, HIGH);
      if (cw6) {
        state = UNLOCK;
        enc_count = 0;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      } else if (enc_dir == CW || rot_flag == false) {
        state = STAGE_3;
        enc_dir = CW;
      } else {
        state = STAGE_0;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      }
      break;
    case UNLOCK:
      digitalWrite(13, LOW);
      if (cw10 || ccw10) {
        state = STAGE_0;
        sprintf(buffer, "CURRENT STATE: %d", state);
        Serial.println(buffer);
      } else {
        state = UNLOCK;
      }
      break;
    default:
      break;
  }

  // 5. STORE ENCODER DIRECTION AND RESET FLAG
  last_enc_dir = enc_dir;
  rot_flag = false;
}

int debounceSwitch(int pin, int bdelay) {
  // A function that a switch value from a specified pin
  // number and returns the most recent stable value from that
  // pin as either HIGH or LOW, based on a given bounce delay
  // (in msec). A switch index must be assigned to keep bounce
  // timing for each switch from overwriting one another.
  
  // Define variables to hold the most recent
  // switch value and last switch transition time
  static int prior_reading = LOW;
  static long prior_time = 0;

  // Define variable to hold current stable switch state
  // and initialize to be in the LOW state
  static int switch_state = LOW;

  // Read current switch value into local variable
  int current_reading = digitalRead(pin);

  // Check for a change in switch state
  if (current_reading != prior_reading) {
    // If a change is detected in the switch value,
    // reset the delay clock and update the prior value
    prior_time = millis();
    prior_reading = current_reading;
  }

  // Has the switch potentially taken on a new value?
  if (switch_state != current_reading) {
    // Did the switch state remained unchanged long enough to
    // be considered debounced?
    if ((millis() - prior_time) > bdelay) {
      // If so, accept updated switch value   
      switch_state = current_reading;
      // if (switch_state == 1) {
      //   buttonFlag = !buttonFlag;
      // }
    }
  }

  return switch_state;
}

int readEncoder(int chA, int chB) {
  // A function that reads the quadrature input from specified
  // pins and returns an encoder count change.
  
  // Create variable to return based on detected rotor rotation
  int result = 0;

  // Keep track of last switch state
  static int chA_last = digitalRead(chA);
  static int chB_last = digitalRead(chB);

  // Read the quadrature inputs
  int chA_new = digitalRead(chA);
  int chB_new = digitalRead(chB);

  // We look for Channel A to switch from LOW to HIGH
  if ((chA_last == LOW) && (chA_new == HIGH)) {
    // If Channel B is HIGH while Channel A goes 0->1, 
    // then the shaft is turning CW, and we should
    // increase the encoder count by 1. Otherwise it is
    // turning CCW, and we should decrease the encoder
    // count by 1.    
    if ((chB_last == HIGH) && (chB_new == HIGH)) { // CW turn
      result = 1;
    } 
    if ((chB_last == LOW) && (chB_new == LOW)) { // CCW turn
      result = -1;
    } 
  }

  // Reassign prior Channel A value
  chA_last = chA_new;
  chB_last = chB_new;

  // Return encoder change
  return result;
}
