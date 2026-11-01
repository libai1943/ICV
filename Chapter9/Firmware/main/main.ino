#include <SSD1306.h> // SSD1306 OLED display library
#include <Servo.h> // Servo-control library
#include <PinChangeInt.h>    //External interrupts
#include <MsTimer2.h>        //Timer interrupts

#include <TinyProtocol.h> // TinyProtocol communication library

////////OLED display pins///////////
#define OLED_DC 10 // OLED data/command selection pin
#define OLED_CLK 19 // OLED clock pin
#define OLED_MOSI 13 // OLED serial-data output pin
#define OLED_RESET 12 // OLED reset pin
/////////TB6612 motor-driver pins////
#define AIN1 11 // Motor-driver pin
#define AIN2 5
#define BIN1 6
#define BIN2 3
#define SERVO 9 // Steering-servo control pin
/////////Encoder pins////////
#define ENCODER_L 8  //Left encoder pulse input; each wheel uses two encoder channels
#define DIRECTION_L 4 // Left encoder direction-channel input
#define ENCODER_R 7
#define DIRECTION_R 2

#define ENCODER_K 2.55 // p / 39.25 (cm) * 100 / 10ms * 100
/////////Button pin////////
#define KEY 18

#define TRACK_WIDTH 0.156f // Vehicle track width, in meters
#define WHEEL_BASE 0.1445f // Vehicle wheelbase, in meters

// ============ MSG DEFINATION =============

#define MSG_SELF 0 // Vehicle destination identifier

#define MSG_TYPE_TWIST 0xe1
#define MSG_TYPE_PID 0xe2
#define MSG_TYPE_VELOCITY 0xe3

struct TwistMessage {
  int16_t velocity;
  int16_t angle;
};

struct PIDMessage {
  float kp;
  float ki;
};

struct VelocityMessage {
  int16_t velocity;
};


volatile long pulseL_, pulseR_;
int velocityLeft_, velocityRight_, targetVelocity_ = 0; // cm/s
int leftPWM_, rightPWM_;
int targetAngle_ = 0;
int voltage_;
float velocityKp_ = 10.0, velocityKi_ =  1.0;
bool racing_ = true;

Tiny::ProtoLight proto; // Create the TinyProtocol instance
Tiny::Packet<32> g_packet; // Create a packet with a 32-byte capacity
Servo servo; // Create the steering-servo instance
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0); // Create the OLED display instance

void setup() {
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();

  pinMode(AIN1, OUTPUT); //Motor-control pin
  pinMode(AIN2, OUTPUT); //Motor-control pin
  pinMode(BIN1, OUTPUT); //Motor-speed control pin
  pinMode(BIN2, OUTPUT); //Motor-speed control pin
  servo.attach(SERVO);

  pinMode(ENCODER_L, INPUT); //Encoder pins
  pinMode(DIRECTION_L, INPUT); //Encoder pins
  pinMode(ENCODER_R, INPUT); //Encoder pins
  pinMode(DIRECTION_R, INPUT); //Encoder pins
  pinMode(KEY, INPUT); //Button pin
  pinMode(LED_BUILTIN, OUTPUT);
  delay(200); //Wait for initialization to finish

  attachInterrupt(0, readEncoderR, CHANGE);           //Enable the external interrupt for encoder input 1
  attachPinChangeInterrupt(4, readEncoderL, CHANGE);  //Enable the pin-change interrupt for encoder input 2

  //Use Timer2 to generate an interrupt every 10 ms
  MsTimer2::set(10, timer);
  MsTimer2::start();

  Serial.setTimeout(0);
  Serial.begin(57600);
  proto.enableCheckSum();
  proto.beginToSerial();
}


/*
The itoa function takes three arguments:

Integer value: the number to convert.
String buffer: storage for the converted characters.
Radix: the numeral base, such as 10 for decimal or 16 for hexadecimal.
*/
void display() {
  static char buf[8] = { 0 };

  oled.clear();

  oled.drawstring(00, 00, "Volta");
  oled.drawstring(06 * 6, 00, itoa(voltage_, buf, 10));


  oled.drawstring(00, 01, "Speed");
  oled.drawstring(06 * 6, 01, itoa(velocityLeft_, buf, 10));
  oled.drawstring(06 * 11, 01, itoa(velocityRight_, buf, 10));
  oled.drawstring(06 * 17, 01, "cm/s");

  oled.drawstring(00, 02, "PWM");
  oled.drawstring(06 * 6, 02, itoa(leftPWM_, buf, 10));
  oled.drawstring(06 * 11, 02, itoa(rightPWM_, buf, 10));

  oled.drawstring(00, 03, "PID");
  oled.drawstring(06 * 6, 03, itoa(velocityKp_ * 100, buf, 10));
  oled.drawstring(06 * 11, 03, itoa(velocityKi_ * 100, buf, 10));

  oled.drawstring(00, 04, "Targt");
  oled.drawstring(06 * 6, 04, itoa(targetVelocity_, buf, 10));
  oled.drawstring(06 * 11, 04, "cm/s");
  oled.drawstring(06 * 17, 04, itoa(targetAngle_, buf, 10));

  oled.drawstring(00, 05, "Msg");
  oled.drawstring(06 * 6, 05, itoa(MSG_SELF, buf, 10));

  oled.display();
}

void loop() {
  digitalWrite(LED_BUILTIN, racing_ ? HIGH : LOW); // Set the built-in LED to HIGH if 'racing_' is true, otherwise set it to LOW.

  if (millis() % 1000 == 0 && !racing_) display(); // Call the display function every 1000 milliseconds if not in racing mode.
  if (racing_) recv(); // If in racing mode, call the 'recv' function to receive data.
}

void recv() {
  int len = proto.read(g_packet); // Read data into 'g_packet' using the 'proto' object, and store the length in 'len'.

  if (len > 1) { // If the length of the received data is greater than 1...
    byte* data = g_packet.data(); // Retrieve a pointer to the data in 'g_packet'.
    if (data[0] != MSG_SELF) return; // If the first byte is not 'MSG_SELF', return from the function.
    handleMessage(data[1], data + 2, len - 2); // Call 'handleMessage' with the type, data, and length.
  }
}

void handleMessage(byte type, byte *data, int len) {
  // Handle different message types
  if (type == MSG_TYPE_TWIST) {
    // Handle Twist Message
    if (len != sizeof(TwistMessage)) return; // If the length does not match 'TwistMessage', return.
    TwistMessage *ptr = (TwistMessage *)data; // Cast 'data' to 'TwistMessage'.
    targetVelocity_ = min(max(ptr->velocity, -100), 100); // Set target velocity, ensuring it stays within [-100, 100].
    targetAngle_ = min(max(ptr->angle, -40), 40); // Set target angle, ensuring it stays within [-40, 40].
  } else if (type == MSG_TYPE_PID) {
    // Handle PID Message
    if (len != sizeof(PIDMessage)) return; // If the length does not match 'PIDMessage', return.
    PIDMessage *pid = (PIDMessage *)data; // Cast 'data' to 'PIDMessage'.
    velocityKp_ = pid->kp; // Set the Kp value for velocity.
    velocityKi_ = pid->ki; // Set the Ki value for velocity.
  }
}

void timer() {
  sei(); //Enable global interrupts

  measureVelocity(); // Call function to measure velocity.
  measureVoltage(); // Call function to measure voltage.

  if (racing_) {
    control(); // If in racing mode, call the 'control' function for active control.
  } else {
    standby(); // If not racing, enter standby mode.
  }
  if (doesClick()) { // Check for click/button press.
    racing_ = !racing_; // Toggle the 'racing_' state.
  }
}

void standby() {
  // Set all motor control pins to LOW and position the servo to a neutral position
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  servo.write(95); // Set servo to a neutral position.
}

char doesClick() {
  static byte flag_key = 1; // Static variable to hold the state of the key.
  if (flag_key && digitalRead(KEY) == 0) { // If key was previously up and is now pressed...
    flag_key = 0; // Set the flag to indicate key is pressed.
    if (digitalRead(KEY) == 0)  return 1; // If key is still pressed, return 1 (indicating a click).
  }
  else if (digitalRead(KEY) == 1) flag_key = 1; // If key is released, reset flag.
  return 0; // Return 0 if no click is detected.
}

void control() {
  int targetLeft, targetRight; // Variables to hold target speeds for left and right wheels.
  kinematicAnalysis(&targetLeft, &targetRight); // Perform kinematic analysis to determine target speeds.

  leftPWM_ = incrementalPI(velocityLeft_, targetLeft, 0); // Calculate left PWM using PI controller.
  rightPWM_ = incrementalPI(velocityRight_, targetRight, 1); // Calculate right PWM using PI controller.

  setPWM(leftPWM_, rightPWM_); // Set the calculated PWM values to the motors.
  servo.write(95 + 1.61437 * targetAngle_ - 0.86936); // Adjust the servo position based on the target angle.
}

void measureVelocity() {
  // Measure the velocity of the left and right wheels using encoders
  velocityLeft_ = pulseL_ * ENCODER_K;  pulseL_ = 0; //Estimate the left-wheel velocity from pulses counted over the sampling interval, then reset the count.
  velocityRight_ = pulseR_ * ENCODER_K; pulseR_ = 0; //Read the right-wheel pulse count, then reset it
}

void measureVoltage() {
  // Measure the battery voltage
  static int totalVoltage = 0, measureCount = 0; // Static variables to accumulate voltage readings and count.
  totalVoltage += analogRead(0); // Add the current voltage reading to the total.
  measureCount++; // Increment the count of measurements.
  if (measureCount == 200) { // If 200 measurements have been made...
    voltage_ = totalVoltage * 0.05371 / 2; // Calculate the average voltage.
    totalVoltage = 0; // Reset the total voltage.
    measureCount = 0; // Reset the measurement count.
  }
}

/**
   Vehicle kinematic model
**/
void kinematicAnalysis(int *targetA, int *targetB) {
  //Rear-wheel differential velocities
  // Compute the target velocity for motor A from the commanded speed and steering angle
  *targetA = targetVelocity_ * (1 + TRACK_WIDTH * tan(targetAngle_ * PI / 180) / 2 / WHEEL_BASE);
  // Compute the target velocity for motor B from the commanded speed and steering angle
  *targetB = targetVelocity_ * (1 - TRACK_WIDTH * tan(targetAngle_ * PI / 180) / 2 / WHEEL_BASE);
}

/**
   Incremental PI controller
   pwm += Kp[e(k)-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
   e(k) is the current error
   e(k-1) is the previous error, and so on
   pwm stores the accumulated PWM command updated by the incremental controller
   The wheel-speed feedback loop uses PI control
   pwm += Kp[e(k)-e(k-1)]+Ki*e(k)
**/
int incrementalPI(int current, int target, char index)
{
  // Static variables retain the output and the previous error
  static int output[2] = { 0 };
  static float lastError[2] = { 0.0 };

  // Compute the current error
  float error = target - current;
  // Update the output using the incremental PI equation
  output[index] += velocityKp_ * (error - lastError[index]) + velocityKi_ * error;
  // Limit the output to the interval [-250, 250]
  output[index] = min(max(output[index], -250), 250);
  // Store the current error for the next update
  lastError[index] = error;
  // Return the computed output
  return output[index];
}

// setPWM: apply the motor PWM commands
void setPWM(int motorA, int motorB) {
  // Control motor A
  if (motorA > 0) {
    // Motor A rotates forward
    analogWrite(AIN2, motorA);
    digitalWrite(AIN1, LOW);
  }
  else {
    // Motor A rotates in reverse
    digitalWrite(AIN1, HIGH);
    analogWrite(AIN2, 255 + motorA);
  }

  // Control motor B
  if (motorB > 0) {
    // Motor B rotates forward
    digitalWrite(BIN2, LOW);
    analogWrite(BIN1, motorB);
  }
  else {
    // Motor B rotates in reverse
    analogWrite(BIN1, 255 + motorB);
    digitalWrite(BIN2, HIGH);
  }
}

/**
   Read the encoder on both signal edges to obtain two counts per pulse cycle
 **/
// readEncoderL: read the left encoder
void readEncoderL() {
  // Read the left encoder pulse level
  if (digitalRead(ENCODER_L) == LOW) {
    //Falling-edge interrupt
    if (digitalRead(DIRECTION_L) == LOW) pulseL_--;  //Determine direction from the other encoder channel
    else pulseL_++;
  }
  else {
    //Rising-edge interrupt
    if (digitalRead(DIRECTION_L) == LOW) pulseL_++; //Determine direction from the other encoder channel
    else pulseL_--;
  }
}

/**
   Read the encoder on both signal edges to obtain two counts per pulse cycle
 **/
// readEncoderR: read the right encoder
void readEncoderR() {
  // Read the right encoder pulse level
  if (digitalRead(ENCODER_R) == LOW) { //Falling-edge interrupt
    if (digitalRead(DIRECTION_R) == LOW) pulseR_++; //Determine direction from the other encoder channel
    else pulseR_--;
  }
  else { //Rising-edge interrupt
    if (digitalRead(DIRECTION_R) == LOW) pulseR_--; //Determine direction from the other encoder channel
    else pulseR_++;
  }
}
