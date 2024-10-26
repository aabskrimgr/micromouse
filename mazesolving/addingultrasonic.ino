//library
#include <SparkFun_TB6612.h>  
#include <QTRSensors.h> 

//sensors
#define NUM_SENSORS 8
unsigned int sensors1[8];
int thr[8];

//motor pins
#define AIN1 4
#define BIN1 6
#define AIN2 5
#define BIN2 7
#define PWMA 3
#define PWMB 9
#define STBY 8
const int offsetA = 1;
const int offsetB = 1;

//pid parameters
#define MaxSpeed 255
#define BaseSpeed 255
int lastError = 0;
float kp = 0.4;    
float kd = 0.1;
int last_pos = 3500;

//control pins
#define sw1 11
#define sw2 12
#define sw3 2
#define led 8
int s1,s2;
char dir;
int chr = 0;

// ultrasnic sensor
#define TRIG_PIN 10  // to be changed as hardware
#define ECHO_PIN 9 // to be changed as hardware
const int distanceThreshold = 20; // in cm

//distance measurement function
long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // Convert time to distance in cm
}



// shortest path parameters
int num = 0;
char path[100];
int path_length = 0;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);
QTRSensors qtra;

// debouncing function
bool debounce(int btn) {
  static uint16_t state = 0; 
  state = (state << 1) | digitalRead(btn) | 0xfe00; 
  return (state == 0xff00); 
}

// timeout parameters
const unsigned long timeLimit = 4000;
unsigned long startTime = 0;

// line following configuration
bool lineColor = false; // true - black and false - white

// helper function to read line based on color selection
int readLine(){
    if(linecolor){
         return qtra.readLineBlack(sensors1);
    }
    else{
         return qtra.readLineWhite(sensors1);
    }
}

// helper function to check sensor threshold based on color
bool isSensorOnLine(int sensorValue, int threshold) {
    if(isBlackLine) {
        return sensorValue > threshold;
    } else {
        return sensorValue < threshold;
    }
}

void setup() {
  Serial.begin(9600);
  
  // QTR8A sensor setup
  qtra.setTypeAnalog();
  qtra.setSensorPins((const uint8_t[]) {   
    A0, A1, A2, A3, A4, A5, A6, A7
  }, NUM_SENSORS);
  
  pinMode(sw1, INPUT);
  pinMode(sw2,INPUT);
  pinMode(10,OUTPUT);
  pinMode(led, OUTPUT);
  digitalWrite(10,HIGH);

  // Line color selection timeout
  startTime = millis();
  while ((millis()-startTime)<timeLimit){
    if(bounce(sw3)){
        lineColor=true;
        break;
    }
  }

  while(!debounce(sw1)){
    debounce(sw1);
  }

  digitalWrite(2,HIGH);
  delay(800);
  calibration();
  digitalWrite(2,LOW);

  // Choose maze solving strategy
  while (1) {
    if(debounce(sw1)){
      digitalWrite(13, HIGH);
      chr = 1;
      break; 
    }
    
    if(debounce(sw2)){
      digitalWrite(13, LOW);
      chr = 2;
      break;
    }
  }
}

void loop() {
   

  // Add ultrasonic sensor check here
  if (readDistance() < distanceThreshold) {
    brake(motor1, motor2); // Stop the motors if an obstacle is close
    while (readDistance() < distanceThreshold) {
      // Wait until obstacle is removed
      delay(100);
    }
  }


  digitalWrite(2,LOW);
  
  while(!debounce(sw2)){
    debounce(sw2);
  }

  delay(800);
  
  // Gradual speed increase at start
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  forward(motor1, motor2, 100);
  delay(40);
   
  maze();
}

void calibration() {
  for (int i = 0; i <= 100; i++) {
    if (i < 25 || i >= 75) {
      left(motor1, motor2, 400);
    }
    else {
      right(motor1, motor2, 400);
    }

    qtra.calibrate();
    delay(10);
  }
  
  brake(motor1, motor2);
 
  // Calculate threshold values for line detection
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibrationOn.minimum[i]);
    Serial.print(' ');
    thr[i] = (qtra.calibrationOn.minimum[i] + qtra.calibrationOn.maximum[i]) / 2;
  } 

  Serial.println();
  for (int i = 0; i < NUM_SENSORS; i++) {
    Serial.print(qtra.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println(thr[0]);
  Serial.println(thr[7]);
}

void maze() {
  while (1) {
    follow_Segment();
    
    brake(motor1, motor2);
    forward(motor1, motor2, 50);
    delay(30);
    brake(motor1, motor2);
    digitalWrite(led,HIGH);
    
    unsigned char found_left = 0;
    unsigned char found_straight = 0;
    unsigned char found_right = 0;

    readLine();

    if (isSensorOnLine(sensors1[7], thr[7])) {
        found_right = 1;
        if (isSensorOnLine(sensors1[1], thr[1])) {
            found_left = 1;
        }
    }
    if (isSensorOnLine(sensors1[0], thr[0])) {
        found_left = 1;
        if (isSensorOnLine(sensors1[6], thr[6])) {
            found_right = 1;
        }
    }

    forward(motor1, motor2, 230);
    delay(200);
    brake(motor1, motor2);
    readLine();

    for(int i = 1; i <= 6; i++) {
        if(isSensorOnLine(sensors1[i], thr[i])) {
            found_straight = 1;
            break;
        }
    }

    // Check for endpoint
    bool isEndpoint = true;
    for(int i = 1; i <= 6; i++) {
        if(!isSensorOnLine(sensors1[i], thr[i])) {
            isEndpoint = false;
            break;
        }
    }
    if(isEndpoint) break;

    // Select turn based on solving strategy
    if (chr == 1)
      dir = select_turnL(found_left, found_straight, found_right);
    else if (chr == 2)
      dir = select_turnR(found_right, found_straight, found_left);
    
    Serial.println(dir);
    turn(dir);
    Serial.println(dir);
    
    path[path_length] = dir;
    path_length++;
    simplify_path();
  }

  // Endpoint reached
  brake(motor1, motor2);
  forward(motor1, motor2, 80);
  delay(400);
  brake(motor1, motor2);

  for (int w = 0; w < path_length; w++) {
    Serial.print(path[w]);
    Serial.print(' ');
  }

  digitalWrite(led, HIGH);
  delay(4000);
  digitalWrite(led, LOW);

  while(!debounce(sw2)){
    debounce(sw2);
  }

  delay(800);
  
  // Gradual speed increase for return journey
  forward(motor1, motor2, 60);
  delay(40);
  forward(motor1, motor2, 80);
  delay(40);
  forward(motor1, motor2, 100);
  delay(40);
  
  // Follow simplified path
  while (1) {
    int k;
    for (k = 0; k < path_length; k++) {
      follow_Segment();
      forward(motor1, motor2, 180);
      delay(50);
      forward(motor1, motor2, 180);
      delay(200);
      brake(motor1, motor2);
      delay(5);
      turn(path[k]);
    }
    follow_Segment();
    brake(motor1, motor2);
    forward(motor1, motor2, 200);
    delay(400);
    brake(motor1, motor2);
    digitalWrite(led, HIGH);
    delay(4000);
    digitalWrite(led, LOW);
  }
}

void follow_Segment() {
  while (1) {
    digitalWrite(led, LOW);
    int position = readLine();
    
      // Check for obstacles during line following
    if (readDistance() < distanceThreshold) {
      brake(motor1, motor2); // Stop if obstacle detected
      while (readDistance() < distanceThreshold) {
        delay(100); // Wait until obstacle is removed
      }
    }
    
    int error = 3500 - position;
    int motorSpeed = kp * error + kd * (error - lastError);
    lastError = error;
    
    Serial.println(position);
    Serial.println(error);
    
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    
    Serial.println(rightMotorSpeed);
    Serial.println(leftMotorSpeed);
   
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);

    if(isSensorOnLine(sensors1[0], thr[0]) || isSensorOnLine(sensors1[7], thr[7])) {
      return;
    }
    
    for(int i = 1; i <= 6; i++) {
      if(!isSensorOnLine(sensors1[i], thr[i])) {
        return;
      }
    }
  }
}

void turn(char dir) {
  int line_position;
  
  switch (dir) {
    case 'L':
      left(motor1, motor2, 400);
      readLine();
      
      while (!isSensorOnLine(sensors1[0], thr[0])) {
        line_position = readLine(); 
      }
      
      left(motor1, motor2, 400);
      qtra.readLineWhite(sensors1);
      
      while (isSensorOnLine(sensors1[0], thr[0])) {
        line_position = readLine();
      }
      
      follow_segment1();
      brake(motor1, motor2);
      break;
      
    case 'R':
      right(motor1, motor2, 400);
      readLine();
      
      while(!isSensorOnLine(sensors1[7], thr[7])) {
        line_position = readLine();
      }
      
      right(motor1, motor2, 400);
      readLine();
      
      while(isSensorOnLine(sensors1[7], thr[7])) {
        line_position = readLine();
      }
      
      follow_segment1();
      brake(motor1, motor2);
      break;
      
    case 'B':
      right(motor1, motor2, 400);
      readLine();
      
      while (!isSensorOnLine(sensors1[7], thr[7])) {
        line_position = readLine();
      }
      
      right(motor1, motor2, 400);
      readLine();
      
      while (isSensorOnLine(sensors1[7], thr[7])) {
        line_position = readLine();
      }
      
      follow_segment3();
      brake(motor1, motor2);
      delay(50);
      follow_segment2();
      forward(motor1, motor2, 200);
      delay(40);
      break;
  }
}

// Fast PID for quick alignment after turn
void follow_segment1() {
  int Kp = 1;
  int Kd = 10;

    // Check for obstacles during line following
    if (readDistance() < distanceThreshold) {
      brake(motor1, motor2); // Stop if obstacle detected
      while (readDistance() < distanceThreshold) {
        delay(100); // Wait until obstacle is removed
      }
    }
    
  
  for (int j = 0; j < 30; j++) {
    int position = readLine();
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    
    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}

// Backward movement PID
void follow_segment2() {
  int Kp = 1;
  int Kd = 50;
  int baseSpeed = 90;
  int maxSpeed = 90;
  
    // Check for obstacles during line following
    if (readDistance() < distanceThreshold) {
      brake(motor1, motor2); // Stop if obstacle detected
      while (readDistance() < distanceThreshold) {
        delay(100); // Wait until obstacle is removed
      }
    }
     
  for (int j = 0; j < 70; j++) {
    int position = readLine();
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    int rightMotorSpeed = baseSpeed + motorSpeed;
    int leftMotorSpeed = baseSpeed - motorSpeed;
    
    rightMotorSpeed = constrain(rightMotorSpeed, 0, maxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, maxSpeed);
    
    motor1.drive(-rightMotorSpeed);
    motor2.drive(-leftMotorSpeed);
    delay(1);
  }
}

void follow_segment3() {
  int Kp = 1;
  int Kd = 10;

    // Check for obstacles during line following
    if (readDistance() < distanceThreshold) {
      brake(motor1, motor2); // Stop if obstacle detected
      while (readDistance() < distanceThreshold) {
        delay(100); // Wait until obstacle is removed
      }
    }
    
    
  
  for (int j = 0; j < 10; j++) {
    int position = readLine();
    int error = 3500 - position;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;
    
    int rightMotorSpeed = BaseSpeed - motorSpeed;
    int leftMotorSpeed = BaseSpeed + motorSpeed;
    
    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed);
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);

    motor1.drive(rightMotorSpeed);
    motor2.drive(leftMotorSpeed);
    delay(1);
  }
}

// Left-hand-on-wall strategy
char select_turnL(char found_left, char found_straight, char found_right) {
  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}

// Right-hand-on-wall strategy
char select_turnR(char found_right, char found_straight, char found_left) {
  if (found_right)
    return 'R';
  else if (found_straight)
    return 'S';
  else if (found_left)
    return 'L';
  else
    return 'B';
}

// Path simplification - removes unnecessary turns

  void simplify_path() {
  if (path_length < 3 || path[path_length - 2] != 'B')
    return;
    
  int total_angle = 0;
  int m;
  
  // Calculate total angle of turn sequence
  for (m = 1; m <= 3; m++) {
    switch (path[path_length - m]) {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }

  // Simplify turn sequence based on total angle
  total_angle = total_angle % 360;
  switch (total_angle) {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
  path_length -= 2;
}