#define SWITCH1_PIN 2
#define SWITCH2_PIN 3

#define MOTOR1_IN1 4
#define MOTOR1_IN2 5

#define MOTOR2_IN1 6
#define MOTOR2_IN2 7

#define MOTOR3_IN1 8
#define MOTOR3_IN2 9

void setup() {
  Serial.begin(9600);
  pinMode(SWITCH1_PIN, INPUT_PULLUP);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "enable") {
      // Drive the first and second motor forward
      Serial.println("Bucket Lowering ...");
      digitalWrite(MOTOR1_IN1, HIGH);
      digitalWrite(MOTOR1_IN2, LOW);
      digitalWrite(MOTOR2_IN1, HIGH);
      digitalWrite(MOTOR2_IN2, LOW);
      
      while (digitalRead(SWITCH1_PIN) == HIGH) {}
      // Stop the first and second motor on first switch 
      Serial.println("Bucket Lowered!!");
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);
      
      // Drive the third motor forward for 10 seconds
      Serial.println("Claw Initiated ...");
      digitalWrite(MOTOR3_IN1, HIGH);
      digitalWrite(MOTOR3_IN2, LOW);
      delay(10000);
      // Stop the second and third motor
  
      digitalWrite(MOTOR3_IN1, LOW);
      digitalWrite(MOTOR3_IN2, LOW);
      Serial.println("Claw Stopped !!");

      // Reverse the direction of the first motor
      Serial.println("Bucket Reversing ...");
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, HIGH);
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, HIGH);
      
      while (digitalRead(SWITCH2_PIN) == HIGH) {}
      Serial.println("Bucket Reversed !!");
      // Stop the first motor
      digitalWrite(MOTOR1_IN1, LOW);
      digitalWrite(MOTOR1_IN2, LOW);
      digitalWrite(MOTOR2_IN1, LOW);
      digitalWrite(MOTOR2_IN2, LOW);

      // Send the "complete" message to notify action is complete
      Serial.println("complete");
    }
    else {
      Serial.println("Invalid Command")
    }
  }
}
