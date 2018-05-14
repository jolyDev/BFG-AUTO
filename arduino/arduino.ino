
#include <SoftwareSerial.h>
#include <Servo.h>

Servo constServo;
Servo angleServo;

int servoAngle = 45;

unsigned long prevUpdate;

enum class State {
  Pos,
  Neg,
  Stop,
};

State angleState = State::Stop;
State constState = State::Stop;

void setup() {
  // put your setup code here, to run once:

  pinMode(13, OUTPUT);
  constServo.attach(9);
  angleServo.attach(10);

  angleServo.write(45);
  constServo.write(90);

  
  Serial.begin(9600);
  while (!Serial) {}

  prevUpdate = millis();
}

void loop() {
  if (Serial.available() >= 3) {
    char fst;
    char snd;
    
    char control = Serial.read();
    if (control != 'a') {
      return;  
    }
    fst = Serial.read();
    snd = Serial.read();

    switch (fst) {
      case '0':
        
        //constantServo.write(95);
        constState = State::Pos;
        break;

      case '2':
        
        //constantServo.write(85);
        constState = State::Neg;
        break;

        default:
        
        //constantServo.write(90);
        constState = State::Stop;
        break;
    }

    switch(snd) {
      case '0':
        angleState = State::Pos;
        break;

      case '1':
        angleState = State::Stop;
        break;

      case '2':
        angleState = State::Neg;
        break;
      }

  }

  switch (constState) {
    case State::Pos:
      constServo.write(94);
      break;

    case State::Neg:
      constServo.write(85);
      break;
  
    case State::Stop:
      constServo.write(90);
      break;
  }

  unsigned long currentTime = millis();
    if (currentTime - prevUpdate >= 100) {
      prevUpdate = currentTime;
  
      switch (angleState) {
        case State::Neg:
          if (servoAngle > 10) {
            servoAngle -= 1;
          }
          break;

          case State::Pos:
            if (servoAngle < 165) {
              servoAngle += 1;
            }
            break;

          default:
            break;
      }
    }
    
    angleServo.write(servoAngle);
}
