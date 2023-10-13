#include <Servo.h>

#define SERVO_BASE_PIN 3
#define SERVO_SHOULDER_PIN 5
#define SERVO_ELBOW_PIN 9
#define SERVO_WRIST_PIN 10
//Initialize
#define BASE_START 0
#define SHOULDER_START 90
#define ELBOW_START 45
#define WRIST_START 0

Servo base;  
Servo shoulder;  
Servo elbow;  
Servo wrist;

int angles[4] = {0, 0, 0, 0};
int last_pose[4] = {0, 0, 0, 0};
int angle_idx = 0;

void setup() {
  base.attach(SERVO_BASE_PIN);
  shoulder.attach(SERVO_SHOULDER_PIN);
  elbow.attach(SERVO_ELBOW_PIN);
  wrist.attach(SERVO_WRIST_PIN);

  base.write(BASE_START);
  shoulder.write(SHOULDER_START);
  elbow.write(ELBOW_START);
  wrist.write(WRIST_START);

  Serial.begin(115200);
  Serial.setTimeout(1);

  last_pose[0] = base.read();
  last_pose[1] = shoulder.read();
  last_pose[2] = elbow.read();
  last_pose[3] = wrist.read();
  
}
void loop() {
  if (Serial.available())
  {
    char chr = Serial.read();
    if (chr >= '0' && chr <= '9') {
      angles[angle_idx] = angles[angle_idx] * 10 + (chr - '0');
    }
    else if(chr == ',')
    {
      int targetpose = last_pose[angle_idx];
        while(targetpose != angles[angle_idx]){
          if(targetpose < angles[angle_idx]){
            targetpose++;
          } else if(targetpose > angles[angle_idx]){
            targetpose--;
          }
          if(angle_idx == 0){
            base.write(targetpose);
          }
          else if(angle_idx == 1){
            shoulder.write(targetpose);
          }
          else if(angle_idx == 2){
            elbow.write(targetpose);
          }
          else if(angle_idx == 3){
            wrist.write(targetpose);
          }
          delay(10);
        }
      last_pose[angle_idx] = angles[angle_idx];
      angle_idx++;
    }
    else if(chr == '.'){
      delay(1000);
      angle_idx = 0;
      for(int i=0;i<4;i++){
        angles[i] = 0;
      }
    }
  }
}