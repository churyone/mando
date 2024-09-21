#include <ros.h>
#include <std_msgs/Float32.h>
#include <Servo.h>

ros::NodeHandle nh;

float throttle = 0.0;
float servo_value = 0.0;

Servo myServo;

const int throttleLED = 13; 
const int servoLED = 12;    

void throttleCallback(const std_msgs::Float32& msg) {
  throttle = msg.data;
  
  char log_msg[50];
  snprintf(log_msg, sizeof(log_msg), "Received Throttle: %f", throttle);
  
  nh.loginfo(log_msg);
  
  Serial.print("Throttle: ");
  Serial.println(throttle);
  
  digitalWrite(throttleLED, HIGH); 
  delay(100);                       
  digitalWrite(throttleLED, LOW);  
}

void servoCallback(const std_msgs::Float32& msg) {
  servo_value = msg.data;
  
  char log_msg[50];
  snprintf(log_msg, sizeof(log_msg), "Received Servo: %f", servo_value);
  
  nh.loginfo(log_msg);
  
  Serial.print("Servo: ");
  Serial.println(servo_value);
  
  int angle = map(servo_value, -100, 100, 0, 180);
  myServo.write(angle);
  
  // LED 깜박임
  digitalWrite(servoLED, HIGH); 
  delay(100);                    
  digitalWrite(servoLED, LOW);  
}

// Subscribers
ros::Subscriber<std_msgs::Float32> throttle_sub("throttle", throttleCallback);
ros::Subscriber<std_msgs::Float32> servo_sub("servo", servoCallback);

void setup() {
  pinMode(throttleLED, OUTPUT);
  pinMode(servoLED, OUTPUT);
  
  delay(5000); 
  Serial.begin(57600);
  Serial.println("Starting ROS node...");
  
  nh.initNode();
  nh.subscribe(throttle_sub);
  nh.subscribe(servo_sub);
  
  myServo.attach(9);
  
  while (!nh.connected()) {
    Serial.println("Waiting for ROS connection...");
    nh.spinOnce();
    delay(100);
  }
  
  nh.loginfo("Arduino connected to ROS");
}

void loop() {
  nh.spinOnce();
  delay(10);
}
