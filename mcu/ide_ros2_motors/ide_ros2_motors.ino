#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <std_msgs/msg/string.h>

// Pin definitions
#define dirL 7
#define speedL 6
#define speedR 5
#define dirR 4
#define LED_PIN 13 // Built-in LED for status

// Encoder pins
#define ENC_L_PIN 2
#define ENC_L_PIN2 9
#define ENC_R_PIN 3
#define ENC_R_PIN2 10

// Encoder variables
volatile long encL = 0;
volatile long encR = 0;

// Robot parameters
#define WHEEL_RADIUS 0.033  // meters
#define WHEEL_SEPARATION 0.17  // meters
#define TICKS_PER_REVOLUTION 280  // Adjust based on your encoders (appears to be ~35 ticks per cm based on your code)
#define UPDATE_RATE 10  // Hz

// Robot state
float x = 0.0;
float y = 0.0;
float theta = 0.0;
float linear_velocity = 0.0;  // Target velocities from ROS
float angular_velocity = 0.0;

// PID control variables
// Left motor PID
float targetLeftVelocity = 0.0;  // Target velocity in rad/s
float actualLeftVelocity = 0.0;  // Actual velocity in rad/s
float prevLeftError = 0.0;
float leftErrorSum = 0.0;
int leftPwm = 0;

// Right motor PID
float targetRightVelocity = 0.0;  // Target velocity in rad/s
float actualRightVelocity = 0.0;  // Actual velocity in rad/s
float prevRightError = 0.0;
float rightErrorSum = 0.0;
int rightPwm = 0;

// PID constants - tune these for your specific motors
#define KP 5.0   // Proportional gain
#define KI 2.0   // Integral gain
#define KD 0.1   // Derivative gain
#define MAX_PWM 255
#define MIN_PWM 0
#define MAX_INTEGRAL 250.0  // Anti-windup

// ROS variables
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t pose_pub;
rcl_publisher_t log_pub;  // Custom log publisher
rcl_timer_t timer;
geometry_msgs__msg__Twist cmd_vel_msg;
geometry_msgs__msg__Pose2D pose_msg;
std_msgs__msg__String log_msg;  // Message for logging
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

uint32_t prev_update_time = 0;
int32_t prev_encL = 0;
int32_t prev_encR = 0;

// Buffer for log message
char log_buffer[100];

// Error handling macros
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Define PI
#ifndef PI
#define PI 3.14159265359
#endif

// Custom log function to replace RCUTILS_LOG_INFO
void log_info(const char* message) {
    // Copy message to the ROS message
    log_msg.data.data = (char*)message;
    log_msg.data.size = strlen(message);
    log_msg.data.capacity = strlen(message) + 1;
    
    // Publish the message
    RCSOFTCHECK(rcl_publish(&log_pub, &log_msg, NULL));
}

void error_loop() {
  while(1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// ---- Encoder Interrupt Functions ----
void e_encL() {
  if(digitalRead(ENC_L_PIN) == digitalRead(ENC_L_PIN2)) {
    encL++;
  } else {
    encL--;
  }
}

void e_encR() {
  if(digitalRead(ENC_R_PIN) == digitalRead(ENC_R_PIN2)) {
    encR--;
  } else {
    encR++;
  }
}

// ---- PID Velocity Control ----
void updatePID(float dt) {
    // Left motor PID
    float leftError = targetLeftVelocity - actualLeftVelocity;
    leftErrorSum += leftError * dt;
    
    // Anti-windup for integral term
    leftErrorSum = constrain(leftErrorSum, -MAX_INTEGRAL, MAX_INTEGRAL);
    
    float leftDerivative = (leftError - prevLeftError) / dt;
    float leftOutput = KP * leftError + KI * leftErrorSum + KD * leftDerivative;
    prevLeftError = leftError;
    
    // Right motor PID
    float rightError = targetRightVelocity - actualRightVelocity;
    rightErrorSum += rightError * dt;
    
    // Anti-windup for integral term
    rightErrorSum = constrain(rightErrorSum, -MAX_INTEGRAL, MAX_INTEGRAL);
    
    float rightDerivative = (rightError - prevRightError) / dt;
    float rightOutput = KP * rightError + KI * rightErrorSum + KD * rightDerivative;
    prevRightError = rightError;
    
    // Update PWM values with some deadzone compensation
    int leftDir = (targetLeftVelocity >= 0) ? 1 : -1;
    int rightDir = (targetRightVelocity >= 0) ? 1 : -1;
    
    // Convert PID output to PWM values
    leftPwm = constrain(abs(leftOutput), MIN_PWM, MAX_PWM);
    rightPwm = constrain(abs(rightOutput), MIN_PWM, MAX_PWM);
    
    // Apply PWM with direction
    move_(leftDir * map(leftPwm, 0, 255, 0, 100), 
          rightDir * map(rightPwm, 0, 255, 0, 100));
    
    // Log velocity errors for debugging
    sprintf(log_buffer, "L: %.2f->%.2f (e:%.2f) R: %.2f->%.2f (e:%.2f)", 
            targetLeftVelocity, actualLeftVelocity, leftError,
            targetRightVelocity, actualRightVelocity, rightError);
    log_info(log_buffer);
}

// ---- Motor Control Functions ----
void move_(int left, int right) {
  digitalWrite(dirL, left < 0);
  digitalWrite(dirR, right > 0);

  left = abs(constrain(left, -100, 100));
  right = abs(constrain(right, -100, 100));
  left = map(left, 0, 100, 0, 255);
  right = map(right, 0, 100, 0, 255);

  analogWrite(speedL, left);
  analogWrite(speedR, right);
}

void stop_() {
  move_(0, 0);
  
  // Reset PID variables when stopping
  leftErrorSum = 0;
  rightErrorSum = 0;
  prevLeftError = 0;
  prevRightError = 0;
}

// ---- ROS Callback Functions ----
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // Cap velocity commands to reasonable values
  linear_velocity = constrain(msg->linear.x, -5.0, 5.0);   // Max 0.3 m/s
  angular_velocity = constrain(msg->angular.z, -10.0, 10.0); // Max 2.0 rad/s
  
  // Turn on LED when robot is moving
  digitalWrite(LED_PIN, (fabs(linear_velocity) > 0.001 || fabs(angular_velocity) > 0.001) ? HIGH : LOW);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Calculate delta time
    uint32_t current_time = millis();
    float dt = (current_time - prev_update_time) / 1000.0;
    if (dt <= 0) dt = 0.01;
    
    // Calculate wheel velocities and distance traveled
    int32_t delta_encL = encL - prev_encL;
    int32_t delta_encR = encR - prev_encR;
    prev_encL = encL;
    prev_encR = encR;
    
    // Calculate actual velocities in rad/s
    actualLeftVelocity = (2.0 * PI * delta_encL) / (TICKS_PER_REVOLUTION * dt);
    actualRightVelocity = (2.0 * PI * delta_encR) / (TICKS_PER_REVOLUTION * dt);
    
    // Calculate distance traveled by each wheel
    float dl = 2.0 * PI * WHEEL_RADIUS * delta_encL / TICKS_PER_REVOLUTION;
    float dr = 2.0 * PI * WHEEL_RADIUS * delta_encR / TICKS_PER_REVOLUTION;
    
    float dc = (dl + dr) / 2.0; // Center distance
    
    // Update odometry
    if (fabs(dl - dr) < 1e-6) {
      // Straight line
      x += dc * cos(theta);
      y += dc * sin(theta);
    } else {
      // Arc movement
      float dtheta = (dr - dl) / WHEEL_SEPARATION;
      
      x += dc * cos(theta + dtheta/2.0);
      y += dc * sin(theta + dtheta/2.0);
      theta += dtheta;
      
      // Normalize theta
      while (theta > PI) theta -= 2*PI;
      while (theta < -PI) theta += 2*PI;
    }
    
    // Fill pose message
    pose_msg.x = x;
    pose_msg.y = y;
    pose_msg.theta = theta;
    
    // Publish pose
    RCSOFTCHECK(rcl_publish(&pose_pub, &pose_msg, NULL));
    
    // Update motor control based on target velocities
    if (fabs(linear_velocity) > 0.001 || fabs(angular_velocity) > 0.001) {
      // Convert target velocities to wheel velocities in rad/s
      targetLeftVelocity = (linear_velocity - angular_velocity * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
      targetRightVelocity = (linear_velocity + angular_velocity * WHEEL_SEPARATION / 2.0) / WHEEL_RADIUS;
      
      // Apply PID control to maintain target velocities
      updatePID(dt);
    } else {
      targetLeftVelocity = 0;
      targetRightVelocity = 0;
      stop_();
    }
    
    prev_update_time = current_time;
  }
}

// Memory management configuration
extern "C" int clock_gettime(clockid_t unused, struct timespec *tp) {
  (void)unused;
  uint64_t m = micros();
  tp->tv_sec = m / 1000000;
  tp->tv_nsec = (m % 1000000) * 1000;
  return 0;
}

void setup() {
  // Initialize pins
  pinMode(ENC_L_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN, INPUT_PULLUP);
  pinMode(ENC_R_PIN2, INPUT_PULLUP);
  pinMode(ENC_L_PIN2, INPUT_PULLUP);
  pinMode(dirL, OUTPUT);
  pinMode(speedL, OUTPUT);
  pinMode(dirR, OUTPUT);
  pinMode(speedR, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  // Start with LED on to indicate we're in setup
  digitalWrite(LED_PIN, HIGH);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_L_PIN), e_encL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_PIN), e_encR, CHANGE);

  // Initialize serial and micro-ROS
  Serial.begin(115200);
  delay(2000); // Give serial time to connect
  
  // Initialize micro-ROS with explicit serial configuration
  rmw_uros_set_custom_transport(
    true,
    (void *) &Serial,
    arduino_transport_open,
    arduino_transport_close,
    arduino_transport_write,
    arduino_transport_read
  );
  
  // Flash LED pattern to indicate start of ROS initialization
  for(int i=0; i<3; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(200);
    digitalWrite(LED_PIN, HIGH);
    delay(200);
  }

  allocator = rcl_get_default_allocator();
  
  // Initialize ROS node with minimal options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "arduino_robot", "", &support));
  
  // Create subscriber for cmd_vel
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));
  
  // Create publisher for 2D pose
  RCCHECK(rclc_publisher_init_default(
    &pose_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
    "pose"));
  
  // Create publisher for log messages
  RCCHECK(rclc_publisher_init_default(
    &log_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "robot_log"));
  
  // Initialize log message
  log_msg.data.capacity = 100;
  log_msg.data.size = 0;
  log_msg.data.data = (char*) malloc(log_msg.data.capacity * sizeof(char));
  
  // Create timer for updates
  const unsigned int timer_period = 1000 / UPDATE_RATE; // milliseconds
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_period),
    timer_callback));
  
  // Initialize executor with minimum handles
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Initialize time variable
  prev_update_time = millis();
  
  // Successful initialization pattern
  for(int i=0; i<5; i++) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    digitalWrite(LED_PIN, HIGH);
    delay(50);
  }
  
  // Send initialization log
  log_info("Robot initialized with PID velocity control");
}

void loop() {
  // Use a short spin timeout to prevent blocking too long
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
  delay(10);
}
