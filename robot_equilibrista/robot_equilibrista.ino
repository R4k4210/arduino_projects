#include <LMotorController.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 30

// Interrupt Pin
const int INTERRUPT = 2;

// MPU instance
MPU6050 mpu;

// Bluetooth instance
SoftwareSerial BT(12,11);

// Bluetooth constants
const int EN = 9;
const int VCC = 8;
const byte BT_CONFIG_ON = 0;
const byte DEBUG_YPR = 1;
String message;

// Motors control
int ENA = 5;
int IN1 = 6;
int IN2 = 7;
int IN3 = 9;
int IN4 = 8;
int ENB = 10;
double left_speed_motor = 0.45; //double left_speed_motor = 0.3;
double right_speed_motor = 0.45; //double right_speed_motor = 0.3;

// MPU controls
bool dmp_ready = false; // set true if DMP init was successful
uint8_t mpu_int_status; // holds actual interrupt status byte from MPU
uint8_t dev_status; // return status after each device operation (0 = success, !0 = error)
uint16_t packet_size; // expected DMP packet size (default is 42 bytes)
uint16_t fifo_count; // count of all bytes currently in FIFO
uint8_t fifo_buffer[64]; // FIFO storage buffer

// Orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
int accel[3]; // [x, y, z] accel container
int gyro[3]; // [x, y, z] accel container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// Initial status 'stopped'
int state = 'g';

// PID contorls (Modify these values based on design)
double breakeven = 177;
double Kp = 62;  //double Kp = 60; 
double Kd = 2.6;  //double Kd = 2.2;  
double Ki = 220;  //double Ki = 270;

double original_setpoint = breakeven;   //double originalSetpoint = 172.50;
double setpoint = original_setpoint;
double moving_angle_offset = 0.1;
double input, output;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motor_speed_factor_left = left_speed_motor; //double motor_speed_factor_left = 0.6;
double motor_speed_factor_right = right_speed_motor; //double motor_speed_factor_right = 0.5;

LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motor_speed_factor_left, motor_speed_factor_right);

volatile bool mpu_interrupt = false; // indicates whether MPU interrupt pin has gone high

void setup(){
  bt_init();
  
  // Serial config
  Serial.begin(9600);

  if(BT_CONFIG_ON){
    Serial.print("Waiting for AT Commands:");
  }else{
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
    #endif
  
    mpu.initialize();
    dev_status = mpu.dmpInitialize();

    // Supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if(dev_status == 0){
      // Turn on the DMP, now that it's ready
      mpu.setDMPEnabled(true);
      // Enable Arduino interrupt detection
      attachInterrupt(digitalPinToInterrupt(INTERRUPT), dmpDataReady, RISING);
      mpu_int_status = mpu.getIntStatus();
      // Set our DMP Ready flag so the main loop() function knows it's okay to use it
      dmp_ready = true;
      // get expected DMP packet size for later comparison
      packet_size = mpu.dmpGetFIFOPacketSize();
      //setup PID
      pid.SetMode(AUTOMATIC);
      pid.SetSampleTime(10);
      pid.SetOutputLimits(-255, 255); 
    }else{
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(dev_status);
      Serial.println(F(")"));
    }
  }  
}

void loop(){
  if(BT_CONFIG_ON){
    config_bt();
  } else {
    //read_bt();
    set_direction();
    
    // If programming failed, don't try to do anything
    if(!dmp_ready) return;
    
    // Wait for MPU interrupt or extra packet(s) available
    while(!mpu_interrupt && fifo_count < packet_size){
      // No mpu data - performing PID calculations and output to motors 
      pid.Compute();
      motorController.move(output, MIN_ABS_SPEED);
    }

    if(DEBUG_YPR){
      //draw_ypr();
      draw_accel();
      draw_gyro();
    }
      
    // Reset interrupt flag and get INT_STATUS byte
    mpu_interrupt = false;
    mpu_int_status = mpu.getIntStatus();
    
    // Get current FIFO count
    fifo_count = mpu.getFIFOCount();

    // Check for overflow (this should never happen unless our code is too inefficient)
    if((mpu_int_status & 0x10) || fifo_count == 1024){
      // Reset so we can continue cleanly
      mpu.resetFIFO();
      Serial.println(F("FIFO overflow!"));
      // Otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if(mpu_int_status & 0x02){
      // Wait for correct available data length, should be a VERY short wait
      while (fifo_count < packet_size) {
        fifo_count = mpu.getFIFOCount();
      }
      
      // Read a packet from FIFO
      mpu.getFIFOBytes(fifo_buffer, packet_size);
     
      // Track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifo_count -= packet_size;

      mpu.dmpGetQuaternion(&q, fifo_buffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpu.dmpGetAccel(accel, fifo_buffer);
      mpu.dmpGetGyro(gyro, fifo_buffer);
      input = ypr[1] * 180/M_PI + 180;
    }
  }
}

void draw_ypr() {
  double yaw = ypr[0] * 180/M_PI + 180;
  double pitch = ypr[1] * 180/M_PI + 180;
  double roll = ypr[2] * 180/M_PI + 180;
  Serial.print(yaw);
  Serial.print(",");
  Serial.println(pitch);
  Serial.print(",");
  Serial.print(roll);  
  Serial.println("");
}

void draw_accel() {
  double accel_x = accel[0];
  double accel_y = accel[1];
  double accel_z = accel[2];
  Serial.print(accel_x);
  Serial.print(",");
  Serial.println(accel_y);
  Serial.print(",");
  Serial.print(accel_z);  
  Serial.println("");
}

void draw_gyro() {
  double gyro_x = gyro[0];
  double gyro_y = gyro[1];
  double gyro_z = gyro[2];
  Serial.print(gyro_x);
  Serial.print(",");
  Serial.println(gyro_y);
  Serial.print(",");
  Serial.print(gyro_z);  
  Serial.println("");
}

void dmpDataReady(){
  mpu_interrupt = true;
}

void clean_bt(){
  message = "";
}

void bt_init() {
  pinMode(EN, OUTPUT);
  pinMode(VCC, OUTPUT);
  digitalWrite(EN, HIGH);
  digitalWrite(VCC, HIGH);
  delay(500);
  BT.begin(9600);
}

void set_direction(){
  if(state == 'a'){ // Forward
    setpoint = (setpoint + 0.5);
    state = 'g';
  }
  if(state == 'b'){ // Left
    state = 'g';
  }
  if(state == 'c'){ // Stop
    setpoint = breakeven; 
    state = 'g';
  }
  if(state == 'd'){ // Right
    state = 'g';
  } 
  if(state == 'e'){ // Backwards
    setpoint = (setpoint - 0.5); 
    state = 'g';
  }
}

void read_bt() {
  int i = 0;
  bool end_msg = false;
  while(BT.available()){
    delay(10);
    char msg = BT.read();
    if(msg == 0){ //Error here
      Serial.print("End of message");
      end_msg = true;
      break;
    }
    //message[i] = msg;
    message += msg;
    Serial.print(">");
    Serial.println(msg);
    i++;
  }
  i = 0;
}

void config_bt() {
  if(Serial.available()){
    BT.write(Serial.read());
  }

  if(BT.available()){
    Serial.write(BT.read());
  }
}
