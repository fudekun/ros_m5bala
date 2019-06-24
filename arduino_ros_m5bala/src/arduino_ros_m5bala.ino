#include <M5Stack.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <float.h>
#include <WiFi.h>
#include <Wire.h>
#include "M5Bala.h"


#define NUM_OF_PULSES_PER_WHEEL_REVOLUTION 2560.0     // count up per int8=>256pulses/20deg(16pices)
#define MAX_SPEED_OF_MOVE_MS 0.069                    // m/s
#define MAX_SPEED_OF_ROTATE_RS 0.417                  // rad/s
#define DIAMETER_WHEEL 0.04356                        // m
#define TREAD_WHEEL 0.040                             // m


const char* ssid = "";
const char* password = "";
WiFiClient client;
IPAddress server(192, 168, 10, 16); //ROS core IP adress

M5Bala m5bala(Wire);

int16_t joystick_X;
int16_t joystick_Y;
float enc0 = 0.0;
float enc1 = 0.0;
float accX = 0.0;
float accY = 0.0;
float accZ = 0.0;
float gyroX = 0.0;
float gyroY = 0.0;
float gyroZ = 0.0;
float angleX = 0.0;
float angleY = 0.0;
float angleZ = 0.0;

void setRPY(const float& roll, const float& pitch, const float& yaw, float date[4]) {
  float halfYaw = float(yaw) * float(0.5);  
  float halfPitch = float(pitch) * float(0.5);  
  float halfRoll = float(roll) * float(0.5);  
  float cosYaw = cos(halfYaw);
  float sinYaw = sin(halfYaw);
  float cosPitch = cos(halfPitch);
  float sinPitch = sin(halfPitch);
  float cosRoll = cos(halfRoll);
  float sinRoll = sin(halfRoll);
  date[0] = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
  date[1] = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
  date[2] = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  date[3] = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
}


class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);   
    }
    int read() {
      return client.read();      
    }
    void write(uint8_t* data, int length) {
      client.write(data, length);
    }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

void setupWiFi() {
  WiFi.begin(ssid, password);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    while (1) delay(500);
  }
  M5.Lcd.println(WiFi.localIP());
}


ros::NodeHandle_<WiFiHardware> nh;

void messageCb(const geometry_msgs::Twist& twist) {
  const float linear_x = twist.linear.x;
  const float angle_z = -1.0 * twist.angular.z;
  float accl_main = 0.0;
  float hndl_main = 0.0;
  float accl_cnt = 0.0;
  float hndl_cnt = 0.0;
  if (linear_x >= 0.0) {
    accl_main = 0.5 * (linear_x * 100.0/MAX_SPEED_OF_MOVE_MS - 60.0);
    hndl_cnt = 0.055 * linear_x * 100.0/MAX_SPEED_OF_MOVE_MS;
  } else {
    accl_main = 0.3 * (linear_x * 100.0/MAX_SPEED_OF_MOVE_MS - 100.0);
    hndl_cnt = 0.05 * linear_x * 100.0/MAX_SPEED_OF_MOVE_MS;
  }
  if (angle_z >= 0.0) {
    hndl_main = 0.3 * (angle_z * 100.0/MAX_SPEED_OF_ROTATE_RS);
    accl_cnt = 0.0;
  } else {
    hndl_main = 0.3 * (angle_z * 100.0/MAX_SPEED_OF_ROTATE_RS);
    accl_cnt = -0.5 * (angle_z * 100.0/MAX_SPEED_OF_ROTATE_RS);
  }
  // move
  joystick_Y = constrain((int16_t)(accl_main + accl_cnt), -70, 30);
  // rotate
  joystick_X = constrain((int16_t)(hndl_main + hndl_cnt), -45, 45);
  m5bala.move(joystick_Y);
  m5bala.rotate(joystick_X);
}
ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", &messageCb);

nav_msgs::Odometry odom_msg;
tf::tfMessage odom_tf_msg;
sensor_msgs::Imu imu_msg;
tf::tfMessage imu_tf_msg;
ros::Publisher pub_odom("odom", &odom_msg);
//ros::Publisher pub_odom_tf("tf", &odom_tf_msg);
ros::Publisher pub_imu("imu_data", &imu_msg);
//ros::Publisher pub_imu_tf("tf", &imu_tf_msg);
void pub_th(void* arg) {
  geometry_msgs::TransformStamped odom_trans;
  geometry_msgs::TransformStamped imu_trans;

  char odom[] = "odom";
  char base_footprint[] = "base_footprint";
  char base_link[] = "base_link";
  char imu_link[] = "imu_link";
  

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;
  
  ros::Time current_time = nh.now();
  ros::Time last_time = nh.now();
  float current_enc0_l;
  float last_enc0_l;
  current_enc0_l = last_enc0_l = enc0;
  float current_enc1_r;
  float last_enc1_r;
  current_enc1_r = last_enc1_r = enc1;
  while (1) {
    current_time = nh.now();
    float dt = current_time.toSec() - last_time.toSec();
    current_enc0_l = enc0;
    current_enc1_r = enc1;
    float delta_enc0_l = current_enc0_l - last_enc0_l;
    float delta_enc1_r = current_enc1_r - last_enc1_r;
    double delta_ll = DIAMETER_WHEEL * 3.1415926535897931 * delta_enc0_l;
    double delta_lr = DIAMETER_WHEEL * 3.1415926535897931 * delta_enc1_r;
    float delta_l = (delta_lr + delta_ll) / 2.0;
    float delta_th = (delta_lr - delta_ll) / (2.0 * TREAD_WHEEL);
    float delta_rho = delta_l / delta_th;
    float delta_x = delta_l * cos(th + (delta_th / 2.0));
    float delta_y = delta_l * sin(th + (delta_th / 2.0));

    x += delta_x;
    y += delta_y;
    th += delta_th;

    float date[4];
    setRPY(0.0, 0.0, th, date);
    geometry_msgs::Quaternion odom_quat;
    odom_quat.x = date[0];
    odom_quat.y = date[1];
    odom_quat.z = date[2];
    odom_quat.w = date[3];

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = odom;
    odom_trans.child_frame_id = base_footprint;
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
//    odom_tf_msg.transforms_length = 1;
//    odom_tf_msg.transforms = &odom_trans;

    //next, we'll publish the odometry message over ROS
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = odom;

    //set the position
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    //set the velocity
    odom_msg.child_frame_id = base_footprint;
    odom_msg.twist.twist.linear.x = delta_x / dt;
    odom_msg.twist.twist.linear.y = delta_y / dt;
    odom_msg.twist.twist.angular.z = delta_th / dt;
    odom_msg.pose.covariance[0] = 20.0;
    odom_msg.pose.covariance[7] = 20.0;
    odom_msg.pose.covariance[14] = FLT_MAX;
    odom_msg.pose.covariance[21] = FLT_MAX;
    odom_msg.pose.covariance[28] =FLT_MAX;
    odom_msg.pose.covariance[35] = 50.0;
    odom_msg.twist.covariance[0] = .1;
    odom_msg.twist.covariance[7] = .1;
    odom_msg.twist.covariance[14] = 1000000000;
    odom_msg.twist.covariance[21] = 1000000000;
    odom_msg.twist.covariance[28] = 1000000000;
    odom_msg.twist.covariance[35] = .1;    


    /////////////////////////////////
    // IMU
    imu_msg.header.stamp = current_time;
    imu_msg.header.frame_id = imu_link;
    imu_msg.orientation_covariance[0] =
    imu_msg.orientation_covariance[4] =
    imu_msg.orientation_covariance[8] = 0.017453292519943295;
    imu_msg.linear_acceleration_covariance[0] =
    imu_msg.linear_acceleration_covariance[4] =
    imu_msg.linear_acceleration_covariance[8] = 0.023145;
    imu_msg.angular_velocity_covariance[0] = 
    imu_msg.angular_velocity_covariance[4] = 
    imu_msg.angular_velocity_covariance[8] = 0.0010621;
    imu_msg.linear_acceleration.x = accX;
    imu_msg.linear_acceleration.y = accY;
    imu_msg.linear_acceleration.z = accZ;
    imu_msg.angular_velocity.x = gyroX;
    imu_msg.angular_velocity.y = gyroY;
    imu_msg.angular_velocity.z = gyroZ;
    geometry_msgs::Quaternion imu_quat;
    float date_imu[4];
    setRPY(angleX, angleY, angleZ, date_imu);
    imu_quat.x = date[0];
    imu_quat.y = date[1];
    imu_quat.z = date[2];
    imu_quat.w = date[3];
    imu_msg.orientation = imu_quat;
    //////////////////////////

//    imu_trans.header.stamp = current_time;
//    imu_trans.header.frame_id = imu_link;
//    imu_trans.child_frame_id = imu_link;
//    imu_trans.transform.translation.x = 0.0;
//    imu_trans.transform.translation.y = 0.0;
//    imu_trans.transform.translation.z = 0.035;
//    imu_trans.transform.rotation = imu_quat;
//    imu_tf_msg.transforms_length = 1;
//    imu_tf_msg.transforms = &imu_trans;

    last_enc0_l = current_enc0_l;
    last_enc1_r = current_enc1_r;
    last_time = current_time;
    
    // about 60Hz
    delay(1000/200);
  }
}

void setup() {
  // Power ON Stabilizing...
  //delay(500);
  M5.begin();

  // Init I2C
  Wire.begin();
  Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
  delay(500);

  // Display info
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("M5Stack Balance Mode start");
  
  setupWiFi();

  // for ROS
  nh.initNode();
  nh.subscribe(sub_twist);
  nh.advertise(pub_odom);
//  nh.advertise(pub_odom_tf);
  nh.advertise(pub_imu);
//  nh.advertise(pub_imu_tf);

  xTaskCreatePinnedToCore(pub_th, "pub_th", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(m5_loop_th, "m5_loop_th", 4096, NULL, 1, NULL, 1);
}

void m5_loop_th(void* arg) {
  // Init M5Bala
  m5bala.begin();
  m5bala.imu->setGyroOffsets(0.47, 0.6, -0.86);
  delay(500);
  m5bala.move(-30);
  m5bala.rotate(0);
  
  while (1) {
    // M5Bala run
    m5bala.run();
    // M5 Loop
    M5.update();

    // encoder update
    if (m5bala.getOut0() != 0 && abs(m5bala.getOut0()) < 200) {
      enc0 += (m5bala.getSpeed0()/NUM_OF_PULSES_PER_WHEEL_REVOLUTION);
    }
    if (m5bala.getOut1() != 0 && abs(m5bala.getOut1()) < 200) {
      enc1 += (m5bala.getSpeed1()/NUM_OF_PULSES_PER_WHEEL_REVOLUTION);
    }
    
    accX = -1.0 * m5bala.imu->getAccY() * 9.81;
    accY = 1.0 * m5bala.imu->getAccX() * 9.81;
    accZ = -1.0 * m5bala.imu->getAccZ() * 9.81;
    gyroX = -1.0 * m5bala.imu->getGyroY() * (3.1415926535897931/180.0);
    gyroY = -1.0 * m5bala.imu->getGyroX() * (3.1415926535897931/180.0);
    gyroZ = -1.0 * m5bala.imu->getGyroZ() * (3.1415926535897931/180.0);
    angleX = 1.0 * m5bala.imu->getAngleY();
    angleY = -1.0 * m5bala.imu->getAngleX();
    angleZ = 1.0 * m5bala.imu->getAngleZ();

    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("g_x: %f", gyroX);
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("g_y: %f", gyroY);
    M5.Lcd.setCursor(0, 60);
    M5.Lcd.printf("g_z: %f", gyroZ);
    M5.Lcd.setCursor(0, 80);
    M5.Lcd.printf("_x: %2f", angleX);
    M5.Lcd.setCursor(0, 100);
    M5.Lcd.printf("_y: %2f", angleY);
    M5.Lcd.setCursor(0, 120);
    M5.Lcd.printf("_z: %2f", angleZ);
    M5.Lcd.setCursor(0, 140);
    M5.Lcd.printf("a_x: %f", accX);
    M5.Lcd.setCursor(0, 160);
    M5.Lcd.printf("a_y: %f", accY);
    M5.Lcd.setCursor(0, 180);
    M5.Lcd.printf("a_z: %f", accZ);
  }
}


void loop() {
  // ROS
  pub_odom.publish(&odom_msg);
  nh.spinOnce();
//  pub_odom_tf.publish(&odom_tf_msg);
//  nh.spinOnce();
  pub_imu.publish(&imu_msg);
  nh.spinOnce();
//  pub_imu_tf.publish(&imu_tf_msg);
//  nh.spinOnce();
  // about 60Hz
  delay(1000/30);
}
