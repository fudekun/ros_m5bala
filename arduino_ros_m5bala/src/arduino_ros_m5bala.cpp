#include <Arduino.h>
#include <M5Stack.h>
#include <ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/CompressedImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/ColorRGBA.h>
#include <float.h>
#include <WiFi.h>
#include <Wire.h>
#include "M5Bala.h"
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <Adafruit_NeoPixel.h>


#define NUM_OF_PULSES_PER_WHEEL_REVOLUTION 2560.0     // count up per int8=>256pulses/20deg(16pices)
#define MAX_SPEED_OF_MOVE_MS 0.069                    // m/s
#define MAX_SPEED_OF_ROTATE_RS 0.417                  // rad/s
#define DIAMETER_WHEEL 0.04356                        // m
#define TREAD_WHEEL 0.040                             // m

#define M5STACK_FIRE_NEO_NUM_LEDS 10                  // Left(4~9) Right(0~4)
#define M5STACK_FIRE_NEO_DATA_PIN 15
#define M5STACK_FIRE_MICROPHONE_PIN 34
#define M5STACK_FIRE_SPEAKER_PIN 25


void setRPY(const float& roll, const float& pitch, const float& yaw, float date[4]);
void setupWiFi();
void cmd_vel_cb(const geometry_msgs::Twist& twist);
void image_data_cb(const sensor_msgs::CompressedImage& im);
void led_left_cb(const std_msgs::ColorRGBA& rgba);
void led_right_cb(const std_msgs::ColorRGBA& rgba);
void pub_calc_th(void* arg);
void pub_th(void* arg);
void sub_th(void* arg);
void robot_run();
void neopixel_ctrl(int first_cnt, int last_cnt, int r, int g, int b, int w);

WiFiClient client;
const char* ssid = "";
const char* password = "";
char server[] = "coder01";
//IPAddress server(192, 168, 10, 16); //ROS core IP adress

M5Bala m5bala(Wire);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(M5STACK_FIRE_NEO_NUM_LEDS, M5STACK_FIRE_NEO_DATA_PIN, NEO_GRB + NEO_KHZ400);
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

class WiFiHardware {
  public:
    WiFiHardware() {};
    void init() {
      client.connect(server, 11411);
    }
    int read() {
      return client.read();
    }
    int read(uint8_t* data, int length) {
      return client.read(data, length);
    }
    void write(uint8_t* data, int length) {
      client.write(data, length);
    }
    unsigned long time() {
      return millis(); // easy; did this one for you
    }
};

class SimpleRate {
  private:
    int hz_;
    unsigned long start_;
    int expected_cycle_time_;
    unsigned long expected_end;
    unsigned long actual_end;
    int sleep_time;
    uint32_t actual_cycle_time_;
  public:
    SimpleRate(int hz) {
      hz_ = hz;
      start_ = millis();
      expected_cycle_time_ = (int)((1.0 / hz) * 1000);
    }
    uint32_t sleep() {
      expected_end = start_ + expected_cycle_time_;
      actual_end = millis();
      sleep_time = expected_end - actual_end;
      actual_cycle_time_ = actual_end - start_;
      start_ = expected_end;
      if(sleep_time <= 0) {
        return 0;
      }
      delay((uint32_t)sleep_time);
      return sleep_time;
    }
};

// ROS Node
ros::NodeHandle_<WiFiHardware> nh;
// ROS Pub Msg
nav_msgs::Odometry odom_msg;
sensor_msgs::Imu imu_msg;
// ROS Pub Topic
ros::Publisher pub_odom("odom", &odom_msg);
ros::Publisher pub_imu("imu_data", &imu_msg);
// ROS Sub Topic
ros::Subscriber<geometry_msgs::Twist> sub_twist("cmd_vel", &cmd_vel_cb);
ros::Subscriber<sensor_msgs::CompressedImage> sub_img("usb_cam/image_raw/compressed", &image_data_cb);
ros::Subscriber<std_msgs::ColorRGBA> sub_led_left("led/left", &led_left_cb);
ros::Subscriber<std_msgs::ColorRGBA> sub_led_right("led/right", &led_right_cb);

void setup() {
  Serial.begin(115200);
  // Power ON Stabilizing...
  delay(500);
  M5.begin();

  // Init I2C
  Wire.begin();
  Wire.setClock(400000UL);  // Set I2C frequency to 400kHz
  delay(500);

  // Display info
  M5.Lcd.setBrightness(255);
  M5.Lcd.setTextFont(2);
  M5.Lcd.setTextColor(WHITE, BLACK);
  M5.Lcd.println("M5Stack Balance Mode start");

  // Init M5Bala
  m5bala.begin();
  m5bala.imu->setGyroOffsets(0.47, 0.6, -0.86);
  delay(1000);
  m5bala.move(-30);
  m5bala.rotate(0);

  // Init NeoPixel(LED)
  pixels.begin();
  pixels.setBrightness(30);
  neopixel_ctrl(0, M5STACK_FIRE_NEO_NUM_LEDS, 255, 255, 255, 30);

  setupWiFi();

  ArduinoOTA.setHostname("m5bala-001");
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });
  ArduinoOTA.begin();

  MDNS.begin("m5bala-001");

  // for ROS
  nh.initNode();
  nh.setSpinTimeout(1000);
  nh.subscribe(sub_twist);
  nh.subscribe(sub_img);
  nh.subscribe(sub_led_right);
  nh.subscribe(sub_led_left);
  nh.advertise(pub_odom);
  nh.advertise(pub_imu);

  xTaskCreatePinnedToCore(pub_calc_th, "pub_calc_th", 4096, NULL, 12, NULL, 0);
  xTaskCreatePinnedToCore(pub_th, "pub_th", 4096, NULL, 13, NULL, 0);
  xTaskCreatePinnedToCore(sub_th, "sub_th", 61440, NULL, 1, NULL, 1);
}

SimpleRate* main_loop_rate = new SimpleRate(60);
void loop() {
  // sleep
  main_loop_rate->sleep();

  // Robot
  robot_run();

  // M5Bala run
  m5bala.run();

  // M5 Loop
  M5.update();

  // For OTA
  ArduinoOTA.handle();
}


/////////////////////////////////////////////
//
/////////////////////////////////////////////

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


void setupWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.setHostname("m5bala-001");
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if (i == 21) {
    while (1) delay(500);
  }
  M5.Lcd.println(WiFi.localIP());
}

void cmd_vel_cb(const geometry_msgs::Twist& twist) {
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
}
void image_data_cb(const sensor_msgs::CompressedImage& im) {
  //
  M5.Lcd.drawJpg(im.data, sizeof(im.data), 80, 60, 0, 0, 0, 0, JPEG_DIV_NONE);
}
void led_left_cb(const std_msgs::ColorRGBA& rgba) {
  //
  neopixel_ctrl(5, 10, rgba.r, rgba.g, rgba.b, rgba.a);
}
void led_right_cb(const std_msgs::ColorRGBA& rgba) {
  //
  neopixel_ctrl(0, 5, rgba.r, rgba.g, rgba.b, rgba.a);
}


void pub_calc_th(void* arg) {
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

  SimpleRate* pub_calc_th_rate = new SimpleRate(60);
  while (1) {
    // rateing
    pub_calc_th_rate->sleep();
    // TIME
    current_time = nh.now();
    float dt = current_time.toSec() - last_time.toSec();
    /////////////////////////////////
    // ODOM
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
    //////////////////////////

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


    // Record
    last_enc0_l = current_enc0_l;
    last_enc1_r = current_enc1_r;
    last_time = current_time;
  }
}

void pub_th(void* arg) {
  SimpleRate* pub_th_rate = new SimpleRate(30);
  while (1) {
    pub_odom.publish(&odom_msg);
    pub_imu.publish(&imu_msg);
    pub_th_rate->sleep();
  }
}

void sub_th(void* arg) {
  SimpleRate* sub_th_rate = new SimpleRate(30);
  while (1) {
    nh.spinOnce();
    sub_th_rate->sleep();
  }
}

void robot_run() {
  // encoder update
  if (m5bala.getOut0() != 0 && abs(m5bala.getOut0()) < 200) {
    enc0 += (m5bala.getSpeed0()/NUM_OF_PULSES_PER_WHEEL_REVOLUTION);
  }
  if (m5bala.getOut1() != 0 && abs(m5bala.getOut1()) < 200) {
    enc1 += (m5bala.getSpeed1()/NUM_OF_PULSES_PER_WHEEL_REVOLUTION);
  }
  // Convert units
  accX = -1.0 * m5bala.imu->getAccY() * 9.81;
  accY = 1.0 * m5bala.imu->getAccX() * 9.81;
  accZ = -1.0 * m5bala.imu->getAccZ() * 9.81;
  gyroX = -1.0 * m5bala.imu->getGyroY() * (3.1415926535897931/180.0);
  gyroY = -1.0 * m5bala.imu->getGyroX() * (3.1415926535897931/180.0);
  gyroZ = -1.0 * m5bala.imu->getGyroZ() * (3.1415926535897931/180.0);
  angleX = 1.0 * m5bala.imu->getAngleY();
  angleY = -1.0 * m5bala.imu->getAngleX();
  angleZ = 1.0 * m5bala.imu->getAngleZ();
  // cmd
  m5bala.move(joystick_Y);
  m5bala.rotate(joystick_X);
}

void neopixel_ctrl(int first_cnt, int last_cnt, int r, int g, int b, int w) {
  SimpleRate neopixel_ctrl_rate(15);
  for (int i=first_cnt; i<last_cnt; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b, w));
    pixels.show();
    neopixel_ctrl_rate.sleep();
  }
}

