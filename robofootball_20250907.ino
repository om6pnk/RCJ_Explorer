#include "SmilersoftVectors.h"
#include "motor_control.h"
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "IR_sensor.h"
#include "GyverOLED.h"
#include <string.h>
#include "GyverButton.h"

#define S0 22
#define S1 23
#define S2 24
#define S3 25
#define OUT A15

enum class State {
  Center,
  Front,
  Back,
  Left,
  Right
};

enum class Screen {
  MainScreen,
  MainMenu,
  DebugInfo,
  Movement,
  SecondMenu,
  Camera
};

Vector2 gyro(0, 0);
MPU6050 mpu;
uint8_t fifoBuffer[45];
int b_a = 0; // угол до мяча [-180; 180]
int oldb_a = 0;
int _sensor;
int sensors[16];
uint64_t __millis = 0;
const int filter_number = 10;
int s[filter_number];
GyverOLED<SSD1306_128x32, OLED_NO_BUFFER> oled;
bool j_button;
State j_pos;
int c_a = 0;
Screen screen;
int activeScreenPos = -1;
bool isButtonBlocked = false;
int clickTime;
GButton b(37);
int clickedButton = 1;
bool isYellow;
int yc_a = 0;
int bc_a = 0;

String mainScreenInfo[3];
String mainMenuButtons[4];
String debugInfo[4];
String movement[4];
String secondMenu[3];
String camera[3];

void setup() {
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(48, OUTPUT);
  pinMode(OUT, INPUT);
  pinMode(A13, INPUT);
  pinMode(A14, INPUT);
  pinMode(37, INPUT_PULLUP);
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();

  mpu.initialize();
  mpu.dmpInitialize();
  delay(15);
  mpu.setDMPEnabled(true);
  mpu.setXAccelOffset(-3209);
  mpu.setYAccelOffset(-1014);
  mpu.setZAccelOffset(-5);
  mpu.setXGyroOffset(173);
  mpu.setYGyroOffset(-63);
  mpu.setZGyroOffset(39);

  oled.init();
  oled.clear();

  mainScreenInfo[0] = "explorer";
  mainScreenInfo[1] = "Uchis i tochka, Minsk,";
  mainScreenInfo[2] = "Krasnaya str 7";

  mainMenuButtons[0] = "<- Back";
  mainMenuButtons[1] = "Debug info";
  mainMenuButtons[2] = "Movement";
  mainMenuButtons[3] = "Second menu";

  debugInfo[0] = "<- Back";
  debugInfo[1] = "Camera angle: " + String(c_a);
  debugInfo[2] = "Gyro: " + String(gyro.x);
  debugInfo[3] = "Ball angle: " + String(b_a);

  movement[0] = "<- Back";
  movement[1] = "Play";
  movement[2] = "Stop";
  movement[3] = "Drive to camera";

  secondMenu[0] = "<- Back";
  secondMenu[1] = "Camera";
  secondMenu[2] = "Kicker";

  camera[0] = "<- Back";
  camera[1] = "Yellow";
  camera[2] = "Blue";

  MainScreen();
}

void ReadGyro() { //чтение
  static uint64_t tmr;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer) && millis() - tmr >= 11) {
    // переменные для расчёта (ypr можно вынести в глобал)
    Quaternion q;
    VectorFloat gravity;
    float ypr[3];
    // расчёты
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    gyro.x = degrees(ypr[0]);
    gyro.y = degrees(ypr[2]);
    tmr = millis();  // сброс таймера
  }
}

void PrintGyro() { // гироскопа
  Serial.print("navangle:");
  Serial.print(gyro.x);
  Serial.print(" tildangle:");
  Serial.println(gyro.y);
}

int ReadSensor(int sensor) {
  switch(sensor) {
    case(0):
    digitalWrite(S0, 0);
    digitalWrite(S1, 0);
    digitalWrite(S2, 0);
    digitalWrite(S3, 0);
    break;

    case(1):
    digitalWrite(S0, 0);
    digitalWrite(S1, 0);
    digitalWrite(S2, 0);
    digitalWrite(S3, 1);
    break;

    case(2):
    digitalWrite(S0, 0);
    digitalWrite(S1, 0);
    digitalWrite(S2, 1);
    digitalWrite(S3, 0);
    break;

    case(3):
    digitalWrite(S0, 0);
    digitalWrite(S1, 0);
    digitalWrite(S2, 1);
    digitalWrite(S3, 1);
    break;

    case(4):
    digitalWrite(S0, 0);
    digitalWrite(S1, 1);
    digitalWrite(S2, 0);
    digitalWrite(S3, 0);
    break;

    case(5):
    digitalWrite(S0, 0);
    digitalWrite(S1, 1);
    digitalWrite(S2, 0);
    digitalWrite(S3, 1);
    break;

    case(6):
    digitalWrite(S0, 0);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(S3, 0);
    break;

    case(7):
    digitalWrite(S0, 0);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(S3, 1);
    break;

    case(8):
    digitalWrite(S0, 1);
    digitalWrite(S1, 0);
    digitalWrite(S2, 0);
    digitalWrite(S3, 0);
    break;

    case(9):
    digitalWrite(S0, 1);
    digitalWrite(S1, 0);
    digitalWrite(S2, 0);
    digitalWrite(S3, 1);
    break;

    case(10):
    digitalWrite(S0, 1);
    digitalWrite(S1, 0);
    digitalWrite(S2, 1);
    digitalWrite(S3, 0);
    break;

    case(11):
    digitalWrite(S0, 1);
    digitalWrite(S1, 0);
    digitalWrite(S2, 1);
    digitalWrite(S3, 1);
    break;

    case(12):
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 0);
    digitalWrite(S3, 0);
    break;

    case(13):
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 0);
    digitalWrite(S3, 1);
    break;

    case(14):
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(S3, 0);
    break;

    case(15):
    digitalWrite(S0, 1);
    digitalWrite(S1, 1);
    digitalWrite(S2, 1);
    digitalWrite(S3, 1);
    break;
  }
  _sensor = analogRead(OUT);

  return _sensor;
}

void UpdateSensorsArray() {
  for(int i = 0; i < 16; i++) {
    sensors[i] = ReadSensor(i);
  }
}

void UpdateSensorsHistory() {
  for(int i = filter_number - 1; i > 0; i--) {
    s[i] = s[i-1];
  }
  s[0] = b_a;
}

void BackToBall() {
  aDriveRotation(abs(b_a) + 80, 255, gyro.x);
}

void timer(uint64_t millis) {
  uint64_t _millis = 0;
  while(_millis < millis) {
    delay(1);
    _millis++;
  }
}

void Circle() {
  for(int i = 0; i < 360; i++) {
    aDrive(i, 100, 0);
    timer(200);
  }
}

// void DrivePower(int speed) //функция вращения мотора
// {   
//   // if(speed > 127) {
//   //   speed = 127;
//   // }
//   // if(speed < -127) {
//   //   speed = -127;
//   // }

//   if(speed < 0){
//     digitalWrite(INA1, 1);
//     digitalWrite(INA2, 0);
//   }
//   if(speed > 0){
//     digitalWrite(INA1, 0);
//     digitalWrite(INA2, 1);
//   } 

//   if(speed == 0){
//     digitalWrite(STBY, 0);
//     analogWrite(PWMA, 0);
//   }
//   else
//   {
//     digitalWrite(STBY, 1);
//     analogWrite(PWMA, abs(speed));
//   }  
// }

// void ReadGyro() { //чтение
//   static uint32_t tmr;
//   if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
//     if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
//       // переменные для расчёта (ypr можно вынести в глобал)
//       Quaternion q;
//       VectorFloat gravity;
//       float ypr[3];
//       // расчёты
//       mpu.dmpGetQuaternion(&q, fifoBuffer);
//       mpu.dmpGetGravity(&gravity, &q);
//       mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//       gyro.x = degrees(ypr[0]);
//       gyro.y = degrees(ypr[2]);
//     }
//   }
//   tmr = millis();  // сброс таймера
// }

void ball(int address) {//0x09
  Wire.requestFrom(address, sizeof(int));
  byte buffer[sizeof(int)]; 
  for (int i = 0; i < sizeof(int); i++) {
    buffer[i] = Wire.read();
  }
  memcpy(&b_a, buffer, sizeof(int)); 
  // Serial.println(b_a);
  // b_a = b_a - b_a%5;
  // if (abs(oldb_a - b_a) > 100){
  //   b_a = oldb_a;
  // }
  // oldb_a = b_a;
}

void ReadJoystick() {
  if(b.isSingle() && isButtonBlocked == false) {
    j_button = true;
    clickTime = millis();
  }
  else j_button = false;
  if(analogRead(A13) >= 25 && analogRead(A13) <= 675 && analogRead(A14) >= 25 && analogRead(A14) <= 675) j_pos = State::Center;
  if(analogRead(A13) < 25) {
    j_pos = State::Front;
    activeScreenPos--;
  }
  if(analogRead(A13) > 675) {
    j_pos = State::Back;
    activeScreenPos++;
  }
  if(analogRead(A14) > 500) j_pos = State::Left;
  if(analogRead(A14) < 200) j_pos = State::Right;
}

void Kicker() {
  digitalWrite(48, 1);
  delay(300);
  digitalWrite(48, 0);
}

void BlockButton() {
  while(millis() < clickTime + 500) isButtonBlocked = true;
  isButtonBlocked = false;
}

void MainScreen() {
  screen = Screen::MainScreen;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 3; i++) {
    oled.setCursor(0, i);
    oled.print(mainScreenInfo[i]);
  }
}

void MainMenu() {
  screen = Screen::MainMenu;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 4; i++) {
    oled.setCursor(0, i);
    oled.print(mainMenuButtons[i]);
  }
}

void DebugInfo() {
  screen = Screen::DebugInfo;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 4; i++) {
    oled.setCursor(0, i);
    oled.print(debugInfo[i]);
  }
}

void Movement() {
  screen = Screen::Movement;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 4; i++) {
    oled.setCursor(0, i);
    oled.print(movement[i]);
  }
}

void SecondMenu() {
  screen = Screen::SecondMenu;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 3; i++) {
    oled.setCursor(0, i);
    oled.print(secondMenu[i]);
  }
}

void Camera() {
  screen = Screen::Camera;
  activeScreenPos = -1;
  oled.clear();
  oled.home();
  for(int i = 0; i < 3; i++) {
    oled.setCursor(0, i);
    oled.print(camera[i]);
  }
}

void GetCamera() {
  int32_t temp = 0;
  char buff[128] = {0};

  Wire.requestFrom(0x12, 2);
  if(Wire.available() == 2) { // got length?

    temp = Wire.read() | (Wire.read() << 8);
    delay(1); // Give some setup time...

    Wire.requestFrom(0x12, temp);
    if(Wire.available() == temp) { // got full message?

      temp = 0;
      while(Wire.available()) buff[temp++] = Wire.read();

    } else {
      while(Wire.available()) Wire.read(); // Toss garbage bytes.
    }
  } else {
    while(Wire.available()) Wire.read(); // Toss garbage bytes.
  }

  yc_a = atoi(buff) / 100;
  bc_a = atoi(buff) % 100;

  if(isYellow) c_a = yc_a;
  else c_a = bc_a;

  Serial.print("cam: ");
  Serial.print(buff);
  Serial.print(" yellow: ");
  Serial.print(yc_a);
  Serial.print(" blue: ");
  Serial.println(bc_a);

  delay(1); // Don't loop to quickly.
}

void SwitchCamera(bool _isYellow) {
  isYellow = _isYellow;
}

void loop() {
  b.tick();
  ReadGyro();
  ReadJoystick();
  ball(0x09);
  if(b_a == -167 || b_a == 170 || b_a == 63) b_a = 0;
  // UpdateSensorsHistory();
  // for(int i = 0; i < filter_number; i++) {
  //   Serial.print(s[i]);
  //   Serial.print(' ');
  // }
  // Serial.println(' ');
  // Serial.print(analogRead(A13));
  // Serial.print(' ');
  // Serial.print(analogRead(A14));
  // Serial.print(' ');
  // Serial.println(digitalRead(37));
  // Serial.print(j_button);
  // Serial.print(' ');
  // Serial.println(int(j_pos));
  // if(b_a == -118 || b_a == -167) {
  //     b_a = 20;
  //   }
  // else aDrive(b_a, 255, gyro.x);

  GetCamera();
  if(clickedButton == 0) {
    if(b_a < -80 || b_a > 80) BackToBall();
    else if(c_a > -20 && c_a < 20) aDriveRotation(-b_a, 255,gyro.x);
    else aDriveRotation(-b_a, 255, gyro.x);
  }
  if(clickedButton == 1) aDrive(0, 0, 0);
  if(clickedButton == 2) aDriveRotation(gyro.x, 255 ,gyro.x);
  //Serial.println(c_a);

  if(screen == Screen::MainScreen && j_button == true) MainMenu();

  if(screen == Screen::MainMenu) {
    if(activeScreenPos < -1) activeScreenPos = -1;
    if(activeScreenPos > 3) activeScreenPos = 3;
    for(int j = -1; j < 4; j++) {
      if(j > -1) oled.setCursor(0, j);
      if(j == activeScreenPos && j > -1) oled.print('[' + mainMenuButtons[activeScreenPos] + ']');
      else if(j > -1) oled.print(mainMenuButtons[j] + "  ");
    }

    if(j_button == true && activeScreenPos == 0) MainScreen();
    if(j_button == true && activeScreenPos == 1) DebugInfo();
    if(j_button == true && activeScreenPos == 2) Movement();
    if(j_button == true && activeScreenPos == 3) SecondMenu();
  }

  if(screen == Screen::DebugInfo) {
    if(activeScreenPos < -1) activeScreenPos = -1;
    if(activeScreenPos > 3) activeScreenPos = 3;
    debugInfo[1] = "Camera angle: " + String(c_a);
    debugInfo[2] = "Gyro: " + String(gyro.x);
    debugInfo[3] = "Ball angle: " + String(b_a);
    for(int j = -1; j < 4; j++) {
      if(j > -1) oled.setCursor(0, j);
      if(j == activeScreenPos && j > -1) oled.print('[' + debugInfo[activeScreenPos] + ']');
      else if(j > -1) oled.print(debugInfo[j] + "  ");
    }

    if(j_button == true && activeScreenPos == 0) MainMenu();
  }

  if(screen == Screen::Movement) {
    if(activeScreenPos < -1) activeScreenPos = -1;
    if(activeScreenPos > 3) activeScreenPos = 3;
    for(int j = -1; j < 4; j++) {
      if(j > -1) oled.setCursor(0, j);
      if(j == activeScreenPos && j > -1) oled.print('[' + movement[activeScreenPos] + ']');
      else if(j > -1) oled.print(movement[j] + "  ");
    }
    
    if(j_button == true && activeScreenPos == 0) MainMenu();
    if(j_button == true && activeScreenPos == 1) clickedButton = 0;
    if(j_button == true && activeScreenPos == 2) clickedButton = 1;
    if(j_button == true && activeScreenPos == 3) clickedButton = 2;
  }

  if(screen == Screen::SecondMenu) {
    if(activeScreenPos < -1) activeScreenPos = -1;
    if(activeScreenPos > 2) activeScreenPos = 2;
    for(int j = -1; j < 3; j++) {
      if(j > -1) oled.setCursor(0, j);
      if(j == activeScreenPos && j > -1) oled.print('[' + secondMenu[activeScreenPos] + ']');
      else if(j > -1) oled.print(secondMenu[j] + "  ");
    }
    
    if(j_button == true && activeScreenPos == 0) MainMenu();
    if(j_button == true && activeScreenPos == 1) Camera();
    if(j_button == true && activeScreenPos == 2) Kicker();
  }

  if(screen == Screen::Camera) {
    if(activeScreenPos < -1) activeScreenPos = -1;
    if(activeScreenPos > 2) activeScreenPos = 2;
    for(int j = -1; j < 3; j++) {
      if(j > -1) oled.setCursor(0, j);
      if(j == activeScreenPos && j > -1) oled.print('[' + camera[activeScreenPos] + ']');
      else if(j > -1) oled.print(camera[j] + "  ");
    }
    
    if(j_button == true && activeScreenPos == 0) SecondMenu();
    if(j_button == true && activeScreenPos == 1) SwitchCamera(true);
    if(j_button == true && activeScreenPos == 2) SwitchCamera(false);
  }
}