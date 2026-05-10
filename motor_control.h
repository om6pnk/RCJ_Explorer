#pragma once

#define M1_1 8
#define M1_2 9

#define M3_1 3
#define M3_2 2

#define M4_1 5
#define M4_2 4

const float sqrt3 = sqrt(3);

void drive(int v1, int v2, int v3)
{
    v1 = min(255, max(-255, v1));
    v2 = min(255, max(-255, v2));
    v3 = min(255, max(-255, v3));
    analogWrite(M1_1, v1 >= 0 ? 255 : 0);
    analogWrite(M1_2, v1 >= 0 ? 255 - v1 : -v1);

    analogWrite(M4_1, v2 >= 0 ? 255 : 0);
    analogWrite(M4_2, v2 >= 0 ? 255 - v2 : -v2);

    analogWrite(M3_1, v3 >= 0 ? 255 : 0);
    analogWrite(M3_2, v3 >= 0 ? 255 - v3 : -v3);
}

void motor_setup()
{
    pinMode(M1_1, OUTPUT);
    pinMode(M1_2, OUTPUT);
    pinMode(M3_1, OUTPUT);
    pinMode(M3_2, OUTPUT);
    pinMode(M4_1, OUTPUT);
    pinMode(M4_2, OUTPUT);

    drive(0, 0, 0);
}

void aDrive(int angel, int speed, int rotation_speed)
{
    // float rad_angel = (angel - 120) * PI / 180;
    // float M = 0;//rotation_speed * 10;
    // float Vx = sin(rad_angel) * speed;
    // float Vy = cos(rad_angel) * speed;

    // int v1 = (-M + Vx + sqrt3 * Vy) / 3;
    // int v2 = (M - Vx + sqrt3 * Vy) / 3;
    // int v3 = (M + 2 * Vx) / 3;

    // Serial.print(v1);
    // Serial.print("\t");
    // Serial.print(v2);
    // Serial.print("\t");
    // Serial.print(v3);
    // Serial.print("\n");

    // drive(v1, v2, v3);

  float rad_angel = (-angel) * PI / 180;
  Serial.println(rad_angel);
  float Vx = sin(rad_angel);
  float Vy = cos(rad_angel);
  double max_abs;

  double v3 = -(Vx + sqrt3 * Vy) / 3;
  double v1 = -(-Vx + sqrt3 * Vy) / 3;
  double v2 = (2 * Vx) / 3;
  max_abs = max(abs(v1), max(abs(v2), abs(v3)));

  v1 /= max_abs;
  v2 /= max_abs;
  v3 /= max_abs;

  v1 *= (speed - rotation_speed);
  v2 *= (speed - rotation_speed);
  v3 *= (speed - rotation_speed);

  v1 -= rotation_speed;
  v2 += rotation_speed;
  v3 += rotation_speed;

  Serial.print(v1);
  Serial.print("\t");
  Serial.print(v2);
  Serial.print("\t");
  Serial.print(v3);
  Serial.print("\n");

  drive(v1, v2, v3);
}

void aDriveNormalization(int angel, int speed, int rotation_speed) {
  float rad_angel = (angel - 120) * PI / 180;
  float Vx = sin(rad_angel);
  float Vy = cos(rad_angel);
  double max_abs;

  float v1 = (Vx + sqrt3 * Vy) / 3;
  float v2 = (Vx + sqrt3 * Vy) / 3;
  float v3 = (2 * Vx) / 3;
  max_abs = max(abs(v1), max(abs(v2), abs(v3)));

  v1 /= max_abs;
  v2 /= max_abs;
  v3 /= max_abs;

  v1 *= speed;
  v2 *= speed;
  v3 *= speed;

  v1 += rotation_speed;
  v2 += rotation_speed;
  v3 += rotation_speed;

  max_abs = max(abs(v1), max(abs(v2), abs(v3)));

  v1 /= max_abs;
  v2 /= max_abs;
  v3 /= max_abs;

  v1 *= speed;
  v2 *= speed;
  v3 *= speed;

  Serial.print(v1);
  Serial.print("\t");
  Serial.print(v2);
  Serial.print("\t");
  Serial.print(v3);
  Serial.print("\n");

  drive(v1, v2, v3);
}

void aDriveRotation(int angel, int speed, int rotation_speed) {
  float rad_angel = (-angel) * PI / 180;
  Serial.println(rad_angel);
  float Vx = sin(rad_angel);
  float Vy = cos(rad_angel);
  double max_abs;

  double v3 = -(Vx + sqrt3 * Vy) / 3;
  double v1 = -(-Vx + sqrt3 * Vy) / 3;
  double v2 = (2 * Vx) / 3;
  max_abs = max(abs(v1), max(abs(v2), abs(v3)));

  v1 /= max_abs;
  v2 /= max_abs;
  v3 /= max_abs;

  v1 *= (speed - rotation_speed);
  v2 *= (speed - rotation_speed);
  v3 *= (speed - rotation_speed);

  v1 -= rotation_speed;
  v2 += rotation_speed;
  v3 += rotation_speed;

  Serial.print(v1);
  Serial.print("\t");
  Serial.print(v2);
  Serial.print("\t");
  Serial.print(v3);
  Serial.print("\n");

  drive(v1, v2, v3);
}