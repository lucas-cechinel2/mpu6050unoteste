#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
//as medidas máximas e mínimas são obtidas de forma empírica!
float ax_max = 10.13;
float ax_min = -9.37;
float ay_max = 9.6;
float ay_min = -9.89;
float az_max = 10.10;
float az_min = -9.78;

float ex_offset = (ax_max+ax_min)/2;
float ey_offset = (ay_max+ay_min)/2;
float ez_offset = (az_max+az_min)/2;

float ex_escala = 19.62/(ax_max-ax_min);
float ey_escala = 19.62/(ay_max-ay_min);
float ez_escala = 19.62/(az_max-az_min);


Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(9600);

  while (!Serial) {
    delay(10);
  }

  Serial.println("Iniciando MPU6050...");

  if (!mpu.begin()) {
    Serial.println("MPU6050 nao encontrado!");
    while (1) {
      delay(10);
    }
  }

  Serial.println("MPU6050 encontrado!");

  // Configurações opcionais
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  delay(100);
}

void loop() {

  sensors_event_t a, g, temp;

  mpu.getEvent(&a, &g, &temp);

  
  float ax_ok = ex_escala * (a.acceleration.x - ex_offset);
  float ay_ok = ey_escala * (a.acceleration.y - ey_offset);
  float az_ok = ez_escala * (a.acceleration.z - ez_offset);


  Serial.print(" Ax bruto: ");
  Serial.print(a.acceleration.x);
  Serial.print(" m/s²");
  Serial.print(" | Ax corrigido: ");
  Serial.print(ax_ok);
  Serial.print(" m/s²");
  Serial.println();

  Serial.print(" Ay bruto: ");
  Serial.print(a.acceleration.y);
  Serial.print(" m/s²");
  Serial.print(" | Ay corrigido");
  Serial.print(ay_ok);
  Serial.print(" m/s²");
  Serial.println();

  Serial.print(" Az bruto: ");
  Serial.print(a.acceleration.z);
  Serial.print(" m/s²");
  Serial.print(" | Az corrigido");
  Serial.print(az_ok);
  Serial.print(" m/s²");
  Serial.println(); 
  
  delay(7000);
}