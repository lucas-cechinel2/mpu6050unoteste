#include <Arduino.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  Serial.println("Iniciando MPU6050...");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("Falha na conexao com o MPU6050!");
    while (1) { delay(10); }
  }
  Serial.println("MPU6050 conectado com sucesso!");

  // --- CALIBRAÇÃO PROFISSIONAL ---
  // Injetando os seus valores RAW diretamente na memória do MPU6050.
  // A partir desta linha, o chip corrige os próprios erros fisicamente.
  mpu.setXAccelOffset(-318);
  mpu.setYAccelOffset(-423);
  //mpu.setZAccelOffset(6664); <-- causou erro na leitura de z, overflow. 19,61m/s²
  
  mpu.setXGyroOffset(106);
  mpu.setYGyroOffset(55);
  mpu.setZGyroOffset(-20);
  // -------------------------------
}

void loop() {
  // Variáveis para receber os valores RAW (brutos)
  int16_t ax, ay, az;
  int16_t gx, gy, gz;

  // A biblioteca busca os dados já corrigidos internamente pelo sensor
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // CONVERSÃO PARA FÍSICA REAL (m/s²)
  // A biblioteca usa a escala padrão de 2G. Nessa escala, 1G = 16384.
  // Dividimos o valor RAW por 16384 e multiplicamos pela gravidade (9.80665).
  float ax_ms2 = (ax / 16384.0) * 9.80665;
  float ay_ms2 = (ay / 16384.0) * 9.80665;
  float az_ms2 = (az / 16384.0) * 9.80665;

  // Impressão limpa no monitor serial
  Serial.print("Ax: "); Serial.print(ax_ms2); Serial.print(" m/s² | ");
  Serial.print("Ay: "); Serial.print(ay_ms2); Serial.print(" m/s² | ");
  Serial.print("Az: "); Serial.print(az_ms2); Serial.println(" m/s²");

  delay(500);
}