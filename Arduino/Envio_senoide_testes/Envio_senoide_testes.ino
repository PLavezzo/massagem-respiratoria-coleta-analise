#include <Arduino.h>
#include <math.h>

// Constantes para geração da onda
const float PI2       = 6.28318530718;  // 2·π
const float AMPLITUDE = 511.0;          // meia amplitude (0–1023 → ±511)
const float OFFSET    = 512.0;          // deslocamento para manter valores positivos
const float DELTA_ANGLE = 0.02;         // passo de ângulo a cada loop (ajuste frequência)

const byte startMarker = 0xFF; // 255

float angle = 0.0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  // Gera senoide (valorX) e cosseno (valorY)
  int valorX = (int)(sin(angle) * AMPLITUDE + OFFSET);
  int valorY = (int)(cos(angle) * AMPLITUDE + OFFSET);

  // Incrementa ângulo, mantendo em [0, PI2)
  angle += DELTA_ANGLE;
  if (angle >= PI2) {
    angle -= PI2;
  }

  // Envio binário idêntico ao seu original:
  Serial.write(startMarker);          // 1. byte de início

  Serial.write(valorX & 0xFF);        // LSB de valorX
  Serial.write((valorX >> 8) & 0xFF); // MSB de valorX

  Serial.write(valorY & 0xFF);        // LSB de valorY
  Serial.write((valorY >> 8) & 0xFF); // MSB de valorY

  delay(10); // ~100 Hz (ajuste conforme quiser)
}
