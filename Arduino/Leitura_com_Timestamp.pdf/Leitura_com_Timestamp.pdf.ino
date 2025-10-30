// .ino com timestamp (pacote = 1 + 2 + 2 + 4 = 9 bytes)
const int eixoY = A0;
const int eixoX = A1;
const byte startMarker = 0xFF;

const uint32_t SAMPLE_RATE_HZ = 1000;
const uint32_t PERIOD_US      = 1000000UL / SAMPLE_RATE_HZ;

void setup() {
  Serial.begin(115200);
  pinMode(eixoX, INPUT);
  pinMode(eixoY, INPUT);
}

void loop() {
  static uint32_t next_ts = micros();
  uint32_t now = micros();
  if ((int32_t)(now - next_ts) >= 0) {
    next_ts += PERIOD_US;

    uint16_t valorX = analogRead(eixoX);
    uint16_t valorY = analogRead(eixoY);
    uint32_t t_us   = micros(); // timestamp da amostra

    Serial.write(startMarker);
    Serial.write(valorX & 0xFF);
    Serial.write((valorX >> 8) & 0xFF);
    Serial.write(valorY & 0xFF);
    Serial.write((valorY >> 8) & 0xFF);
    // timestamp little-endian:
    Serial.write((uint8_t)(t_us & 0xFF));
    Serial.write((uint8_t)((t_us >> 8) & 0xFF));
    Serial.write((uint8_t)((t_us >> 16) & 0xFF));
    Serial.write((uint8_t)((t_us >> 24) & 0xFF));
  }
}
