// Pinos joystick
const int eixoY = A0;
const int eixoX = A1;

const byte startMarker = 0xFF; // 255

void setup() {
  Serial.begin(115200); 
  pinMode(eixoX, INPUT);
  pinMode(eixoY, INPUT);
}

void loop() {
  // Lê os analógicos
  int valorX = analogRead(eixoX);
  int valorY = analogRead(eixoY);

  // Envia os dados em formato binário:
  Serial.write(startMarker);      // 1. Byte de início

  // Envia os 2 bytes do valorX (Little Endian - comum em AVR/ESP)
  Serial.write(valorX & 0xFF);        // Byte menos significativo (LSB)
  Serial.write((valorX >> 8) & 0xFF); // Byte mais significativo (MSB)

  // Envia os 2 bytes do valorY
  Serial.write(valorY & 0xFF);        // LSB
  Serial.write((valorY >> 8) & 0xFF); // MSB


  // Serial.println(valorX);


}