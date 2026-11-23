const uint8_t NUM_STRAIN_CHANNELS = 14;

const uint8_t strainChannels[NUM_STRAIN_CHANNELS] = {
  0, 1, 2, 3, 4, 5, 6, 14, 15, 9, 10, 11, 12, 13
};
const uint8_t forceChannel = 7;

void setup() {
  Serial.begin(115200);
  analogReference(EXTERNAL);
}

void loop() {
  Serial.print("IIP");
  for (uint8_t i = 0; i < NUM_STRAIN_CHANNELS; ++i) {
    Serial.print(',');
    Serial.print(analogRead(strainChannels[i]));
  }
  Serial.print(',');
  Serial.println(analogRead(forceChannel));
  delay(50);
}
