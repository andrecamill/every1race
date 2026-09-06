#define THROTTLE_PIN 2
#define BRAKE_PIN 4
// Uncomment macro below to enable third clutch pedal support
// #define THROTTLE_PIN 6

void setup() {
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);

#ifdef THROTTLE_PIN
    pinMode(THROTTLE_PIN, INPUT);
#endif

  Serial.begin(9600);
}

void loop() {
  int throttle_state = !digitalRead(THROTTLE_PIN);
  int brake_state = !digitalRead(BRAKE_PIN);

#ifdef THROTTLE_PIN
  int clutch_state = !digitalRead(THROTTLE_PIN);
  Serial.print("PEDAL_CLUTCH:");
  Serial.println(clutch_state);
#endif

  Serial.print("PEDAL_THROTTLE:");
  Serial.println(throttle_state);

  Serial.print("PEDAL_BRAKE:");
  Serial.println(brake_state);

  delay(50);
}