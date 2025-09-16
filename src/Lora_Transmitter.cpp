#define RX PD_6
#define TX PD_7
#define M1 PA_5
#define M0 PA_6
#define pushButton PF_2


void setup() {
  Serial.begin(9600);        // USB debug
  Serial2.begin(9600);       // PD6/PD7 -> LoRa
  Serial.println("TIVA Initialized");

  pinMode(pushButton, INPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void loop() {
  bool button_state = digitalRead(pushButton);

  // detect rising edge: button goes from LOW -> HIGH
  if (button_state == HIGH) {
    Serial2.write('1');
    Serial.println("Stop Command sent");
    delay(5000);
  }

   
}
