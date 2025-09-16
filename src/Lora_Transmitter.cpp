#define RX PD_6
#define TX PD_7   
#define M1 PA_5
#define M0 PA_6
bool bot_state = true;
bool lastButtonState = LOW;  
bool currentButtonState= LOW;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600);          
  Serial.println("TIVA Initialized");
  pinMode(PF_2,INPUT);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
}

void loop() {
  currentButtonState = digitalRead(PF_2);

  if (lastButtonState == LOW && currentButtonState == HIGH) {
    if (bot_state) {
      Serial2.print('1');
      Serial.println("Bot Stop data sent");
      bot_state = false;
      
    } else {
      Serial2.print('1');
      Serial.println("Bot On data sent");
      bot_state = true;
    }
    
  }
  delay(100);
  lastButtonState = currentButtonState; 
  
}