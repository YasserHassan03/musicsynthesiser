#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
//Constants
  const uint32_t interval = 100; //Display update interval
  volatile uint32_t currentStepSize;
//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}
void sampleISR(){
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);

}
std::bitset<4> readCols() {
  std::bitset<4> result;
  // digitalWrite(REN_PIN,HIGH);
  // digitalWrite(RA0_PIN,LOW);
  // digitalWrite(RA1_PIN,LOW);
  // digitalWrite(RA2_PIN,LOW);
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

const uint32_t stepSizes [] = {49977801/2, 54113269/2, 57330981/2, 60740013/2, 64351885/2, 68178311/2, 72232370/2, 76527532/2, 81078245/2, 85899346/2,91007233/2, 96418697/2};


const char* getNote(std::bitset<32> inputs){
  const char* out = "No note pressed";
  if ((inputs.to_ulong() & 0x1) == 0 ) {
    out = "C";
  } else if ((inputs.to_ulong() & 0x2) == 0){
    out = "C#/Db";
  } else if ((inputs.to_ulong() & 0x4) == 0){
    out = "D";
  } else if ((inputs.to_ulong() & 0x8) == 0){
    out = "D#/Eb";
  } else if ((inputs.to_ulong() & 0x10) == 0){
    out = "E";
  } else if ((inputs.to_ulong() & 0x20) == 0){
    out = "F";
  } else if ((inputs.to_ulong() & 0x40) == 0){
    out = "F#/Gb";
  } else if ((inputs.to_ulong() & 0x80) == 0){
    out = "G";
  } else if ((inputs.to_ulong() & 0x100) == 0){
    out = "G#/Ab";
  } else if ((inputs.to_ulong() & 0x200) == 0){
    out = "A";
  } else if ((inputs.to_ulong() & 0x400) == 0){
    out = "A#/Bb";
  } else if ((inputs.to_ulong() & 0x800) == 0){
    out = "B";
  }
  return out;


}

uint32_t getStepSize(std::bitset<32> inputs) {
  uint32_t result = 0;
  if ((inputs.to_ulong() & 0x1) == 0) {
    result = stepSizes[0];
  } else if ((inputs.to_ulong() & 0x2) ==0) {
    result = stepSizes[1];   
  } else if ((inputs.to_ulong() & 0x4) ==0) {
    result = stepSizes[2];
  } else if ((inputs.to_ulong() & 0x8) ==0) {
    result = stepSizes[3];
  } else if ((inputs.to_ulong() & 0x10) ==0) {
    result = stepSizes[4];
  } else if ((inputs.to_ulong() & 0x20) ==0) {
    result = stepSizes[5];
  } else if ((inputs.to_ulong() & 0x40) ==0) {
    result = stepSizes[6];
  } else if ((inputs.to_ulong() & 0x80) ==0) {
    result = stepSizes[7];
  } else if ((inputs.to_ulong() & 0x100) ==0) {
    result = stepSizes[8];
  } else if ((inputs.to_ulong() & 0x200) ==0) {
    result = stepSizes[9];
  } else if ((inputs.to_ulong() & 0x400) ==0) {
    result = stepSizes[10];
  } else if ((inputs.to_ulong() & 0x800) ==0) {
    result = stepSizes[11];
  }
  return result;
}

void setRow(uint8_t row) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN,row & 0x01);
  digitalWrite(RA1_PIN,row & 0x02);
  digitalWrite(RA2_PIN,row & 0x04);
  digitalWrite(REN_PIN,LOW);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,HIGH);
}

void setup() {
  // put your setup code here, to run once:
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
}



void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;
  std::bitset<32> inputs;
  while (millis() < next);  //Wait for next interval

  next += interval;

  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(2,10,"Helllo World!");  // write something to the internal memory
  u8g2.setCursor(2,20);
  for (int i = 0; i < 3; i++) {
    setRow(i);
    delayMicroseconds(3);
    std::bitset<4> cols = readCols();
    inputs |= (cols.to_ulong() << (i*4));
  }
  currentStepSize=getStepSize(inputs);
  u8g2.print(inputs.to_ulong(),HEX);
  // transfer internal memory to the display
  u8g2.drawStr(2,30,getNote(inputs));
  u8g2.sendBuffer();  
  //Toggle LED
  digitalToggle(LED_BUILTIN);
  
}