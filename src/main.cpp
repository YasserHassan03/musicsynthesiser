#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <string>
#include <iostream>
#include <STM32FreeRTOS.h>

//Constants
  const uint32_t interval = 100; //Display update interval
    const uint32_t stepSizes [] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346, 91007233, 96418697};
    const std::string notes [] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};
    volatile uint32_t currentStepSize;
    std::string note = "No note detected";
    struct {
      std::bitset<32> inputs;
      volatile int rotationCount;
      SemaphoreHandle_t mutex;  
    } sysState;

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

std::bitset<4> readCols(){
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN,HIGH);
}

void scanKeysTask(void *pvParameters){
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool prevA = false, prevB = false;
  std::string note;

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    uint32_t localCurrentStepSize;
    sysState.inputs.reset();
    for (int i = 0; i<=3; i++){
      setRow(i);
      delayMicroseconds(3);
      sysState.inputs |= (readCols().to_ulong() << (4 * i));
    }
    if ((sysState.inputs.to_ulong() & 0xFFF) != 0xFFF) {
      int index = static_cast<int>(std::log2((0xFFF - (sysState.inputs.to_ulong() & 0xFFF))));
      localCurrentStepSize = stepSizes[index];
      note = notes[index];
    }
    else {  
      localCurrentStepSize = 0;
      note = "No note detected";
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    bool curA = sysState.inputs[12];
    bool curB = sysState.inputs[13]; 
    if ((!prevA && !prevB && !curB && curA) || (prevA && prevB && curB && !curA)) {
      if (__atomic_load_n(&sysState.rotationCount, __ATOMIC_RELAXED) < 8){
        __atomic_store_n(&sysState.rotationCount, __atomic_load_n(&sysState.rotationCount, __ATOMIC_RELAXED) + 1, __ATOMIC_RELAXED);
    }
  }
    else if ((prevA && !prevB && !curB && !curA) || (!prevA && prevB && curB && curA)) {
      if (__atomic_load_n(&sysState.rotationCount, __ATOMIC_RELAXED) > 0){
        __atomic_store_n(&sysState.rotationCount, __atomic_load_n(&sysState.rotationCount, __ATOMIC_RELAXED) - 1, __ATOMIC_RELAXED);
    }
  }
    prevA = curA;
    prevB = curB;
    xSemaphoreGive(sysState.mutex);
  }
}

void displayUpdate(void *pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    u8g2.clearBuffer();        
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    u8g2.drawStr(2,10,"Stack Synthesizer!!");  
    u8g2.setCursor(2,20);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print(sysState.inputs.to_ulong() & 0xFFF,HEX);  
    u8g2.print(sysState.rotationCount,DEC);  
    xSemaphoreGive(sysState.mutex);
    u8g2.drawStr(2, 30, note.c_str());
    u8g2.sendBuffer();          
    digitalToggle(LED_BUILTIN);
  }
}

void sampleISR(){
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize = 0;
  __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - __atomic_load_n(&sysState.rotationCount, __ATOMIC_RELAXED));
  analogWrite(OUTR_PIN, Vout + 128);
}

void setup() {
  // put your setup code here, to run once:
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		
  "scanKeys",		
  64,      		
  NULL,			
  1,			
  &scanKeysHandle);	
  
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdate,		
  "displayUpdate",		
  64,      		
  NULL,			
  2,			
  &displayUpdateHandle);	

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
  sysState.mutex = xSemaphoreCreateMutex();
  vTaskStartScheduler();
}

void loop() {

}
