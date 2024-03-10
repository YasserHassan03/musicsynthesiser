#include <Arduino.h>
#include <U8g2lib.h>
#include <STM32FreeRTOS.h> 
#include <Context.hpp>
#include <cstdint>

//Constants
  const uint32_t interval = 100; //Display update interval

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




// DEFINES 
#define KEY_MASK 0xFFF

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

const uint32_t steps[12] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346,91007233, 96418697}; 
const uint16_t keys[12] = {0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800};
const char * notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No Note"};
Context context;

void sampleISR();
void scanKeysTask(void * pvParameters);
uint32_t getStepSize(uint16_t key);
void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t readCols ();
uint32_t readMatrix();
void setRow (const uint8_t row);
const char * getNote(const uint16_t keys);
volatile uint32_t step = 0;


void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
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

  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  TaskHandle_t scanKeysHandle = NULL;

  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &scanKeysHandle 
  ); 


  vTaskStartScheduler(); 
}


void scanKeysTask(void * pvParameters) {

  uint32_t stepValue = 0;
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t currentState = 0;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    // xSemaphoreTake(context.mutex, portMAX_DELAY); 
    context.lock();
    context.setState(readMatrix());
    currentState = context.getState();
    context.unlock();
    stepValue = getStepSize(currentState & KEY_MASK);
    __atomic_store_n(&step, stepValue, __ATOMIC_RELAXED);
    // xSemaphoreGive(context.mutex);
  }

}


void sampleISR()
{
  uint32_t currentStep = 0;
  __atomic_load(&step, &currentStep, __ATOMIC_RELAXED);
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStep;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}


void loop() {
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next);  //Wait for next interval

  next += interval;

  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.setCursor(10, 20);
  uint32_t copyState;
  context.lock(); 
  copyState = context.getState();
  context.unlock();
  const char * note = getNote((uint16_t) copyState & KEY_MASK);
  u8g2.drawStr(2, 20, note);
  u8g2.sendBuffer();          // transfer internal memory to the display

    //Toggle LED
  digitalToggle(LED_BUILTIN);
  
}



uint32_t getStepSize(uint16_t key) {
  // Serial.printf("key: %d, step:%d\n", key, step);

  for (uint8_t i = 0; i < 12; i++) {
    if ((key & keys[i]) == 0) {
      return steps[i];
    }
  }
  return 0;
}


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



uint8_t readCols () {
  return  ((digitalRead(C3_PIN) & 0x1) << 3) + ((digitalRead(C2_PIN) & 0x1) << 2) + ((digitalRead(C1_PIN) & 0x1) << 1) + (digitalRead(C0_PIN) & 0x1);
}


void setRow (const uint8_t row) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, row & 0x01);
  digitalWrite(RA1_PIN, row & 0x02);
  digitalWrite(RA2_PIN, row & 0x04);
  digitalWrite(REN_PIN,HIGH);
}


uint32_t readMatrix() {
  uint32_t result = 0;

  for (uint8_t row = 0; row < 9; row++) {
    setRow(row);
    delayMicroseconds(2);
    result |= (readCols() << (row * 4));
  }

  return result;

}

// function that matches the step size to the note
const char * getNote(const uint16_t keys) {
  for (int i = 0; i < 12; ++i)
  {
    if ((keys & (1 << i)) == 0)
    {
      return notes[i];
    }
  }
  return notes[12];
}

