#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>
#include <Context.hpp>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <sys/types.h>
#include <ES_CAN.h>
#include "reent.h"
#include "stm32l432xx.h"
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_dac.h"
#include "stm32l4xx_hal_def.h"
#include "stm32l4xx_hal_dma.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_rcc.h"
#include "stm32l4xx_hal_tim.h"
#include "wiring_time.h"
#include <iostream>
#include <numeric>


//Constants
const uint32_t interval = 100; //Display update interval
uint32_t ID = 0x123;

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
#define DAC_WRITE_FREQ 22000
#define KEY_PRESSED 0x50
#define KEY_RELEASED 0x52
#define TOTAL_KEYS 12u
#define MESSAGE_SIZE 8
#define L 64
#define VOICES 12

// Initiaion Intervals
const TickType_t xStateUpdateFreq = 50/portTICK_PERIOD_MS; // 50ms Initiation Interval


//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);


// <$> Constant Arrays <$>
const uint32_t steps[12] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346,91007233, 96418697}; 
const uint16_t keyMasks[12] = {0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800};
const char * notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No Note"};


// TODO: Change Context to singleton as per instruction
// Initialise Global Variables
Context context = Context();

// TODO: Encapsulate txMessages, and rxMessages in a way that does not use alot of memory, and without dynamic memory allocation
uint8_t txMessages[MESSAGE_SIZE] = {0}; 
QueueHandle_t msgInQ = xQueueCreate(36, MESSAGE_SIZE);
QueueHandle_t msgOutQ = xQueueCreate(36, MESSAGE_SIZE);
SemaphoreHandle_t txSemaphore = xSemaphoreCreateCounting(3,3);
volatile uint32_t step = 0;
volatile uint32_t stepArray[VOICES] = {0};

// ISR's
void sampleISR();
void canRxISR();
void canTxISR();

// Tasks
void scanKeysTask(void * params);
void decodeTask(void * params);
void transmitTask(void * params);

// Task handles 
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;
TaskHandle_t transmitTaskHandle = NULL;

// Helpers
uint32_t getStepSize(uint16_t key);
void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t readCols ();
uint32_t readMatrix();
void setRow (const uint8_t row);
const char * getNote(const uint16_t keys);
void serialPrintBuff(volatile uint8_t * buff);
void updateTxMessage(uint32_t currentState, uint32_t newState, Octave oct);

DMA_HandleTypeDef hdma;
DAC_HandleTypeDef hdac;
TIM_HandleTypeDef htim7;
DAC_ChannelConfTypeDef sConfig;
HAL_StatusTypeDef status;
GPIO_InitTypeDef igpio;
GPIO_TypeDef gpio;


void printStatus (HAL_StatusTypeDef st, char * id) { 
  printf("%s Status: %d\n", id,  status);
}



void initGPIO () { 
  __HAL_RCC_GPIOA_CLK_ENABLE();
  igpio.Pin = GPIO_PIN_4; 
  igpio.Mode = GPIO_MODE_ANALOG;
  igpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(&gpio, &igpio);
}


HAL_StatusTypeDef initDAC() { 
  __HAL_RCC_DAC1_CLK_ENABLE();   
  hdac.Instance = DAC1;
  status = HAL_DAC_Init(&hdac);
  if (status != HAL_OK)
    printStatus(status, (char *)"DAC Init Error");
  
  sConfig = {0};
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE; 
  sConfig.DAC_Trigger = DAC_TRIGGER_T7_TRGO;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;

  return HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1);

}

HAL_StatusTypeDef initDMA() { 
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma.Instance = DMA1_Channel3; 
  hdma.Init.Request = DMA_REQUEST_6;
  hdma.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma.Init.MemInc = DMA_MINC_ENABLE;
  hdma.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
  hdma.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
  hdma.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma.Init.Mode = DMA_CIRCULAR;
  hdma.Init.Priority = DMA_PRIORITY_VERY_HIGH;
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  NVIC_SetPriorityGrouping(0);
  return HAL_DMA_Init(&hdma);
}

HAL_StatusTypeDef initTim7() { 
  __HAL_RCC_TIM7_CLK_ENABLE();
  TIM_MasterConfigTypeDef scfg = {0}; 

  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 64-1; 
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 45-1; 
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_StatusTypeDef tim_status = HAL_TIM_Base_Init(&htim7);
  if (tim_status != HAL_OK)
    printStatus(tim_status, (char *)"Timer Init Error");
  
  scfg.MasterOutputTrigger = TIM_TRGO_UPDATE;
  scfg.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  tim_status = HAL_TIMEx_MasterConfigSynchronization(&htim7, &scfg);
  if (tim_status!=HAL_OK)
    printStatus(tim_status, (char *)"Attach Config To Timer Error");

  return tim_status;
}

struct DataHandle { 
  volatile uint8_t buff[L];
  volatile uint8_t writeCounter;
} typedef  DataHandle_t;

DataHandle_t data;

void setup() {
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

  //Initialise UART
  Serial.begin(9600); 
 

  // DMA 
  status = HAL_Init();
  printStatus(status, (char *) "HAL");
  initGPIO();
  status = initTim7();
  assert(stats == HAL_OK);
  printStatus(status, (char *) "TIM");
  status = initDAC();
  assert(status == HAL_OK);
  printStatus(status, (char *) "DAC");
  status = initDMA();
  assert(status == HAL_OK);
  printStatus(status, (char *) "DMA");
  assert(status == HAL_OK);
  __HAL_LINKDMA(&hdac, DMA_Handle1, hdma);
   
  
  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  // Initialise CAN Bus
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_Start();


  // TODO: Repace with Ping Pong buffer + DMA
  // Initialise DAC ISR
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(DAC_WRITE_FREQ, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  
  // Initialise CAN RX ISR
  CAN_RegisterRX_ISR(canRxISR);
  CAN_RegisterTX_ISR(canTxISR);
  
  
  // Create Tasks
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &scanKeysHandle 
  ); 


  // TODO: Justify Parameters
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decodeMsg",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &decodeTaskHandle 
  );
  

  // TODO: Justify Parameters
  xTaskCreate(
    transmitTask,		/* Function that implements the task */
    "transmitTask",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &transmitTaskHandle 
  );


  // Start Scheduler
  vTaskStartScheduler(); 
}




// ----------------------------------------------------- //
// --------------------   ISR's    --------------------- //
// ----------------------------------------------------- //

void sampleISR()
{
  //uint32_t currentStep = 0;
  uint8_t volume;
  int32_t vout = 0;
  //__atomic_load(&step, &currentStep, __ATOMIC_RELAXED);
  static uint32_t phaseAccArray[VOICES] = {0};
  static uint32_t phaseAcc = 0;
  for (int i = 0; i < VOICES; i++){
    uint32_t currentStep = __atomic_load_n(&stepArray[i], __ATOMIC_RELAXED);
    phaseAccArray[i] += currentStep;
    if (currentStep != 0){
      vout += ((phaseAccArray[i] >> 24) - 128);
    }
  }
  //phaseAcc += currentStep;
  // for (int j = 0; j< VOICES; j++){
  //   if 
  //   vout += ((phaseAccArray[j] >> 24) - 128);
  //   // if (phaseAccArray[j] != 0){
  //   //   vout += ((phaseAccArray[j] >> 24) - 128);
  //   // }else{
  //   //   vout += 0;
  //   // }  
  // }
  //phaseAcc += std::accumulate(phaseAccArray, phaseAccArray + VOICES, 0);
  //int32_t vout = (phaseAcc >> 24) - 128;
  //vout = std::accumulate(phaseAccArray, phaseAccArray + VOICES, 0);
  vout = vout/VOICES;
  volume = context.getVolume(); // Atmoc operation
  vout = vout >> (8 - volume);
  analogWrite(OUTR_PIN, vout + 128);
}


void canRxISR (void) {
	uint8_t rxMessages[8];
	uint32_t ID;
	CAN_RX(ID, rxMessages);
	xQueueSendFromISR(msgInQ, rxMessages, NULL);
}


void canTxISR (void) {
	xSemaphoreGiveFromISR(txSemaphore, NULL);
}

// ----------------------------------------------------- //
// --------------------   Tasks    --------------------- //
// ----------------------------------------------------- //


// Display Task
void loop() {
  static uint32_t next = millis();
  static uint32_t count = 0;

  while (millis() < next);  //Wait for next interval

  next += interval;

  //Update display
  u8g2.clearBuffer();         // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  uint32_t copyState;
  uint8_t volume;
  context.lock(); 
  copyState = context.getState();
  volume = context.getVolume();
  context.unlock();
  const char * note = getNote((uint16_t) copyState & KEY_MASK);
  u8g2.drawStr(2, 20, note);
  u8g2.setCursor(90, 20);
  u8g2.print((uint16_t) copyState, HEX);
  u8g2.setCursor(50, 20);
  u8g2.print(volume, HEX);
  u8g2.setCursor(60, 20);
  u8g2.print( (((uint16_t) txMessages[1]) << 8) + txMessages[0], HEX);
  
  u8g2.sendBuffer();          // transfer internal memory to the display

  
  //Toggle LED
  digitalToggle(LED_BUILTIN);
}


void getStepSizes(uint16_t key) {
  for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
    if ((key & keyMasks[i]) == 0) {
       __atomic_store_n(&stepArray[i], steps[i], __ATOMIC_RELAXED);
    }else{
      __atomic_store_n(&stepArray[i], 0, __ATOMIC_RELAXED);
    }
  }
}

void scanKeysTask(void * params) {

  uint32_t stepValue = 0;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t currentState = 0;
  uint32_t newState = 0;
  Octave oct;
  uint32_t noteArray[VOICES] = {0};
  uint32_t curVoices = 0;

  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xStateUpdateFreq);
    newState = readMatrix();
    context.lock();
    currentState = context.getState();
    context.updateVolume(newState);
    context.setState(newState);
    oct = context.getOctave();
    context.unlock();
    updateTxMessage(currentState, newState, oct);
    xQueueSend(msgOutQ, txMessages, portMAX_DELAY);
    // stepValue = getStepSize(newState & KEY_MASK);
    // __atomic_store_n(&step, stepValue, __ATOMIC_RELAXED);
    getStepSizes(newState & KEY_MASK);
  }

}


// Will wait portMAX_DELAY before continuing, hence portMAX_DELAY is the minimum Initiation interval
void decodeTask(void * params) {
  
  
  uint8_t rxMessage[8];
  
  while (1) {
    xQueueReceive(msgInQ, rxMessage, portMAX_DELAY); 
    // TODO: Implement Mixer in order to respond to incoming messages
    uint8_t octave = rxMessage[1];
    uint8_t msgOut[8] = {octave};
    xSemaphoreTake(txSemaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
    // TODO: Implement Mixer in order to respond to incoming messages
  }
}


void transmitTask(void * params) {
	uint8_t msgOut[8];

	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(txSemaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}

}



// ----------------------------------------------------- // 
// ----------------     Helpers    --------------------- // 
// ----------------------------------------------------- //


uint32_t getStepSize(uint16_t key) {
  for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
    if ((key & keyMasks[i]) == 0) {
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
  for (int i = 0; i < TOTAL_KEYS; ++i)
  {
    if ((keys & keyMasks[i]) == 0)
    {
      return notes[i];
    }
  }
  return notes[12];
}


void updateTxMessage(uint32_t currentState, uint32_t newState, Octave oct) {
  if (currentState == newState) {
    return;
  } 

  uint32_t currentKeyState, newKeyState;  

  for ( auto i = 0; i < TOTAL_KEYS; ++i) {
    currentKeyState = currentState & keyMasks[i];
    newKeyState = newState & keyMasks[i];

    if (currentKeyState != newKeyState) {
      txMessages[0] = (newKeyState == 0) ? KEY_PRESSED : KEY_RELEASED;
      txMessages[2] = i;
      break;
    } 

  }

  txMessages[1] = oct;   
  return;
}



