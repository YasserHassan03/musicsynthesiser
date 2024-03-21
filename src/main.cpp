#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <U8g2lib.h>
#include <Context.hpp>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <sys/types.h>
#include <ES_CAN.h>
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
#include <list>
#include <numeric>
#include <cmath>

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
#define WEST_MASK 0x800000
#define EAST_MASK 0x8000000
#define RS_MASK 0x100000
#define DAC_WRITE_FREQ 22000
#define KEY_MESSAGE 0x50
#define TOTAL_KEYS 12u
#define MESSAGE_SIZE 8
#define L 64
#define VOICES 24
#define VOICES_PER_BOARD 12
#define SR_MESSAGE 0x98
#define WAVE_MESSAGE 0x99
#define WAVEFORM_MASK 0x2000000
// #define DISABLE_ISR 
// #define DISABLE_THREADS 
// #define TEST_SCANKEYS


// Initiaion Intervals
const TickType_t xStateUpdateFreq = 50/portTICK_PERIOD_MS; // 50ms Initiation Interval
const TickType_t xDecodeFreq = 70/portTICK_PERIOD_MS; // 50ms Initiation Interval
const TickType_t xTransmitFreq = 100/portTICK_PERIOD_MS; // 50ms Initiation Interval
const TickType_t xMixUpdateFreq = 110/portTICK_PERIOD_MS; // 50ms Initiation Interval
// const TickType_t xDebugFreq = 2000/portTICK_PERIOD_MS;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);


// <$> Constant Arrays <$>
const uint32_t steps[12] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346,91007233, 96418697}; 
const uint16_t keyMasks[12] = {0x1, 0x2, 0x4, 0x8, 0x10, 0x20, 0x40, 0x80, 0x100, 0x200, 0x400, 0x800};
const char * notes[13] = {"C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B", "No Note"};
float sine[255];


// TODO: Change Context to singleton as per instruction
// Initialise Global Variables
Context context = Context();
TIM_TypeDef *Instance = TIM1;
HardwareTimer *sampleTimer = new HardwareTimer(Instance);

// TODO: Encapsulate txMessages, and rxMessages in a way that does not use alot of memory, and without dynamic memory allocation
struct { 
  uint8_t txMessages[MESSAGE_SIZE] = {0}; 
  SemaphoreHandle_t txSemaphore = xSemaphoreCreateCounting(3,3);
} Message;

QueueHandle_t msgInQ = xQueueCreate(384, MESSAGE_SIZE);
QueueHandle_t msgOutQ = xQueueCreate(384, MESSAGE_SIZE);

struct { 
  volatile uint32_t stepArray[VOICES] = {0};
  SemaphoreHandle_t stepMutex = xSemaphoreCreateMutex();
} StepData;

volatile uint16_t yAxis = 555;

// ISR's
void sampleISR();
void canRxISR();
void canTxISR();

// Tasks
void scanKeysTask(void * params);
void decodeTask(void * params);
void transmitTask(void * params);
void mixingTask(void * params);

// Task handles 
TaskHandle_t scanKeysHandle = NULL;
TaskHandle_t decodeTaskHandle = NULL;
TaskHandle_t transmitTaskHandle = NULL;
TaskHandle_t mixingTaskHandle = NULL;

// Helpers
uint32_t getStepSize(uint16_t key);
void setOutMuxBit(const uint8_t bitIdx, const bool value);
uint8_t readCols ();
uint32_t readMatrix();
void setRow (const uint8_t row);
const char * getNote(const uint16_t keys);
void serialPrintBuff(volatile uint8_t * buff);
void keyTxMessage(uint32_t newState, Octave oct);
void srTxMessage();
void waveTypeMessage();

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
  for (int i = 0; i < 255; i++)
  {
    sine[i] = (128 * sin((i * 2 * PI) / 255));//populate sine LUT on setup
  }

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
  Serial.println("Starting");

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
  CAN_Init(false);
  setCANFilter(0x123,0x7ff);
  CAN_Start();


  // TODO: Repace with Ping Pong buffer + DMA
  // Initialise DAC ISR
  #ifndef DISABLE_ISR
  sampleTimer->setOverflow(DAC_WRITE_FREQ, HERTZ_FORMAT);
  if (context.getRole() == Receiver) { 
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();
  }
  #endif
  
  // Initialise CAN RX ISR
  CAN_RegisterRX_ISR(canRxISR);
  CAN_RegisterTX_ISR(canTxISR);

  #ifdef TEST_SCANKEYS
  for (int i = 0; i < 200; i++){
    uint8_t testMsg[8] = {0};
    xQueueSend(msgOutQ, testMsg, portMAX_DELAY);
  }
  #endif
  
  #ifndef DISABLE_THREADS
  // Create Tasks
  xTaskCreate (
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &scanKeysHandle 
  ); 


  // TODO: Justify Parameters
  xTaskCreate (
    decodeTask,		/* Function that implements the task */
    "decodeMsg",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &decodeTaskHandle 
  );
  

  // TODO: Justify Parameters
  xTaskCreate (
    transmitTask,		/* Function that implements the task */
    "transmitTask",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &transmitTaskHandle 
  );
  


    // TODO: Justify Parameters
  xTaskCreate (
    mixingTask,		/* Function that implements the task */
    "MIXINGTask",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &mixingTaskHandle 
  );

  // Start Scheduler
  vTaskStartScheduler(); 
  #endif

  #ifdef TEST_SCANKEYS
  uint32_t startTime = micros();
	for (int iter = 0; iter < 20; iter++) {
		mixingTask(NULL);
	}
	Serial.println(micros()-startTime);
	while(1);
  #endif

}


uint32_t genWaveform(uint32_t phaseAcc, uint8_t waveform)
{
  uint32_t scaled = (phaseAcc >> 24);
  switch (waveform)
  {
  case 0:
    return scaled - 128; // sawtooth
  case 1:                // square
    return scaled > 128 ? 127 : -128;
  case 2: // sine
    return (uint32_t)sine[scaled];
  case 3: // triangle
    return scaled < 129 ? 128 - 2 * scaled : -127 + 2 * (scaled - 128);
  default:
    return scaled - 128;
  }
}

float calcPitchBend(uint16_t yAxis)
{
  float pitchBend = (float)yAxis;
  if (pitchBend > 556 || pitchBend < 553)
    return pitchBend > 554 ? 1 + (554 - pitchBend) / 5091.46 : 1 + (554 - pitchBend) / 6000; // posneg pitchbend for semitone
  else
    return 1;
}

// ----------------------------------------------------- //
// --------------------   ISR's    --------------------- //
// ----------------------------------------------------- //

void sampleISR()
{
  uint16_t locBend = __atomic_load_n(&yAxis, __ATOMIC_RELAXED);
  float pitchAmount = calcPitchBend(locBend);
  uint8_t volume;
  int32_t vout = 0;
  uint8_t waveform = context.getWaveform();
  uint8_t scaleDynamic = 1;
  static uint32_t phaseAccArray[VOICES] = {};
  static uint32_t phaseAcc = 0;
  for (int i = 0; i < VOICES; i++)
  {
    uint32_t currentStep = __atomic_load_n(&StepData.stepArray[i], __ATOMIC_RELAXED);
    float modStep = (float)currentStep * pitchAmount;
    phaseAccArray[i] += (uint32_t)modStep;
    if (currentStep != 0)
    {
      scaleDynamic += 1;
      vout += genWaveform(phaseAccArray[i], waveform);
    }
  }
  vout = vout / scaleDynamic;
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
	xSemaphoreGiveFromISR(Message.txSemaphore, NULL);
}

// ----------------------------------------------------- //
// --------------------   Tasks    --------------------- //
// ----------------------------------------------------- //

void drawSineWave()
{
  const int numPoints = 128;
  const int amplitude = 10;
  const int frequency = 2;
  const float phaseShift = 0.1;
  float phaseIncrement = TWO_PI / numPoints * frequency;
  for (int i = 0; i < numPoints; i++)
  {
    float x = map(i, 0, numPoints, 0, u8g2.getDisplayWidth());
    float y = amplitude * sin(i * phaseIncrement + phaseShift) + u8g2.getDisplayHeight() / 2;
    u8g2.drawPixel(x, y);
  }
}

void drawSquareWave()
{
  const int numPoints = 128;
  const int amplitude = 10;
  const int frequency = 2;
  const float phaseShift = 0.1;
  float phaseIncrement = TWO_PI / numPoints * frequency;
  bool high = false;
  float previousY = 0;

  for (int i = 0; i < numPoints; i++)
  {
    float x = map(i, 0, numPoints, 0, u8g2.getDisplayWidth());
    if (i % (numPoints / frequency) == 0)
    {
      high = !high;
    }
    float y;
    if (high)
    {
      y = amplitude + u8g2.getDisplayHeight() / 2;
    }
    else
    {
      y = -amplitude + u8g2.getDisplayHeight() / 2;
    }
    u8g2.drawLine(x - 1, previousY, x, y);
    previousY = y;
  }
}

void drawSawtoothWave()
{
  const int numPoints = 128;
  const int amplitude = 10;
  const int frequency = 2;
  const float phaseShift = 0.1;
  float phaseIncrement = TWO_PI / numPoints * frequency;
  float prevX, prevY;
  for (int i = 0; i < numPoints; i++)
  {
    float x = map(i, 0, numPoints, 0, u8g2.getDisplayWidth());
    float y = amplitude * ((i * phaseIncrement + phaseShift) - floor(i * phaseIncrement + phaseShift)) + u8g2.getDisplayHeight() / 2;
    if (i > 0)
    {
      u8g2.drawLine(prevX, prevY, x, y);
    }
    prevX = x;
    prevY = y;
  }
}

void drawTriangle()
{
  // Draw the triangle wave
  for (int x = 0; x < 128; x++)
  {
    int y = abs((x % 64) - 64 / 2) * 2 * 15 / 64 + 10;
    u8g2.drawPixel(x, y);
  }
}

// Display Task
void loop()
{
  static uint32_t next = millis();
  static uint32_t count = 0;
  while (millis() < next); // Wait for next interval
  next += interval;
  // Update display
  u8g2.clearBuffer();                 // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  uint32_t copyState;
  uint8_t volume;
  uint16_t y = analogRead(JOYY_PIN);
  uint32_t role;
  __atomic_store_n(&yAxis, analogRead(JOYY_PIN), __ATOMIC_RELAXED);
  float pitchBend = (float)y > 554 ? 2 - (float)y / 555 : 1 + (554 - (float)y) / 555;
  uint8_t waveform;
  bool pageToggle;
  bool playback;
  copyState = context.getState();
  role = context.getRole();
  context.lock();
  volume = context.getVolume();
  context.unlock();
  Octave octave = context.getOctave();
  waveform = context.getWaveform();
  pageToggle = context.getPage();

  const char * note = getNote((uint16_t) copyState & KEY_MASK);
  u8g2.drawStr(2, 10, note);
  u8g2.setCursor(2, 20);
  u8g2.print(copyState, HEX);
  u8g2.setCursor(50, 10);
  u8g2.print(volume, HEX);
  u8g2.setCursor(60, 10);
  u8g2.drawStr(60, 10, (role == Sender) ? "S" : "R");
  
  if (context.getRole() == Receiver && !sampleTimer->hasInterrupt()) { 
    sampleTimer->attachInterrupt(sampleISR);
    sampleTimer->resume();
  } else if (context.getRole() == Sender && sampleTimer->hasInterrupt()) { 
    sampleTimer->detachInterrupt();
    sampleTimer->resume();
  } 

  const char *wavetype;
  if (waveform == 0x0)
  {
    wavetype = "Sawtooth";
  }
  else if (waveform == 0x1)
  {
    wavetype = "Square";
  }
  else if (waveform == 0x2)
  {
    wavetype = "Sine";
  }
  else
  {
    wavetype = "Triangle";
  }

  u8g2.setCursor(2, 30);
  u8g2.print(wavetype);

  // const char *note = getNote((uint16_t)copyState & KEY_MASK);
  // if (!pageToggle)
  // {
  //   u8g2.setCursor(1, 10);
  //   u8g2.print("Note: ");
  //   u8g2.print(note);
  //   u8g2.setCursor(2, 20);
  //   u8g2.print("Wave: ");
  //   u8g2.print(wavetype);
  //   u8g2.setCursor(1, 30);
  //   u8g2.print("Vol: ");
  //   u8g2.print(volume, HEX);
  //   u8g2.setCursor(40, 30);
  //   u8g2.print("Oct: ");
  //   u8g2.print(octave, HEX);
 
  //   int y_flat = u8g2.getHeight() - (pitchBend - 0.45) * 15;
  //   u8g2.drawFrame(u8g2.getWidth() - 10, u8g2.getHeight() -21, 10, 21);
  //   u8g2.drawLine(u8g2.getWidth() - 10, y_flat, u8g2.getWidth(), y_flat);
  // }
  // else
  // {
  //   if (wavetype == "Sine")
  //   {
  //     drawSineWave();
  //   }
  //   else if (wavetype == "Square")
  //   {
  //     drawSquareWave();
  //   }
  //   else if (wavetype == "Sawtooth")
  //   {
  //     drawSawtoothWave();
  //   }
  //   else
  //   {
  //     drawTriangle();
  //   }
  // }

  u8g2.sendBuffer(); // transfer internal memory to the display
  
  // Toggle LED
  digitalToggle(LED_BUILTIN);
}


void sendTxMessage(uint32_t currentState, uint32_t newState) {
  if (!((currentState & WEST_MASK) == WEST_MASK) || !((currentState & EAST_MASK) == EAST_MASK)) {
    xQueueSend(msgOutQ, Message.txMessages, portMAX_DELAY);
  }
}

void scanKeysTask(void *params)
{

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t currentState = 0;
  uint32_t newState = 0;
  Octave oct;

  #ifdef TEST_SCANKEYS
  for (int i = 0; i< 13; i++){
    newState = readMatrix();
    context.lock();
    currentState = context.getState();
    context.updateVolume(newState);
    context.setState(newState);
    oct = context.getOctave();
    context.updatePage(newState);
    context.unlock();
    
    if (true) {
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      keyTxMessage(newState, oct);

      if (true) { 
        sendTxMessage(currentState, newState);
      }

      xSemaphoreGive(Message.txSemaphore);
    }
    
    if (true) { 
      context.lock();
      context.inverseRole();
      context.unlock();
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      srTxMessage();
      sendTxMessage(currentState, newState);
      xSemaphoreGive(Message.txSemaphore);
    }

    if (true) { 
      context.lock();
      context.setNextWaveForm(newState);
      context.unlock();
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      waveTypeMessage();
      sendTxMessage(currentState, newState);
      xSemaphoreGive(Message.txSemaphore);
    }
  }
  #else
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xStateUpdateFreq);
    newState = readMatrix();
    context.lock();
    currentState = context.getState();
    context.updateVolume(newState);
    context.setState(newState);
    oct = context.getOctave();
    context.updatePage(newState);
    context.unlock();
    
    if ((currentState & KEY_MASK) != (newState & KEY_MASK)) {
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      keyTxMessage(newState, oct);

      if (context.getRole() == Sender) { 
        sendTxMessage(currentState, newState);
      }

      xSemaphoreGive(Message.txSemaphore);
    }
    
    if ((currentState & RS_MASK) && !(newState & RS_MASK)) { 
      context.lock();
      context.inverseRole();
      context.unlock();
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      srTxMessage();
      sendTxMessage(currentState, newState);
      xSemaphoreGive(Message.txSemaphore);
    }

    if ((currentState & WAVEFORM_MASK) && !(newState & WAVEFORM_MASK)) { 
      context.lock();
      context.setNextWaveForm(newState);
      context.unlock();
      xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
      waveTypeMessage();
      sendTxMessage(currentState, newState);
      xSemaphoreGive(Message.txSemaphore);
    }

  }
  #endif
}


void waveTypeMessage() { 
  Message.txMessages[0] = WAVE_MESSAGE;
  Message.txMessages[1] = context.getWaveform();
}

void srTxMessage() { 
  Message.txMessages[0] = SR_MESSAGE;
}



// Will wait portMAX_DELAY before continuing, hence portMAX_DELAY is the minimum Initiation interval
void decodeTask(void * params) {
  
  uint8_t rxMessage[8];
  Octave oct;
  uint16_t keyState;
  #ifdef TEST_SCANKEYS
    xQueueReceive(msgInQ, rxMessage, portMAX_DELAY); 
    // printf("msg Type= %x\n",  rxMessage[0]); 

    if (true) { 
      // printf("keyState: %x\n", (((uint16_t) rxMessage[2]) << 8) | rxMessage[1]);
      oct = (Octave) rxMessage[3];
      keyState = (((uint16_t) rxMessage[2]) << 8) | rxMessage[1];
      context.lock();
      context.setStateKey(oct, keyState);
      context.unlock();
    } else if (false) { 
      uint32_t role = context.getRole();
      // printf("role: %d\n", role);
      if (role == Receiver) {
        context.lock();
        context.inverseRole();
        context.unlock();
      }
    } else if (false) { 
      context.lock();
      context.setWaveform(rxMessage[1]);
      context.unlock();
    }
  #else
  while (1) {
    xQueueReceive(msgInQ, rxMessage, portMAX_DELAY); 
    printf("msg Type= %x\n",  rxMessage[0]); 

    if (rxMessage[0] == KEY_MESSAGE && context.getRole() == Receiver) { 
      printf("keyState: %x\n", (((uint16_t) rxMessage[2]) << 8) | rxMessage[1]);
      oct = (Octave) rxMessage[3];
      keyState = (((uint16_t) rxMessage[2]) << 8) | rxMessage[1];
      context.lock();
      context.setStateKey(oct, keyState);
      context.unlock();
    } else if (rxMessage[0] == SR_MESSAGE) { 
      uint32_t role = context.getRole();
      printf("role: %d\n", role);
      if (role == Receiver) {
        context.lock();
        context.inverseRole();
        context.unlock();
      }
    } else if (rxMessage[0] == WAVE_MESSAGE) { 
      context.lock();
      context.setWaveform(rxMessage[1]);
      context.unlock();
    }
  }
  #endif
}

void transmitTask(void * params) {
	uint8_t msgOut[8];
  #ifdef TEST_SCANKEYS
  for (int i = 0; i < 5; i++){
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
    // CAN_TX(0x123, msgOut);
    xSemaphoreGive(Message.txSemaphore);
  }
  #else
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(Message.txSemaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
  #endif
}

// ----------------------------------------------------- //
// ----------------     Helpers    --------------------- //
// ----------------------------------------------------- //

uint32_t getStepSize(uint16_t key)
{
  for (uint8_t i = 0; i < TOTAL_KEYS; i++)
  {
    if ((key & keyMasks[i]) == 0)
    {
      return steps[i];
    }
  }
  return 0;
}

// Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

uint8_t readCols()
{
  return ((digitalRead(C3_PIN) & 0x1) << 3) + ((digitalRead(C2_PIN) & 0x1) << 2) + ((digitalRead(C1_PIN) & 0x1) << 1) + (digitalRead(C0_PIN) & 0x1);
}

void setRow(const uint8_t row)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, row & 0x01);
  digitalWrite(RA1_PIN, row & 0x02);
  digitalWrite(RA2_PIN, row & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

uint32_t readMatrix()
{
  uint32_t result = 0;

  for (uint8_t row = 0; row < 9; row++)
  {
    setRow(row);
    delayMicroseconds(2);
    result |= (readCols() << (row * 4));
  }

  return result;
}

// function that matches the step size to the note
const char *getNote(const uint16_t keys)
{
  for (int i = 0; i < TOTAL_KEYS; ++i)
  {
    if ((keys & keyMasks[i]) == 0)
    {
      return notes[i];
    }
  }
  return notes[12];
}


void keyTxMessage(uint32_t newState, Octave oct) {
  Message.txMessages[0] = KEY_MESSAGE; 
  Message.txMessages[1] = (uint8_t) (newState & KEY_MASK);
  Message.txMessages[2] = (uint8_t) ((newState & KEY_MASK) >> 8);
  Message.txMessages[3] = oct;   
  return;
}


void handShakeTask(void * params) { 

  TickType_t xLastWakeTime = xTaskGetTickCount();
  while(1) { 
    vTaskDelayUntil(&xLastWakeTime, xDebugFreq);
    
  }
  vTaskDelete(NULL);
}

void mixingTask(void * params) { 
  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t copyStateArray[4] = {0, 0, 0, 0};

  #ifdef TEST_SCANKEYS
    // context.lock();
    context.setStateKey(context.getOctave(), context.getState() & KEY_MASK);
    for (int i = 0; i < 4; i++) { 
      copyStateArray[i] = context.getStateKey((Octave) i) & KEY_MASK;
    }
    // context.unlock();
    
    for (int j = 0; j < 3; j++ ) { 
      for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
        if ((copyStateArray[j] & keyMasks[i]) == 0) {
          __atomic_store_n(&StepData.stepArray[j * VOICES_PER_BOARD + i], steps[i] << j, __ATOMIC_RELAXED);
        } else { 
          __atomic_store_n(&StepData.stepArray[j * VOICES_PER_BOARD + i], 0, __ATOMIC_RELAXED);
        }
      }
    }
  #else
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xStateUpdateFreq);
    
    context.lock();
    context.setStateKey(context.getOctave(), context.getState() & KEY_MASK);
    for (int i = 0; i < 4; i++) { 
      copyStateArray[i] = context.getStateKey((Octave) i) & KEY_MASK;
    }
    context.unlock();
    
    for (int j = 0; j < 2; j++ ) { 
      for (uint8_t i = 0; i < TOTAL_KEYS; i++) {
        if ((copyStateArray[j] & keyMasks[i]) == 0) {
          __atomic_store_n(&StepData.stepArray[j * VOICES_PER_BOARD + i], steps[i] << j, __ATOMIC_RELAXED);
        } else { 
          __atomic_store_n(&StepData.stepArray[j * VOICES_PER_BOARD + i], 0, __ATOMIC_RELAXED);
        }
      }
    }

  }
  #endif
}


