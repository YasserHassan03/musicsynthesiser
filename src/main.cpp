#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <mutex>
#include <ES_CAN.h>

// Constants
const uint32_t interval = 100; // Display update interval
// Pin definitions
// Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;
// struct to store system state
struct
{
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  int rotaryState;
} sysState;

class Knob
{
private:
  int rotaryState;
  int lowerLimit;
  int upperLimit;

public:
  Knob(int initialState, int lower, int upper) : rotaryState(initialState), lowerLimit(lower), upperLimit(upper) {}

  void update(int a, int b, int previous_a, int previous_b)
  {

    if ((previous_a == 0 && previous_b == 0 && a == 1 && b == 0) || (previous_a == 1 && previous_b == 1 && b == 1 && a == 0))
    {
      if (rotaryState < upperLimit)
      {
        rotaryState += 1;
      }
    }
    else if ((previous_a == 1 && previous_b == 0 && a == 0 && b == 0) || (previous_a == 0 && previous_b == 1 && b == 1 && a == 1))
    {
      if (rotaryState > lowerLimit)
      {
        rotaryState -= 1;
      }
    }
  }

  void setLimits(int lower, int upper)
  {
    lowerLimit = lower;
    upperLimit = upper;
  }

  int readState()
  {
    return rotaryState;
  }
};

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

const uint32_t stepSizes[] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346, 91007233, 96418697}; // step sizes required for each of the 12 notes of your keyboard
volatile uint32_t currentStepSize;
QueueHandle_t msgInQ;
QueueHandle_t msgOutQ;
SemaphoreHandle_t CAN_TX_Semaphore;
uint8_t RX_Message[8];
Knob volumeKnob(0, 0, 8);
// function to update the phase accumulator and set the analogue output voltage at each sample interval
void sampleISR()
{
  int localStepSize;
  __atomic_load(&currentStepSize, &localStepSize, __ATOMIC_RELAXED);
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - volumeKnob.readState());
  analogWrite(OUTR_PIN, Vout + 128);
}
//incoming queue ISR
void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}
void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		//xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}
void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}
// Function to decode messages from the queue
void decodeTask(void* parameter)
{
  while (1)
  {
    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    int localStepSize;
    if(RX_Message[0] == 'R')
    {
      currentStepSize=0;
    }
    else if(RX_Message[0] == 'P')
    {
      int stepSize = stepSizes[RX_Message[2]];
    }
    currentStepSize = localStepSize << (RX_Message[1]-4);
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
  }
}

// function to read steo size and match to key
uint32_t readStepSize(std::bitset<32> input)
{
  for (int i = 0; i < 12; ++i)
  {
    if ((input.to_ulong() & (1 << i)) == 0)
    {
      return stepSizes[i];
    }
  }
  return 0;
}
// function that matches the step size to the note
std::string getNote(std::bitset<32> input)
{
  const std::string notes[] = {
      "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  for (int i = 0; i < 12; ++i)
  {
    if ((input.to_ulong() & (1 << i)) == 0)
    {
      return notes[i];
    }
  }
  return "no note";
}

int getNoteNumber(std::bitset<32> input)
{

  for (int i = 0; i < 12; ++i)
  {
    if ((input.to_ulong() & (1 << i)) == 0)
    {
      return i;
    }
  }
  return -1;
}

// function to read inputs from the four columns of the switch matrix and store in c++ bitset
std::bitset<4> readCols()
{

  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}
// function to select a given row of the switch matrix by setting the value of each row select address pin
void setRow(uint8_t rowIdx)
{
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
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
// function for scanning the keyboard into a single function:
void scanKeysTask(void *pvParameters)
{
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> inputs;
    for (int i = 0; i < 4; i++)
    {
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> input = readCols();
      inputs |= (input.to_ulong() << (i * 4));
    }
    uint8_t TX_Message[8] = {0};
    int previousNote;
    int currentNote = getNoteNumber(inputs);
    if (currentNote != previousNote)
    {
      TX_Message[0] = 'R';
    }
    else
    {
      TX_Message[0] = 'P';
    }
    TX_Message[1] = 4;
    TX_Message[2] = currentNote;
    xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
    previousNote = currentNote;
    int previous_a;
    int previous_b;

    int a = sysState.inputs[12];
    int b = sysState.inputs[13];
    volumeKnob.update(a, b, previous_a, previous_b);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    previous_a = sysState.inputs[12];
    previous_b = sysState.inputs[13];
    sysState.inputs = inputs;
    int localStepSize = readStepSize(sysState.inputs);
    xSemaphoreGive(sysState.mutex);
    __atomic_store_n(&currentStepSize, localStepSize, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void *pvParameters)
{
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    u8g2.clearBuffer();                 // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    std::string note = getNote(sysState.inputs);
    xSemaphoreGive(sysState.mutex);
    u8g2.drawStr(2, 10, note.c_str());
    u8g2.drawStr(2, 20, std::to_string(volumeKnob.readState()).c_str());

    u8g2.setCursor(66, 30);
    uint32_t ID = 0x123;
    u8g2.print((char)RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    u8g2.sendBuffer(); // transfer internal memory to the display
    digitalToggle(LED_BUILTIN);
  }
}

void setup()
{
  // put your setup code here, to run once:

  // Set pin directions
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

  // Initialise display
  setOutMuxBit(DRST_BIT, LOW); // Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH); // Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH); // Enable display power supply

  // Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  // timer to trigger interrupt to call sampleISR()
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  // initialise CAN
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  //initialise queue variable
  msgInQ = xQueueCreate(36,8);
  // multithreading
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
      scanKeysTask,     /* Function that implements the task */
      "scanKeys",       /* Text name for the task */
      64,               /* Stack size in words, not bytes */
      NULL,             /* Parameter passed into the task */
      1,                /* Task priority */
      &scanKeysHandle); /* Pointer to store the task handle */
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
      displayUpdateTask,     /* Function that implements the task */
      "displayUpdate",       /* Text name for the task */
      64,                    /* Stack size in words, not bytes */
      NULL,                  /* Parameter passed into the task */
      1,                     /* Task priority */
      &displayUpdateHandle); /* Pointer to store the task handle */
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
      decodeTask,     /* Function that implements the task */
      "decode",       /* Text name for the task */
      64,             /* Stack size in words, not bytes */
      NULL,           /* Parameter passed into the task */
      1,              /* Task priority */
      &decodeHandle); /* Pointer to store the task handle */
  sysState.mutex = xSemaphoreCreateMutex();
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);

  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
}