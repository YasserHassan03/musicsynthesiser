#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>

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

// Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

const uint32_t stepSizes [] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346,91007233, 96418697}; // step sizes required for each of the 12 notes of your keyboard
volatile uint32_t currentStepSize;

// function to read steo size and match to key
uint32_t readStepSize(std::bitset<32> input)
{
  if ((input.to_ulong() & 0x1) == 0)
  {
    return stepSizes[0];
  }
  else if ((input.to_ulong() & 0x2) == 0)
  {
    return stepSizes[1];
  }
  else if ((input.to_ulong() & 0x4) == 0)
  {
    return stepSizes[2];
  }
  else if ((input.to_ulong() & 0x8) == 0)
  {
    return stepSizes[3];
  }
  else if ((input.to_ulong() & 0x10) == 0)
  {
    return stepSizes[4];
  }
  else if ((input.to_ulong() & 0x20) == 0)
  {
    return stepSizes[5];
  }
  else if ((input.to_ulong() & 0x40) == 0)
  {
    return stepSizes[6];
  }
  else if ((input.to_ulong() & 0x80) == 0)
  {
    return stepSizes[7];
  }
  else if ((input.to_ulong() & 0x100) == 0)
  {
    return stepSizes[8];
  }
  else if ((input.to_ulong() & 0x200) == 0)
  {
    return stepSizes[9];
  }
  else if ((input.to_ulong() & 0x400) == 0)
  {
    return stepSizes[10];
  }
  else if ((input.to_ulong() & 0x800) == 0)
  {
    return stepSizes[11];
  }
  else
  {
    return 0;
  }
}
// function that matches the step size to the note
std::string getNote(std::bitset<32> input)
{
  if ((input.to_ulong() & 0x1) == 0)
  {
    return "C";
  }
  else if ((input.to_ulong() & 0x2) == 0)
  {
    return "C#";
  }
  else if ((input.to_ulong() & 0x4) == 0)
  {
    return "D";
  }
  else if ((input.to_ulong() & 0x8) == 0)
  {
    return "D#";
  }
  else if ((input.to_ulong() & 0x10) == 0)
  {
    return "E";
  }
  else if ((input.to_ulong() & 0x20) == 0)
  {
    return "F";
  }
  else if ((input.to_ulong() & 0x40) == 0)
  {
    return "F#";
  }
  else if ((input.to_ulong() & 0x80) == 0)
  {
    return "G";
  }
  else if ((input.to_ulong() & 0x100) == 0)
  {
    return "G#";
  }
  else if ((input.to_ulong() & 0x200) == 0)
  {
    return "A";
  }
  else if ((input.to_ulong() & 0x400) == 0)
  {
    return "A#";
  }
  else if ((input.to_ulong() & 0x800) == 0)
  {
    return "B";
  }
  else
  {
    return "no note";
  }
}
// function to update the phase accumulator and set the analogue output voltage at each sample interval
void sampleISR()
{
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
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
  //timer to trigger interrupt to call sampleISR()
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
}

void loop()
{
  // put your main code here, to run repeatedly:
  static uint32_t next = millis();
  static uint32_t count = 0;
  std::bitset<32> inputs;

  while (millis() < next); // Wait for next interval

  next += interval;

  // Update display
  u8g2.clearBuffer();                 // clear the internal memory
  u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  u8g2.drawStr(2, 10, "Helllo World!");
  for (int i = 0; i < 3; i++)
  {
    setRow(i);
    delayMicroseconds(3);
    std::bitset<4> input = readCols();
    inputs |= (input.to_ulong() << (i * 4));
  } // write something to the internal memory
  currentStepSize = readStepSize(inputs);
  std::string note = getNote(inputs);
  u8g2.drawStr(2, 30, note.c_str());

  // std::bitset<4> inputs = readCols();
  u8g2.setCursor(2, 20);
  u8g2.print(inputs.to_ulong(), HEX);
  u8g2.sendBuffer(); // transfer internal memory to the display

  // Toggle LED
  digitalToggle(LED_BUILTIN);
}