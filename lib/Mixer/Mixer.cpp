#include "Mixer.hpp"
#include "Octave.hpp"
#include <Arduino.h>


Mixer::Mixer() : stepDataMutex(xSemaphoreCreateMutex()), _keyMessages(0xFFFF), _octave(First) {}

Mixer::~Mixer()
{
  vSemaphoreDelete(stepDataMutex);
}

void Mixer::setKeyMessages(uint16_t keyMessages)
{
  _keyMessages = keyMessages;
}


void Mixer::lock()
{
  xSemaphoreTake(stepDataMutex, portMAX_DELAY);
}


void Mixer::unlock()
{
  xSemaphoreGive(stepDataMutex);
}


void Mixer::setOctave(Octave octave)
{
  _octave = octave;
}


void Mixer::updateSteps(uint32_t newState)
{
  uint16_t stateToPlay = newState & _keyMessages;

  for (auto i = 0; i < VOICES; i++)
  {
    if (~(stateToPlay & (0x1 << i)) >> i) 
    {
      stepArray[i] = steps[i];
    }
    else
    {
      stepArray[i] = 0;
    }
  }
}









