#include "Context.hpp"
#include <cstdint>


Context::Context()
  :_state(0), _mutex(xSemaphoreCreateMutex()), _volume(4), _lowerLimit(0), _upperLimit(8), 
  _octave(First), _waveform(0), _Page(false), _role(Receiver), _stateKeys{0xfff, 0xfff, 0xfff, 0xfff}
  {}

Context::~Context() {
  vSemaphoreDelete(_mutex);
}

void Context::setState(uint32_t state) {
  _state = state;
}

void Context::lock() {
  xSemaphoreTake(_mutex, portMAX_DELAY);
}

void Context::unlock() {
  xSemaphoreGive(_mutex);
}

void Context::setVolumeLimits(int lower, int upper)
{
  _lowerLimit = lower;
  _upperLimit = upper;
}

// TODO: Rewrite as a function of current state and new state -> Reduces insructions and load on CPU
void Context::updateVolume(uint32_t newState)
{
  uint8_t previous_a = (_state & 0x4000) >> 14;
  uint8_t previous_b = (_state & 0x8000) >> 15;
  uint8_t a = (newState & 0x4000) >> 14;
  uint8_t b = (newState & 0x8000) >> 15;
  
  
  
  if ((previous_a == 0 && previous_b == 0 && a == 1 && b == 0) || (previous_a == 1 && previous_b == 1 && b == 1 && a == 0))
  {
    if (_volume < _upperLimit)
    {
      _volume += 1;
    }
  }
  else if ((previous_a == 1 && previous_b == 0 && a == 0 && b == 0) || (previous_a == 0 && previous_b == 1 && b == 1 && a == 1))
  {
    if (_volume > _lowerLimit)
    {
      _volume -= 1;
    }
  }
}



void Context::inverseRole() { 
  _role = (Role) !_role;
}


void Context::setOctave(Octave octave) {
  _octave = octave;
}


void Context::updatePage(uint32_t newState)
{
  uint8_t click = (newState & 0x200000) >> 21;
  if (click == 0)//active low
  {
    _Page = !_Page;
  }
}

void Context::setNextWaveForm(uint32_t newState)
{
  _waveform = (_waveform + 1) % 4;
}


void Context::setWaveform(uint8_t waveform)
{
  _waveform = waveform;
}


void Context::setStateKey(Octave octave, uint32_t state)
{
  _stateKeys[octave] = state;
}

