#include <Arduino.h>
#include <STM32FreeRTOS.h> 
#include "Octave.hpp"


class Mixer { 
private:
  static const uint8_t VOICES = 12;
  volatile uint32_t stepArray[VOICES] = {0};
  SemaphoreHandle_t stepDataMutex;
  uint16_t _keyMessages;
  Octave _octave;
    
public:

  Mixer();
  ~Mixer();
  inline uint32_t getStep(uint8_t voice) { return stepArray[voice]; }
  void setKeyMessages(uint16_t keyMessages);
  void setOctave(Octave octave);
  void updateSteps(uint32_t newState);


  void lock();
  void unlock();
  inline uint16_t getKeyMessages() { return _keyMessages; };
  inline Octave getOctave() { return _octave; };


  const uint32_t steps[12] = {49977801, 54113269, 57330981, 60740013, 64351885, 68178311, 72232370, 76527532, 81078245, 85899346,91007233, 96418697}; 
};