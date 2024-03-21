#include <Arduino.h>
#include <STM32FreeRTOS.h>


// Small Enum no need for seperate header
enum Octave {
  First = 1, 
  Second = 2, 
  Third = 3, 
  Fourth = 4
};


enum Role { 
  Sender = 0x1, 
  Receiver = 0x2
};


// TODO: Add Octave for Handshake
// For now Octave is Fixed
class Context {
private:
    uint32_t _state;
    SemaphoreHandle_t _mutex;  
    uint8_t _volume;
    uint8_t _lowerLimit;
    uint8_t _upperLimit;
    Octave _octave;
    Role _role;   
    
    
public:


     Context();
    ~Context();

    void setState(uint32_t state);
    void lock();
    void unlock();
    void updateVolume(uint32_t newState);
    void setVolumeLimits(int lower, int upper);
    void setRole();
    


    // Getters are defined as inline for atomic operation -> i.e no jump instruction to get a value,
    // The function call gets replaced with a ld instruction. This is so long as all our getters are <= 32 bits
    inline uint8_t getVolume() { return _volume; };
    inline Role getRole() { return _role; }; 
    inline uint32_t getState() { return _state; };
    inline Octave getOctave() { return _octave; };

};



