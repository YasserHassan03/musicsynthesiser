#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <cstdint>


// Small Enum no need for seperate header
enum Octave {
  First = 0, 
  Second = 1, 
  Third = 2, 
  Fourth = 3
};


enum Role { 
  Sender = 0x1, 
  Receiver = 0x0
};

// enum Waveform {
//   sawtooth,
//   square,
//   sine
// };


// TODO: Add Octave for Handshake
class Context {
private:
    uint32_t _state;
    SemaphoreHandle_t _mutex;  
    uint8_t _volume;
    uint8_t _lowerLimit;
    uint8_t _upperLimit;
    Octave _octave;
    Role _role;
    std::unordered_map<Octave, uint32_t> _neighborStates;
    uint8_t _waveform;
    uint8_t _playback;

    bool _Page;
    
    
public:


     Context();
    ~Context();

    void setState(uint32_t state);
    void lock();
    void unlock();
    void updateVolume(uint32_t newState);
    void chooseWaveform(uint32_t newState);
    void setVolumeLimits(int lower, int upper);
    void inverseRole();
    void setOctave(Octave octave);
    void setNeighborState(Octave octave, uint32_t state);
    void updatePage(uint32_t newState);

    
    void setNextWaveForm(uint32_t newState);
    void setWaveform(uint8_t waveform);

    // Getters are defined as inline for atomic operation -> i.e no jump instruction to get a value,
    // The function call gets replaced with a ld instruction. This is so long as all our getters are <= 32 bits
    inline uint8_t getVolume() { return _volume; };
    inline uint32_t getRole() { return _role; }; 
    inline uint32_t getState() { return _state; };
    inline Octave getOctave() { return _octave; };
    uint32_t getNeighborState(Octave octave);
    inline uint8_t getWaveform() { return _waveform; };
    inline bool getPage() { return _Page; };

  
};



