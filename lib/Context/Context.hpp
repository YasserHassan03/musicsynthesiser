#include <Arduino.h>
#include <STM32FreeRTOS.h>


class Context {
private:
    uint32_t _state;
    SemaphoreHandle_t _mutex;  
    uint8_t _volume;
    uint8_t _lowerLimit;
    uint8_t _upperLimit;

public:


     Context();
    ~Context();

    void setState(uint32_t state);
    inline uint32_t getState() { return _state; };
    void lock();
    void unlock();
    void updateVolume(uint32_t newState);
    void setVolumeLimits(int lower, int upper);
    inline uint8_t getVolume() { return _volume; };

};
