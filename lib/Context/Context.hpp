#include <Arduino.h>
#include <STM32FreeRTOS.h>


class Context {
private:
    uint32_t _state;
    SemaphoreHandle_t _mutex;  

public:


    Context();
    ~Context();

    void setState(uint32_t state);
    uint32_t getState();
    void lock();
    void unlock();

};