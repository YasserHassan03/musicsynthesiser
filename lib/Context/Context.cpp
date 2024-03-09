#include "Context.hpp"


Context::Context() {
    _state = 0;
    _mutex = xSemaphoreCreateMutex();
}


Context::~Context() {
    vSemaphoreDelete(_mutex);
}


void Context::setState(uint32_t state) {
    _state = state;
}

uint32_t Context::getState() {
    return _state;
}


void Context::lock() {
    xSemaphoreTake(_mutex, portMAX_DELAY);
}


void Context::unlock() {
    xSemaphoreGive(_mutex);
}

