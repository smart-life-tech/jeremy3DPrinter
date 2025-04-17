#ifndef ESP32_ENCODER_H
#define ESP32_ENCODER_H

#include <Arduino.h>

class ESP32Encoder {
private:
    int8_t _oldState;
    int _pin1, _pin2;
    volatile long _position;
    
public:
    ESP32Encoder(uint8_t pin1, uint8_t pin2) {
        _pin1 = pin1;
        _pin2 = pin2;
        _position = 0;
        _oldState = 0;
        
        pinMode(_pin1, INPUT_PULLUP);
        pinMode(_pin2, INPUT_PULLUP);
    }
    
    void tick() {
        int8_t newState = 0;
        if (digitalRead(_pin1)) newState |= 1;
        if (digitalRead(_pin2)) newState |= 2;
        
        switch (_oldState) {
            case 0:
                if (newState == 1) _position++;
                else if (newState == 2) _position--;
                break;
            case 1:
                if (newState == 3) _position++;
                else if (newState == 0) _position--;
                break;
            case 2:
                if (newState == 0) _position++;
                else if (newState == 3) _position--;
                break;
            case 3:
                if (newState == 2) _position++;
                else if (newState == 1) _position--;
                break;
        }
        
        _oldState = newState;
    }
    
    long read() {
        tick();
        return _position;
    }
    
    void write(long p) {
        _position = p;
    }
};

#endif // ESP32_ENCODER_H