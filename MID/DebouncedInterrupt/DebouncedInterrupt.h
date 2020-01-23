 
#ifndef DEBOUNCED_INTERRUPT_H
#define DEBOUNCED_INTERRUPT_H

#include <stdint.h>
#include "mbed.h"

/**
typedef enum {
    IRQ_NONE,
    IRQ_RISE,
    IRQ_FALL
} gpio_irq_event;
**/

/** A replacement for InterruptIn that debounces the interrupt
 *
 * Example:
 * @code
 *
 * #include "DebouncedInterrupt.h"
 *
 * DebouncedInterrupt up_button(USER_BUTTON);
 * 
 * void onUp()
 * {
 *    // Do Something
 * }
 * 
 * int main()
 * {
 *     // Will immediatly call function and ignore other interrupts until timeout
 *     up_button.attach(&onUp, IRQ_FALL, 1000, true);
 *
 *     // Will call function only if button has been held for the specified time
 *     //up_button.attach(&onUp, IRQ_FALL, 500, false);
 *
 *     while(1) {
 *         ...
 *     }
 * }
 * @endcode
 */
class DebouncedInterrupt {
private:
    unsigned int _debounce_us;
    bool _immediate;
    bool _timeout_expired;
    InterruptIn *_in;
    DigitalIn *_din;
    gpio_irq_event _trigger;
    Timeout *_timeout;
    
    // Diagnostics
    volatile unsigned int _bounce_count;
    volatile unsigned int _last_bounce_count;
    
    void _onInterrupt(void);
    void _onCallback(void);
public:
    DebouncedInterrupt(PinName pin);
    ~DebouncedInterrupt();
    
    // Start monitoring the interupt and attach a callback
    void attach(void (*fptr)(void), const gpio_irq_event trigger, const uint32_t debounce_ms=10, bool immediate=false);
    
    template<typename T, typename M>
    void attach(T *obj, M method, const gpio_irq_event trigger, const uint32_t debounce_ms=10, bool immediate=false) {
        _callback = callback(obj, method);
        _last_bounce_count = _bounce_count = 0;
        _debounce_us = 1000*debounce_ms;
        _trigger = trigger;
        
        switch(trigger)
        {
            case IRQ_RISE:
                _in->rise(callback(this, &DebouncedInterrupt::_onInterrupt));
                break;
            case IRQ_FALL:
                _in->fall(callback(this, &DebouncedInterrupt::_onInterrupt));
                break;
            case IRQ_NONE:
                reset(); // Unexpected. Clear callbacks.
                break;
        }
    }
   
    // Stop monitoring the interrupt
    void reset();
    
    
    /*
    * Get number of bounces 
    * @return: bounce count
    */
    unsigned int get_bounce();
protected:
//    https://github.com/mbedmicro/mbed/blob/master/libraries/mbed/api/FunctionPointer.h
    Callback<void()> _callback;
};
#endif