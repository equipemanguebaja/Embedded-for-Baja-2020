/**
* DebouncedInterrupt.cpp
**/
#include "DebouncedInterrupt.h"


DebouncedInterrupt::DebouncedInterrupt(PinName pin)
{
    _in = new InterruptIn(pin);
    _din = new DigitalIn(pin, PullUp);
    _timeout = new Timeout;
}

DebouncedInterrupt::~DebouncedInterrupt()
{
    delete _in;
    delete _din;
    delete _timeout;
}

void DebouncedInterrupt::attach(void (*fptr)(void), const gpio_irq_event trigger, const uint32_t debounce_ms, bool immediate)
{
    if(fptr) {
        _callback = callback(fptr);
        _last_bounce_count = _bounce_count = 0;
        _debounce_us = 1000*debounce_ms;
        _trigger = trigger;
        _timeout_expired = true;
        _immediate = immediate;
        
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
    } else {
        reset();
    }
}

void DebouncedInterrupt::reset()
{
    _timeout->detach();
    _timeout_expired = true;
}

unsigned int DebouncedInterrupt::get_bounce()
{
    return _last_bounce_count;
}

void DebouncedInterrupt::_onCallback()
{
    _last_bounce_count = _bounce_count;
    _bounce_count = 0;
    if (!_immediate) {
        if (_din->read() == (_trigger==IRQ_RISE)) {
            _callback.call();
        }
    }
    _timeout_expired = true;
}

void DebouncedInterrupt::_onInterrupt()
{
    _bounce_count++;

    if (_immediate) {
        if (_timeout_expired) {
            _timeout_expired = false;
            _callback.call();
            _timeout->attach_us(callback(this, &DebouncedInterrupt::_onCallback), _debounce_us);
        }
    }
    else {
        _timeout->attach_us(callback(this, &DebouncedInterrupt::_onCallback), _debounce_us);
    }
}
