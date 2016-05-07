
#include <stdlib.h>
#include <string.h>
#include <Ultrasonic.h>


Ultrasonic::Ultrasonic(int tp, int ep)
    {
    pinMode(tp, OUTPUT);
    pinMode(ep, INPUT);
    _trigPin = tp;
    _echoPin = ep;
    _cmDivisor = 27.6233;
    _inDivisor = 70.1633;
    }

long Ultrasonic::timing()
    {
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
    return pulseIn(_echoPin, HIGH);
    }

float Ultrasonic::convert(long microsec, int metric)
    {
    // microsec / 29 / 2;
    if(metric) return microsec / _cmDivisor / 2.0;  // CM
    // microsec / 74 / 2;
    else return microsec / _inDivisor / 2.0;  // IN
    }

void Ultrasonic::setDivisor(float value, int metric)
    {
    if(metric) _cmDivisor = value;
    else _inDivisor = value;
    }
