#ifndef ARDUINO_PINS_H
#define ARDUINO_PINS_H

#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>
#include <string>
#include <vector>
#include <cstdlib>

/* =========================
   Arduino constants
========================= */

#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1

/* =========================
   Pin simulation
========================= */

static int __pin_mode[100];
static int __pin_state[100];

/* =========================
   Pin functions
========================= */

inline void pinMode(int pin,int mode){

    __pin_mode[pin]=mode;

}

inline void digitalWrite(int pin,int val){

    __pin_state[pin]=val;

    std::cout<<"[digitalWrite] pin "
             <<pin<<" = "<<val<<std::endl;

}

inline int digitalRead(int pin){

    std::cout<<"[digitalRead request] pin "
             <<pin<<" value? ";

    int v;

    std::cin>>v;

    return v;

}

/* =========================
   Analog simulation
========================= */

inline int analogRead(int pin){

    std::cout<<"[analogRead request] pin "
             <<pin<<" value? ";

    int v;

    std::cin>>v;

    return v;

}

/* =========================
   Motor helper
========================= */

inline void motorWrite(int left,int right){

    std::cout<<"[motor] L="
             <<left<<" R="
             <<right<<std::endl;

}

#endif