#ifndef ARDUINO_CORE_H
#define ARDUINO_CORE_H

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

using namespace std;

typedef uint8_t byte;

/* =========================
   Time system
========================= */

static auto __start_time = std::chrono::steady_clock::now();

inline unsigned long millis(){

    auto now = std::chrono::steady_clock::now();

    return std::chrono::duration_cast
    <std::chrono::milliseconds>(now-__start_time).count();

}

inline unsigned long micros(){

    auto now = std::chrono::steady_clock::now();

    return std::chrono::duration_cast
    <std::chrono::microseconds>(now-__start_time).count();

}

inline void delay(unsigned long ms){

    std::this_thread::sleep_for(std::chrono::milliseconds(ms));

}

/* =========================
   Math helpers
========================= */

template<typename T>
T constrain(T x,T a,T b){

    if(x<a) return a;
    if(x>b) return b;

    return x;

}

inline long map(long x,long in_min,long in_max,long out_min,long out_max){

    return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;

}

/* =========================
   Utility debug
========================= */

inline void log(std::string s){

    std::cout<<"[LOG] "<<s<<std::endl;

}

#endif