#ifndef ARDUINO_SIM_H
#define ARDUINO_SIM_H

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

#define HIGH 1
#define LOW 0

#define INPUT 0
#define OUTPUT 1

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

/* =========================
   Serial class
========================= */

class SerialClass{

public:

    void begin(int baud){

        std::cout<<"[Serial begin "
                 <<baud<<"]"<<std::endl;

    }

    template<typename T>
    void print(T val){

        std::cout<<val;

    }

    template<typename T>
    void println(T val){

        std::cout<<val<<std::endl;

    }

    void println(){

        std::cout<<std::endl;

    }

    int available(){

        return std::cin.rdbuf()->in_avail();

    }

    char read(){

        char c;

        std::cin.get(c);

        return c;

    }

};

static SerialClass Serial;

/* =========================
   Utility debug
========================= */

inline void log(std::string s){

    std::cout<<"[LOG] "<<s<<std::endl;

}

#endif