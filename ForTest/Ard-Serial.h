#ifndef ARDUINO_SERIAL_H
#define ARDUINO_SERIAL_H

#include <iostream>
#include <chrono>
#include <thread>
#include <cstdint>
#include <string>
#include <vector>
#include <cstdlib>

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

#endif