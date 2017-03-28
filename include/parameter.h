//
// Created by Bertoli, Francesco (Data61, Canberra City) on 15/02/17.
//

#ifndef LOCALSEARCH_PARAMETER_H
#define LOCALSEARCH_PARAMETER_H

#include <string>

using namespace std;

class Parameter
{
    /** CONSTRUCTOR */
public:
    Parameter(string name, string help, double default_value) : name_(name), help_(help), value_(default_value) {};

    /** METHODS */
public:
//    string help() {return help_;}

    void setValue(double new_value) {value_ = new_value;}

//    string getName() {return name_;}
//
//    double getValue() {return value_;}

    /** VARIABLES */
public:
    string name_;
    string help_;
    double value_;
};




#endif //LOCALSEARCH_PARAMETER_H
