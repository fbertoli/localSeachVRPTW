//
// Created by Bertoli, Francesco (Data61, Canberra City) on 24/02/17.
//
#pragma once

#include "solution.h"
#include "data.h"
#include <vector>
#include "initializerInsertion.h"
#include "options.h"

using namespace std;

/** This class represents an intializer. The method is:
 *  builds a set of routes by choosing each customer as seed and then using the insertion method proposed in: Braysy, Gendrau - Vehicle Routing Problem with Time Windows, Part I. Route Construction and Local Search Algorithms
 *  solve an integer SPP to chose the best set of routes and apply post processing phase to drop double customers*/


class InitializerMultipleRoutes : public InitializerInsertion
{
    /** CONSTRUCTORS */
public:
    InitializerMultipleRoutes(Data &data) : InitializerInsertion(data) {};


    /** METHODS */
public:
    /** build a solution*/
    virtual Solution initializeSolution();


    /** VARIABLES */
};

