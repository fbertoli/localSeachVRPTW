//
// Created by Bertoli, Francesco (Data61, Canberra City) on 28/02/17.
//

#ifndef LOCALSEARCH_INITIALIZERCOSTS_H
#define LOCALSEARCH_INITIALIZERCOSTS_H

#include "options.h"
#include "data.h"

/** function for costs  ----------------------------------------------------------------------------------------
 *  ------------------------------------------------------------------------------------------------------------*/

/** compute the cost of inserting visit between predecessor and successor with the gives loss in departure time */
double positionCost(int predecessor, int visit, int successor, double lost_departure_time, Data& data);

/** ------------------------------------------------------------------------------------------------ */


/** compute the cost of the insertion (only needed if we want a double criteria) */
double insertionCost(int customer, double best_position_cost, int switcher, vector<double> *profits, Data& data) ;

#endif //LOCALSEARCH_INITIALIZERCOSTS_H
