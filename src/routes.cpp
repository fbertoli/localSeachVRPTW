//
// Created by Bertoli, Francesco (Data61, Canberra City) on 27/02/17.
//

#include "routes.h"
/** ROUTES CLASS ------------------------------------------------------------------------
 * ---------------------------------------------------------------------------------------*/
Route::Route() : stops_(), cost_(0) {}

Route::Route(vector<int> stops, double cost_) : stops_(stops), cost_(cost_) {}

Route& Route::operator= (const Route &route) {
    stops_ = route.stops_;
    cost_ =route.cost_;
    return *this;
}





/** ROUTES CONTAINER CLASS ----------------------------------------------------------------
 * ---------------------------------------------------------------------------------------*/
RouteContainer::RouteContainer(Data& data) :
        data_(data),
        env_(new GRBEnv()),
        integer_model_(GRBModel(*env_)),
        to_include_integer_(0),
        solution_(data_),
        routes_(),
        optimization_time_("time","the time allowed to integer_model_", 1000)
//        linear_model_(GRBModel(*env_)),
//        to_integrate_linear_(0),
{
    routes_.clear();
    integer_model_.set(GRB_StringAttr_ModelName, "SPP-MIP");
    integer_model_.getEnv().set(GRB_IntParam_OutputFlag, 0);
//    dual_variables_.resize(data_.n_requests_,0);
//    linear_model_.set(GRB_StringAttr_ModelName, "SPP-LP");
//    linear_model_.getEnv().set(GRB_IntParam_OutputFlag, 0);
//    out_file_.open(data_.name_ + "-duals.txt");
//    for (int c = 0; c < data_.n_requests_; ++c)
//        out_file_ << "Req " << c << whiteSpaces(c, 4);
}

/** ------------------------------------------------------------------------------------------------ */

RouteContainer::~RouteContainer() {
    delete env_;
//    out_file_.close();
}

/** ------------------------------------------------------------------------------------------------ */


void RouteContainer::extractRoutes(Solution &solution, vector<int> index_routes) {
    if (index_routes.size()) {
        Route new_route;
        for (auto & index : index_routes) {
            new_route.stops_.clear();
            new_route.stops_.insert(new_route.stops_.begin(),solution.tour_.cbegin() + solution.start_positions_[index] + 1,
                                         solution.tour_.cbegin() + solution.start_positions_[index + 1]);
//            new_route.arrivals_.resize( new_route.stops_.size());
//            for (int j = solution.start_positions_[index] + 1; j < solution.start_positions_[index + 1]; ++j)
//                new_route.arrivals_[j - solution.start_positions_[index] - 1] = solution.arrivals_[solution.tour_[j]];
            new_route.cost_ = solution.routes_costs_[index];
            routes_.push_back(new_route);
        }
    }
    else {
        Route new_route;
        for (int i = 0; i < solution.n_routes_; ++i) {
            new_route.stops_.clear();
            new_route.stops_.insert(new_route.stops_.begin(), solution.tour_.begin() + solution.start_positions_[i] + 1,
                                                    solution.tour_.begin() + solution.start_positions_[i + 1]);
//            new_route.arrivals_.resize( new_route.stops_.size());
//            for (int j = solution.start_positions_[i] + 1; j < solution.start_positions_[i + 1]; ++j)
//                new_route.arrivals_[j - solution.start_positions_[i] - 1] = solution.arrivals_[solution.tour_[j]];
            new_route.cost_ = solution.routes_costs_[i];
            routes_.push_back(new_route);
        }
    }


}


/** ------------------------------------------------------------------------------------------------ */


void RouteContainer::createIntegerModel() {
    // left hand sides of constraints
    vector<GRBLinExpr> left_hand_sides(data_.n_requests_, 0);

    // Variables
    for (int r = 0; r < routes_.size(); ++r) {
        integer_vars_.push_back(integer_model_.addVar(0, 1, routes_[r].cost_, GRB_BINARY,
                                                      "route-" + to_string(r)));
        for (auto & stop : routes_[r].stops_ )
            left_hand_sides[stop] += integer_vars_.back();
    }

    // Update sense and model
    integer_model_.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    integer_model_.update();

    // Constraints
    for (int i = 0; i < data_.n_requests_; ++i)
        integer_constraints_.push_back(
                integer_model_.addConstr(left_hand_sides[i], GRB_GREATER_EQUAL, 1, "customer-" + to_string(i)));

    to_include_integer_ = routes_.size() - 1;
    integer_model_.update();

}


/** ------------------------------------------------------------------------------------------------ */

void RouteContainer::optimizeIntegerModel(double time, double gap) {
//    integer_model_.getEnv().set(GRB_IntParam_OutputFlag, 1);
    if (time)
        integer_model_.getEnv().set(GRB_DoubleParam_TimeLimit, time);
    if (gap)
        integer_model_.getEnv().set(GRB_DoubleParam_MIPGap, gap);

    integer_model_.optimize();
//    integer_model_.getEnv().set(GRB_IntParam_OutputFlag, 0);
}


/** ------------------------------------------------------------------------------------------------ */


void RouteContainer::updateIntegerModel()  {
    for (int r = to_include_integer_ + 1; r < routes_.size(); ++r) {
        int customers_in_route(routes_[r].stops_.size());
        GRBConstr *constraints_involved = new GRBConstr[customers_in_route];
        double *coefficients = new double[customers_in_route];
        for (int i = 0; i < customers_in_route; ++i) {
            constraints_involved[i] = integer_constraints_[routes_[r].stops_[i]];
            coefficients[i] = 1;
        }

        integer_vars_.push_back(integer_model_.addVar(0, 1, routes_[r].cost_, GRB_BINARY, customers_in_route,
                                                      constraints_involved, coefficients,
                                                      "route-" + to_string(r)));

        delete[] constraints_involved;
        delete[] coefficients;
    }
    to_include_integer_ = routes_.size() - 1;
    integer_model_.update();
}





/** ------------------------------------------------------------------------------------------------ */

void RouteContainer::updateIntegerSolution() {
    vector<bool> inserted(data_.n_requests_, false);
    vector<int> tour{data_.n_requests_};
    int routes = 0;
    for (int r = 0; r < routes_.size(); ++r) {
        if (integer_vars_[r].get(GRB_DoubleAttr_X) > 0.5) {
            for (int i =0; i < routes_[r].stops_.size(); ++i) {
                if (!inserted[routes_[r].stops_[i]]) {
                    tour.push_back(routes_[r].stops_[i]);
                    inserted[routes_[r].stops_[i]] = true;
                }
            }
            if (tour.back() < data_.depot_) {
                ++routes;
                tour.push_back(data_.n_requests_ + routes);
            }
        }
    }
    solution_ = Solution(data_, tour);
}




/** ------------------------------------------------------------------------------------------------ */

//string RouteContainer::whiteSpaces(double d, int spaces) {
//    static int precision = 100;
//
//    int digits;
//    if (d < 10)
//        digits = 1;
//    else if (d < 100)
//        digits = 2;
//    else if (d < 1000)
//        digits = 3;
//    else
//        digits = 4;
//
//    string str("");
//    for (int j =0; j < spaces - digits; ++j)
//        str +=" ";
//    return str;
//
//}



/** ------------------------------------------------------------------------------------------------ */

//void RouteContainer::writeDualVariables() {
//    for (int c = 0; c < data_.n_requests_; ++c)
//        out_file_ << dual_variables_[c] << whiteSpaces(dual_variables_[c]);
//}

/** ------------------------------------------------------------------------------------------------ */


//void RouteContainer::printEndBasicSearch() {
//    out_file_ << "--------------------------------------------------------------------------------------------------------------";
//}


/** ------------------------------------------------------------------------------------------------ */

//
//void RouteContainer::updateDualVariables() {
//    for (int i = 0; i < data_.n_requests_; ++i)
//        dual_variables_[i] = linear_vars_[i].get(GRB_DoubleAttr_X);
//}

/** ------------------------------------------------------------------------------------------------ */

//void RouteContainer::createLinearModel() {
//
//    // Variables
//    for (int i = 0; i < data_.n_requests_; ++i)
//        linear_vars_.push_back(linear_model_.addVar(0, 100000, 1, GRB_CONTINUOUS, "customer-" + to_string(i)));
//
//    // Update sense and model
//    linear_model_.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
//    linear_model_.update();
//
//    // Contraints
//    for (int r = 0; r < routes_.size(); ++r) {
//        // create lhs
//        GRBLinExpr linear_lhs(0);
//        for (auto & stop : routes_[r].stops_ )
//            linear_lhs += linear_vars_[stop];
//        // add constraints
//        linear_constraints_.push_back(
//                linear_model_.addConstr(linear_lhs, GRB_LESS_EQUAL, routes_[r].cost_, "route-" + to_string(r)));
//    }
//
//    to_integrate_linear_ = routes_.size() - 1;
//}



/** ------------------------------------------------------------------------------------------------ */



//void RouteContainer::updateLinearModel() {
//    for (int i = to_integrate_linear_ + 1; i < routes_.size(); ++i) {
//        // Add constraint to linear model
//        GRBLinExpr linear_lhs;
//        for (auto & stop : routes_[i].stops_ )
//            linear_lhs += linear_vars_[stop];
//        linear_constraints_.push_back(
//                linear_model_.addConstr(linear_lhs, GRB_LESS_EQUAL, routes_[i].cost_, "route-" + to_string(i)));
//    }
//    to_integrate_linear_ = routes_.size() - 1;
//}



