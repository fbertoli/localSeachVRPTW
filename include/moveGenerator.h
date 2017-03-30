//
// Created by Bertoli, Francesco (Data61, Canberra City) on 06/02/17.
//

#ifndef LOCALSEARCH_MOVEGENERATOR_H
#define LOCALSEARCH_MOVEGENERATOR_H

class MoveGenerator {

    /** CONSTRUCTORS */
public:
    MoveGenerator(const Data &data, Solution* sol) : data_(data), current_solution_(sol), has_next_(true) {};


    /** METHODS */
public:
    /** return true if it found a move/false if we reached end and found no move */
    virtual bool first() = 0;

    /** return true if it found a move/false if we reached end and found no move */
    virtual bool next()  = 0;

    /** return true if it found a move */
    virtual bool random() = 0;

    /** whether there is some space left to explore */
    bool hasNext() {return has_next_;};

    /** change the pointe to current solution */
    void setSolutionPntr(Solution* solution) {current_solution_ = solution;}

    /** copy the content of the current move into best_move_ */
    virtual void updateBestMove() = 0;



    /** VARIABLES */
protected:
    /** Data representing the instance */
    const Data &data_;

    /** The current solution */
    Solution* current_solution_;

    /** bool to indicate if there is a next move to check */
    bool has_next_;
};
#endif //LOCALSEARCH_MOVEGENERATOR_H
