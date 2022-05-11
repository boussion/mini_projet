/*
 * shortest_edge.h
 *
 *  Created on: 10 mai. 2022
 *      Author: louis_flahault
 */

#ifndef SHORTEST_EDGE_H_
#define SHORTEST_EDGE_H_

#define NSTEP_ONE_TURN              1000    // number of step for 1 turn of the motor
#define CORRECTION_FACTOR           1.05    // correct the angle of rotation to be more precise
#define FULL_PERIMETER_DEG          360.0f     //Degrees needed for the full perimeter
#define CONVERSION_CM_MM            10      // Decimal difference between cm and mm


    void find_shortest_edge(void);
    void halt(void);
    void set_turned(bool new_turn);
    bool get_turned(void);





#endif /* SHORTEST_EDGE_H_ */
