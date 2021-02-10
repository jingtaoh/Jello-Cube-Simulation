/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void computeElasticForce(double k, double r, struct point p1, struct point p2, struct point e);
void computeDamping(double k, struct point v, struct point d);

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

// non-physical move down
void MoveDown(struct world * jello);

#endif

