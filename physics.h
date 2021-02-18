/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _PHYSICS_H_
#define _PHYSICS_H_

void generateSprings(const struct world & jello);

// compute Elastic force
void computeElasticForce(double k, double r, const struct point & p1, const struct point & p2, struct point & e);
void computeDamping(double k, const struct point & p1, const struct point & p2, const struct point & v1, const struct point & v2, struct point & d);

void computeAccelerationForSprings(const struct world * jello, struct point a[8][8][8], std::vector<spring> springs, double invM);
void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

// non-physical move down
void MoveDown(struct world * jello);

#endif

