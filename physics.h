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
// springs forces
void computeAccelerationForSprings(const struct world * jello, struct point a[8][8][8], std::vector<spring> springs, double invM);

// collision detection and response
bool isPointInsideBBox(const point &p, bbox b);
bool isPointInPositiveSide(const point &pt, const plane &pl);
point computeClosestPoint(const point &pt, const plane &pl);
bool checkCollisions(const struct world * jello, std::vector<indices> &points, std::vector<point> &closestPos);
void collisionResponse(std::vector<indices> &points, std::vector<point> &closestPos, std::vector<collisionSpring> &collisionSprings);
void computeAccelerationForCollisions(const struct world * jello, struct point a[8][8][8], double invM);

// external forces
point computeCellWidth(const struct world & jello, bbox &box);
point computeCellIndex(const struct world & jello, const point &p, const bbox &box);
point computeBarycentricCoord(const point &p, point &cellIndex, point &cellWidth, const bbox &box);
std::vector<point> computeNeighborForces(const struct world & jello, const point &p, point &cellOrigin);
point interpolate(const point &bc, const std::vector<point> &forces);
void computeAccelerationForExternalForces(const struct world * jello, struct point a[8][8][8], double invM);

void computeAcceleration(struct world * jello, struct point a[8][8][8]);

// perform one step of Euler and Runge-Kutta-4th-order integrators
// updates the jello structure accordingly
void Euler(struct world * jello);
void RK4(struct world * jello);

// non-physical move down
void MoveDown(struct world * jello);

#endif

