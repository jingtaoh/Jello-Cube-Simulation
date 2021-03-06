/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

/**
 * Generate mass spring system
 * @param jello - Jello structure
 */
void generateSprings(const struct world & jello)
{
    int i,j,k,ip,jp,kp;

    #define PROCESS_NEIGHBOUR(di,dj,dk,springType) \
        ip=i+(di);\
        jp=j+(dj);\
        kp=k+(dk);\
        if\
        (!( (ip>7) || (ip<0) ||\
            (jp>7) || (jp<0) ||\
            (kp>7) || (kp<0) )) \
        {\
            (springType).push_back(spring(i, j, k, ip, jp, kp, scale)); \
        }\

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                double scale;
                // structural springs
                {
                    scale = 1.0;
                    PROCESS_NEIGHBOUR(1,0,0,structuralSprings);
                    PROCESS_NEIGHBOUR(0,1,0,structuralSprings);
                    PROCESS_NEIGHBOUR(0,0,1,structuralSprings);
                }
                // shear springs
                {
                    scale = sqrt(2.0);
                    PROCESS_NEIGHBOUR(1,1,0,shearSprings);
                    PROCESS_NEIGHBOUR(-1,1,0,shearSprings);
                    PROCESS_NEIGHBOUR(0,1,1,shearSprings);
                    PROCESS_NEIGHBOUR(0,-1,1,shearSprings);
                    PROCESS_NEIGHBOUR(1,0,1,shearSprings);
                    PROCESS_NEIGHBOUR(-1,0,1,shearSprings);

                    scale = sqrt(3.0);
                    PROCESS_NEIGHBOUR(1,1,1,shearSprings);
                    PROCESS_NEIGHBOUR(-1,1,1,shearSprings);
                    PROCESS_NEIGHBOUR(-1,-1,1,shearSprings);
                    PROCESS_NEIGHBOUR(1,-1,1,shearSprings);
                }
                // bend springs
                {
                    scale = 2.0;
                    PROCESS_NEIGHBOUR(2,0,0,bendSprings);
                    PROCESS_NEIGHBOUR(0,2,0,bendSprings);
                    PROCESS_NEIGHBOUR(0,0,2,bendSprings);
                }
            }
}

/**
 * Compute elastic force between two mass points
 * @param k - Hook's elasticity coefficient
 * @param r - rest length
 * @param p1 - Position of 1st mass point
 * @param p2 - Position of 2nd mass point
 * @param e - Elastic force
 */
void computeElasticForce(double k, double r, const struct point & p1, const struct point & p2, struct point & e)
{
    // E = -k_h * (|L| - R) * (L/|L|)

    point l;
    pDIFFERENCE(p1, p2, l);
    double length;
    pNORMALIZE(l); // compute length
    pMULTIPLY(l, -k * (length - r), e);
}

 /**
  * Compute damping force between two mass points
  * @param k - Damping coefficient
  * @param p1 - Position of 1st mass point
  * @param p2 - Position of 2nd mass point
  * @param v1 - Velocity of 1st mass point
  * @param v2 - Velocity of 2nd mass point
  * @param d - Damping force
  */
void computeDamping(double k, const struct point & p1, const struct point & p2, const struct point & v1, const struct point & v2, struct point & d)
{
    // D = -k_d * <(v1 - v2), (L/|L|)> * (L/|L|)

    point l;
    pDIFFERENCE(p1, p2, l);
    point v;
    pDIFFERENCE(v1, v2, v);
    double length;
    pNORMALIZE(l);
    double velocity;
    DOTPRODUCTp(v, l, velocity);
    pMULTIPLY(l, -k * velocity, d);
}

/**
 * compute acceleration for a type of spring
 * @param jello - jello state
 * @param a - acceleration array
 * @param springs - spring type
 * @param invM - inverse of mass
 */
void computeAccelerationForSprings(const struct world * jello, struct point a[8][8][8], std::vector<spring> springs, double invM)
{
    for (const auto &s : springs)
    {
        point e,d;
        // compute elastic force and damping
        computeElasticForce(jello->kElastic, s.r, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],e);
        computeDamping(jello->dElastic, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],
                       jello->v[s.i1][s.j1][s.k1], jello->v[s.i2][s.j2][s.k2],d);

        // a = F / m
        pMULTIPLY(e, invM, e);
        pMULTIPLY(d, invM, d);

        // apply to the first particle
        pSUM(a[s.i1][s.j1][s.k1],e,a[s.i1][s.j1][s.k1]);
        pSUM(a[s.i1][s.j1][s.k1],d,a[s.i1][s.j1][s.k1]);

        // negate e and d
        pMULTIPLY(e,-1,e);
        pMULTIPLY(d,-1,d);

        // apply to the second particle
        pSUM(a[s.i2][s.j2][s.k2],e,a[s.i2][s.j2][s.k2]);
        pSUM(a[s.i2][s.j2][s.k2],d,a[s.i2][s.j2][s.k2]);
    }
}

/**
 * check if a point is inside the bounding box
 * @param p - point to be checked
 * @param b - bounding box
 * @return true if p inside b, otherwise false
 */
bool isPointInsideBBox(const point &p, bbox b)
{
    return ((p.x >= b.min.x && p.x <= b.max.x)
      && (p.y >= b.min.y && p.y <= b.max.y)
      && (p.z >= b.min.z && p.z <= b.max.z));
}

/**
 * check if a point is lie at the positive side of a plane
 * @param pt - point
 * @param pl - plane
 * @return true if point lie in the positive side, otherwise false
 */
bool isPointInPositiveSide(const point &pt, const plane &pl)
{
    return pl.a * pt.x + pl.b * pt.y + pl.c * pt.z + pl.d >= 0;
}

/**
 * compute the closest point from plane to a point
 * @param pt - point
 * @param pl - plane
 * @return closest point to point in plane
 */
point computeClosestPoint(const point &pt, plane pl)
{
    // get normal
    point n;
    n = point(pl.a, pl.b, pl.c);

    // create a ray with origin at pt, with -n direction and parameter t: r = pt - nt
    // compute t when ray intersect with plane ax + by + cz + d = 0
    // t = (<n,pt>+d)/<n,n>
    double dot0, dot1;
    DOTPRODUCTp(n, pt, dot0);
    DOTPRODUCTp(n, n, dot1);
    double t;
    t = (dot0 + pl.d) / dot1;

    // insert t back into ray equation we get intersection point : p1 = pt - tn
    pMULTIPLY(n, -t, n);
    point closestPos;
    pSUM(pt, n, closestPos);
    return closestPos;
}

/**
 * check if collision happen and record collision info
 * @param jello - jello state
 * @param points - indices of points have has collided
 * @param closestPos - closest positions on the boundary
 * @return true if the collision happen, otherwise false
 */
bool checkCollisions(const struct world * jello, std::vector<indices> &points, std::vector<point> &closestPos)
{
    for (int i=0; i<=7; i++)
        for (int j=0; j<=7; j++)
            for (int k=0; k<=7; k++)
            {
                point pt = jello->p[i][j][k];
                if (!isPointInsideBBox(pt,boundingBox))  // collide with bounding box
                {
                    for (int pInd = 0; pInd < 6; pInd++)
                    {
                        if (!isPointInPositiveSide(pt, boundingBox.planes[pInd])) {  // collide with any plane
                            points.push_back(indices(i, j, k));
                            closestPos.push_back(computeClosestPoint(pt, boundingBox.planes[pInd]));
                        }
                    }
                }
                plane pl = plane(jello->a, jello->b, jello->c, jello->d);
                if (jello->incPlanePresent == 1
                    && !isPointInPositiveSide(pt, pl))  // collide with inclined plane
                {
                    points.push_back(indices(i, j, k));
                    closestPos.push_back(computeClosestPoint(pt, pl));
                }
            }
    return points.size() != 0;
}
/**
 * collision response
 * @param points - indices of points have has collided
 * @param closestPos - closest positions on the boundary
 * @param collisionSprings - collision springs
 */
void collisionResponse(std::vector<indices> &points, std::vector<point> &closestPos, std::vector<collisionSpring> &collisionSprings)
{
    if (points.size() == 0) // no collision
        return;

    for (int i = 0; i < points.size(); i++) // create a spring for each collision
    {
        collisionSprings.push_back(collisionSpring(points[i], closestPos[i]));
    }
}

/**
 * compute acceleration for all collisions
 * @param jello - jello state
 * @param a - acceleration array
 * @param invM - inverse of mass
 */
void computeAccelerationForCollisions(const struct world * jello, struct point a[8][8][8], double invM)
{
    std::vector<indices> points;
    std::vector<point> closestPos;
    std::vector<collisionSpring> collisionSprings;

    if(checkCollisions(jello, points, closestPos))  // if collision happens
        collisionResponse(points, closestPos, collisionSprings);    // store current particle and create collision spring

    // compute acceleration for each collision
    for (const auto &s : collisionSprings)
    {
        point e,d;
        // compute elastic force and damping
        computeElasticForce(jello->kCollision, s.r, jello->p[s.pInd.ix][s.pInd.iy][s.pInd.iz], s.contactPoint,e);
        computeDamping(jello->dCollision, jello->p[s.pInd.ix][s.pInd.iy][s.pInd.iz], s.contactPoint,
                       jello->v[s.pInd.ix][s.pInd.iy][s.pInd.iz], point(0, 0, 0), d);

        // a = F / m
        pMULTIPLY(e, invM, e);
        pMULTIPLY(d, invM, d);

        // apply to the the particle that has collided
        pSUM(a[s.pInd.ix][s.pInd.iy][s.pInd.iz],e,a[s.pInd.ix][s.pInd.iy][s.pInd.iz]);
        pSUM(a[s.pInd.ix][s.pInd.iy][s.pInd.iz],d,a[s.pInd.ix][s.pInd.iy][s.pInd.iz]);
    }

}

/**
 * compute cell width for external force field
 * @param jello - jello state
 * @param box - bounding box
 * @return cell width of external force field
 */
point computeCellWidth(const struct world & jello, bbox &box)
{
    double invRes = 1.0 / (jello.resolution - 1);  // inverse of cell number in one axis
    return point((box.max.x - box.min.x) * invRes,  // cell length
                 (box.max.y - box.min.y) * invRes,  // cell width
                 (box.max.z - box.min.z) * invRes); // cell height
}

/**
 * compute the force field cell indices of a particle which it lies in
 * @param jello - jello state
 * @param p - current particle
 * @param box - bounding box
 * @return the force field cell indices of a particle which it lies in
 */
point computeCellIndex(const struct world & jello, const point &p, const bbox &box)
{
    #define GET_INDEX(axis) \
        floor((p.axis - box.min.axis) * (jello.resolution - 1) / (box.max.axis - box.min.axis) ) \

    // compute cell index in each dimension
    int i = GET_INDEX(x);
    int j = GET_INDEX(y);
    int k = GET_INDEX(z);

    // make sure the index won't get out of bound
    if (i == (jello.resolution - 1)) i--;
    if (j == (jello.resolution - 1)) j--;
    if (k == (jello.resolution - 1)) k--;

    return point(i, j, k);
}
/**
 * compute barycentric coordinate of the particle inside cell
 * @param p - current particle
 * @param cellIndex - force field cell indices
 * @param cellWidth - force field cell width
 * @param box - bounding box
 * @return barycentric coordinate of the particle inside cell
 */
point computeBarycentricCoord(const point &p, point &cellIndex, point &cellWidth, const bbox &box)
{
    #define GET_COORD(axis) \
        (box.min.axis + (box.max.axis - box.min.axis) * (1.0 * cellIndex.axis / (jello.resolution-1))) \

    return point((p.x - GET_COORD(x)) / cellWidth.x,
                 (p.y - GET_COORD(y)) / cellWidth.y,
                 (p.z - GET_COORD(z)) / cellWidth.z);
}
/**
 * compute particle's neighboring forces based on its location
 * @param jello - jello state
 * @param p - current particle
 * @param cellIndex - the force field cell indices that p lies in
 * @return p's neighboring forces
 */
std::vector<point> computeNeighborForces(const struct world & jello, const point &p, point &cellIndex)
{
    std::vector<point> forces;
    #define GET_IDX(I, J, K) \
        (cellIndex.x + (I)) * jello.resolution * jello.resolution + (cellIndex.y + (J)) * jello.resolution + (cellIndex.z + (K))\

    #define F(i, j, k) \
        { forces.push_back(jello.forceField[int(GET_IDX((i), (j), (k)))]);}  \

    // Add neighboring forces to the array and return
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++) F(i, j, k);

    return forces;
}

/**
 * force-field interpolation
 * @param bc
 * @param forces
 * @return interpolated force
 */
point interpolate(const point &bc, const std::vector<point> &forces)
{
    point f;
    for (int i = 0; i < 2; i++)
        for (int j = 0; j < 2; j++)
            for (int k = 0; k < 2; k++)
            {
                double a, b, c;
                a = (i==1) ? bc.x : (1-bc.x);
                b = (j==1) ? bc.y : (1-bc.y);
                c = (k==1) ? bc.z : (1-bc.z);
                point temp;
                pMULTIPLY(forces[4*i+2*j+k], a*b*c, temp);
                pSUM(f, temp, f);
            }
    return f;
}

point computeExternalForce(const struct world & jello, const point &p)
{

    // decide which cell the current particle is in
    point cellIndex = computeCellIndex(jello, p, boundingBox);
    // compute barycentric cord
    point bc = computeBarycentricCoord(p, cellIndex, cellWidth, boundingBox);
    // interpolate with neighbor forces
    return interpolate(bc, computeNeighborForces(jello, p, cellIndex));
}

/**
 * compute acceleration for external forces
 * @param jello - jello state
 * @param a - acceleration array
 * @param invM - inverse of mass
 */
void computeAccelerationForExternalForces(const struct world * jello, struct point a[8][8][8], double invM)
{
    if (jello->resolution < 2) return;

    // compute external force for each particle
    for (int i=0; i<=7; i++)
        for (int j=0; j<=7; j++)
            for (int k=0; k<=7; k++)
            {
                // skip if the particle is out of bouding box
                if (!isPointInsideBBox(jello->p[i][j][k], boundingBox))
                    continue;

                point f = computeExternalForce(*jello, jello->p[i][j][k]);
                pMULTIPLY(f, invM, f);  // a = F / m
                pSUM(a[i][j][k], f, a[i][j][k]);
            }
}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
    int i, j, k;

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++) { pMAKE(0, 0, 0, a[i][j][k]);}


    double invM = 1.0 / jello->mass;

    // springs forces and damping
    computeAccelerationForSprings(jello, a, structuralSprings, invM);
    computeAccelerationForSprings(jello, a, shearSprings, invM);
    computeAccelerationForSprings(jello, a, bendSprings, invM);

    // collision detection and response
    computeAccelerationForCollisions(jello, a, invM);

    // external force
    computeAccelerationForExternalForces(jello, a, invM);
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  current_time += jello->dt;
  
  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
        jello->p[i][j][k].x += jello->dt * jello->v[i][j][k].x;
        jello->p[i][j][k].y += jello->dt * jello->v[i][j][k].y;
        jello->p[i][j][k].z += jello->dt * jello->v[i][j][k].z;
        jello->v[i][j][k].x += jello->dt * a[i][j][k].x;
        jello->v[i][j][k].y += jello->dt * a[i][j][k].y;
        jello->v[i][j][k].z += jello->dt * a[i][j][k].z;

      }
}

/**
 * performs one step of MidPoint Integration
 * as a result, updates the jello structure
 * @param jello - jello state
 */
void MidPoint(struct world * jello)
{
    point F1p[8][8][8], F1v[8][8][8];

    point a[8][8][8];

    struct world buffer;

    int i,j,k;

    buffer = *jello; // make a copy of jello
    current_time += jello->dt;
    computeAcceleration(jello, a);    // f(x0, t0)

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
                pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);          // k1 = dt * f(x0, t0) -> F1p, F1v
                pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
                pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
                pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
                pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
            }

    computeAcceleration(&buffer, a);  // f(x0 + k1/2, t0 + dt/2)

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                // F2p = dt * buffer.v;
                pMULTIPLY(buffer.v[i][j][k],jello->dt,buffer.p[i][j][k]);
                // F2v = dt * a(buffer.p,buffer.v);
                pMULTIPLY(a[i][j][k],jello->dt,buffer.v[i][j][k]);            // k2 = dt * f(x0 + k1/2, t0 + dt/2) -> F2p, F2v
                pSUM(jello->p[i][j][k],buffer.p[i][j][k],jello->p[i][j][k]);  // x1 = x0 + k2.p -> jello.p
                pSUM(jello->v[i][j][k],buffer.v[i][j][k],jello->v[i][j][k]);  // v1 = v0 + k2.v -> jello.v
            }
}

/* performs one step of RK4 Integration */
/* as a result, updates the jello structure */
void RK4(struct world * jello)
{
  point F1p[8][8][8], F1v[8][8][8], 
        F2p[8][8][8], F2v[8][8][8],
        F3p[8][8][8], F3v[8][8][8],
        F4p[8][8][8], F4v[8][8][8];

  point a[8][8][8];


  struct world buffer;

  int i,j,k;

  buffer = *jello; // make a copy of jello
  current_time += jello->dt;
  computeAcceleration(jello, a);    // f(x0, t0)

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);          // k1 = dt * f(x0, t0) -> F1p, F1v
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);  // f(x0 + k1/2, t0 + dt/2)

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);          // k2 = dt * f(x0 + k1/2, t0 + dt/2) -> F2p, F2v
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);  // f(x0 + k2/2, t0 + dt/2)

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);          // k3 = dt * f(x0 + k2/2, t0 + dt/2) -> F3p, F3v
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);  // f(x0 + k3, t0 + dt)


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);          // k4 = dt * f(x0 + k3, t0 + dt) -> F4p, F4v

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);   // x1 = x0 + 1/6 * (k1 + 2k2 + 2k3 + k4) -> jello.p

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);    // v1 = v0 + 1/6 * (k1 + 2k2 + 2k3 + k4) -> jello.v
      }

  return;  
}