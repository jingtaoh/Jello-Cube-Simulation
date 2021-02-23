/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "physics.h"

/**
 * generate 3 kinds of springs from the world configuration
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
    // TODO : fix the error
    for (const auto &s : springs)
    {
        point e,d;
        computeElasticForce(jello->kElastic, s.r, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],e);
        computeDamping(jello->dElastic, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],
                       jello->v[s.i1][s.j1][s.k1], jello->v[s.i2][s.j2][s.k2],d);

        pMULTIPLY(e, invM, e);
        pMULTIPLY(d, invM, d);

        pSUM(a[s.i1][s.j1][s.k1],e,a[s.i1][s.j1][s.k1]);
        pSUM(a[s.i1][s.j1][s.k1],d,a[s.i1][s.j1][s.k1]);

        // negate e and d
        pMULTIPLY(e,-1,e);
        pMULTIPLY(d,-1,d);

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
point computeClosestPoint(const point &pt, const plane &pl)
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
 *
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
                if (!isPointInsideBBox(jello->p[i][j][k],boundingBox))  // collide with bounding box
                {
                    point p = jello->p[i][j][k];
                    for (int pInd = 0; pInd < 6; pInd++)
                    {
                        if (!isPointInPositiveSide(p, boundingBox.planes[pInd])) {  // collide with any plane
                            points.push_back(indices(i, j, k));
                            closestPos.push_back(computeClosestPoint(p, boundingBox.planes[pInd]));
                        }
                    }
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
    if (points.size() == 0) // no collision to process
        return;

    for (int i = 0; i < points.size(); i++)
    {
        collisionSprings.push_back(collisionSpring(points[i], closestPos[i]));
    }
}


void computeAccelerationForCollisions(const struct world * jello, struct point a[8][8][8], double invM)
{
    std::vector<indices> points;
    std::vector<point> closestPos;
    std::vector<collisionSpring> collisionSprings;

    if(checkCollisions(jello, points, closestPos))
        collisionResponse(points, closestPos, collisionSprings);

    // TODO : fix the error
    for (const auto &s : collisionSprings)
    {
        point e,d;
        computeElasticForce(jello->kCollision, s.r, jello->p[s.pInd.ix][s.pInd.iy][s.pInd.iz], s.contactPoint,e);
        computeDamping(jello->dCollision, jello->p[s.pInd.ix][s.pInd.iy][s.pInd.iz], s.contactPoint,
                       jello->v[s.pInd.ix][s.pInd.iy][s.pInd.iz], point(0, 0, 0), d);

        pMULTIPLY(e, invM, e);
        pMULTIPLY(d, invM, d);

        pSUM(a[s.pInd.ix][s.pInd.iy][s.pInd.iz],e,a[s.pInd.ix][s.pInd.iy][s.pInd.iz]);
        pSUM(a[s.pInd.ix][s.pInd.iy][s.pInd.iz],d,a[s.pInd.ix][s.pInd.iy][s.pInd.iz]);
    }

}

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /* for you to implement ... */
  // TODO: Implement a = F / m
  //    - external force

  int i, j, k;

  for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                pMAKE(0, 0, 0, a[i][j][k]);
            }


  double invM = 1.0 / jello->mass;

    // structural springs
    computeAccelerationForSprings(jello, a, structuralSprings, invM);

    // shear springs
    computeAccelerationForSprings(jello, a, shearSprings, invM);

    // bend springs
    computeAccelerationForSprings(jello, a, bendSprings, invM);

    // collision detection and response
    computeAccelerationForCollisions(jello, a, invM);

//    // external force
//    int i, j, k;
//    for (i=0; i<=7; i++)
//        for (j=0; j<=7; j++)
//            for (k=0; k<=7; k++)
//            {
//                point f;
//                pCPY(jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k],f);
//                pPRINT(f);
//                pMULTIPLY(f, invM, f);
//                pPRINT(f);
//                pSUM(a[i][j][k],f, a[i][j][k]);
//                pPRINT(a[i][j][k]);
//            }
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
  computeAcceleration(jello, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         pMULTIPLY(jello->v[i][j][k],jello->dt,F1p[i][j][k]);
         pMULTIPLY(a[i][j][k],jello->dt,F1v[i][j][k]);
         pMULTIPLY(F1p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F1v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F2p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F2p[i][j][k]);
         // F2v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F2v[i][j][k]);
         pMULTIPLY(F2p[i][j][k],0.5,buffer.p[i][j][k]);
         pMULTIPLY(F2v[i][j][k],0.5,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }

  computeAcceleration(&buffer, a);

  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F3p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F3v[i][j][k]);
         pMULTIPLY(F3p[i][j][k],1.0,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],1.0,buffer.v[i][j][k]);
         pSUM(jello->p[i][j][k],buffer.p[i][j][k],buffer.p[i][j][k]);
         pSUM(jello->v[i][j][k],buffer.v[i][j][k],buffer.v[i][j][k]);
      }
         
  computeAcceleration(&buffer, a);


  for (i=0; i<=7; i++)
    for (j=0; j<=7; j++)
      for (k=0; k<=7; k++)
      {
         // F3p = dt * buffer.v;
         pMULTIPLY(buffer.v[i][j][k],jello->dt,F4p[i][j][k]);
         // F3v = dt * a(buffer.p,buffer.v);     
         pMULTIPLY(a[i][j][k],jello->dt,F4v[i][j][k]);

         pMULTIPLY(F2p[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3p[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1p[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4p[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->p[i][j][k],jello->p[i][j][k]);

         pMULTIPLY(F2v[i][j][k],2,buffer.p[i][j][k]);
         pMULTIPLY(F3v[i][j][k],2,buffer.v[i][j][k]);
         pSUM(buffer.p[i][j][k],buffer.v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F1v[i][j][k],buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],F4v[i][j][k],buffer.p[i][j][k]);
         pMULTIPLY(buffer.p[i][j][k],1.0 / 6,buffer.p[i][j][k]);
         pSUM(buffer.p[i][j][k],jello->v[i][j][k],jello->v[i][j][k]);
      }

  return;  
}

/* har code non-physical sequence movement */
/* as a result, updates the jello structure */
void MoveDown(struct world * jello)
{
    int i,j,k;
    double velocity = 1.0;

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                jello->p[i][j][k].z -= jello->dt * velocity;
            }
}
