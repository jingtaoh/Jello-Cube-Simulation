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

    #define PROCESS_NEIGHBOUR(di,dj,dk,springType,scale) \
        ip=i+(di);\
        jp=j+(dj);\
        kp=k+(dk);\
        if\
        (!( (ip>7) || (ip<0) ||\
            (jp>7) || (jp<0) ||\
            (kp>7) || (kp<0) )) \
        {\
            (springType).push_back(spring(i, j, k, ip, jp, kp,scale)); \
        }\

    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                double scale;
                // structural springs
                {
                    scale =1;
                    PROCESS_NEIGHBOUR(1,0,0,structuralSprinps,scale);
                    PROCESS_NEIGHBOUR(0,1,0,structuralSprinps,scale);
                    PROCESS_NEIGHBOUR(0,0,1,structuralSprinps,scale);
                    PROCESS_NEIGHBOUR(-1,0,0,structuralSprinps,scale);
                    PROCESS_NEIGHBOUR(0,-1,0,structuralSprinps,scale);
                    PROCESS_NEIGHBOUR(0,0,-1,structuralSprinps,scale);
                }
                // shear springs
                {
                    scale = sqrt(2);
                    PROCESS_NEIGHBOUR(1,1,0,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,1,0,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,-1,0,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,-1,0,shearSprings,scale);
                    PROCESS_NEIGHBOUR(0,1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(0,-1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(0,-1,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(0,1,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,0,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,0,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,0,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,0,-1,shearSprings,scale);

                    scale = sqrt(3);
                    PROCESS_NEIGHBOUR(1,1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,-1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,-1,1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,1,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,1,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(-1,-1,-1,shearSprings,scale);
                    PROCESS_NEIGHBOUR(1,-1,-1,shearSprings,scale);
                }
                // bend springs
                {
                    scale = 2;
                    PROCESS_NEIGHBOUR(2,0,0,bendSprings,scale);
                    PROCESS_NEIGHBOUR(0,2,0,bendSprings,scale);
                    PROCESS_NEIGHBOUR(0,0,2,bendSprings,scale);
                    PROCESS_NEIGHBOUR(-2,0,0,bendSprings,scale);
                    PROCESS_NEIGHBOUR(0,-2,0,bendSprings,scale);
                    PROCESS_NEIGHBOUR(0,0,-2,bendSprings,scale);
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
    pMULTIPLY(l, - k * (length - r), e);
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

/* Computes acceleration to every control point of the jello cube, 
   which is in state given by 'jello'.
   Returns result in array 'a'. */
void computeAcceleration(struct world * jello, struct point a[8][8][8])
{
  /* for you to implement ... */
  // TODO: Implement a = F / m
  //    - bouncing off the walls

  double invM = 1 / jello->mass;
//  std::cout << "Mass: " << jello->mass << std::endl;
//  std::cout << "invM: " << invM << std::endl;

  // structural springs
  for (const auto &s : structuralSprinps)
  {
      point e,d;
      computeElasticForce(jello->kElastic, s.r, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],e);
      computeDamping(jello->dElastic, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],
                     jello->v[s.i1][s.j1][s.k1], jello->v[s.i2][s.j2][s.k2],d);
//      pPRINT(e);
//      pPRINT(d);
      pMULTIPLY(e, invM, e);
      pMULTIPLY(d, invM, d);
//      pPRINT(e);
//      pPRINT(d);
      pSUM(a[s.i1][s.j1][s.k1],e,a[s.i1][s.j1][s.k1]);
      pSUM(a[s.i1][s.j1][s.k1],d,a[s.i1][s.j1][s.k1]);
      // negate e and d
      pMULTIPLY(e,-1,e);
      pMULTIPLY(d,-1,d);
//      pPRINT(e);
//      pPRINT(d);
      pSUM(a[s.i2][s.j2][s.k2],e,a[s.i2][s.j2][s.k2]);
      pSUM(a[s.i2][s.j2][s.k2],d,a[s.i2][s.j2][s.k2]);
//      pPRINT(a[s.i1][s.j1][s.k1]);
//      pPRINT(a[s.i2][s.j2][s.k2]);
  }

  // shear springs
  for (const auto &s : shearSprings)
  {
      point e,d;
      computeElasticForce(jello->kElastic, s.r, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],e);
      computeDamping(jello->dElastic, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],
                     jello->v[s.i1][s.j1][s.k1], jello->v[s.i2][s.j2][s.k2],d);
//      pPRINT(e);
//      pPRINT(d);
      pMULTIPLY(e, invM, e);
      pMULTIPLY(d, invM, d);
//      pPRINT(e);
//      pPRINT(d);
      pSUM(a[s.i1][s.j1][s.k1],e,a[s.i1][s.j1][s.k1]);
      pSUM(a[s.i1][s.j1][s.k1],d,a[s.i1][s.j1][s.k1]);
      // negate e and d
      pMULTIPLY(e,-1,e);
      pMULTIPLY(d,-1,d);
//      pPRINT(e);
//      pPRINT(d);
      pSUM(a[s.i2][s.j2][s.k2],e,a[s.i2][s.j2][s.k2]);
      pSUM(a[s.i2][s.j2][s.k2],d,a[s.i2][s.j2][s.k2]);
//      pPRINT(a[s.i1][s.j1][s.k1]);
//      pPRINT(a[s.i2][s.j2][s.k2]);
  }

    // bend springs
    for (const auto &s : bendSprings)
    {
        point e,d;
        computeElasticForce(jello->kElastic, s.r, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],e);
        computeDamping(jello->dElastic, jello->p[s.i1][s.j1][s.k1], jello->p[s.i2][s.j2][s.k2],
                       jello->v[s.i1][s.j1][s.k1], jello->v[s.i2][s.j2][s.k2],d);
//        pPRINT(e);
//        pPRINT(d);
        pMULTIPLY(e, invM, e);
        pMULTIPLY(d, invM, d);
//        pPRINT(e);
//        pPRINT(d);
        pSUM(a[s.i1][s.j1][s.k1],e,a[s.i1][s.j1][s.k1]);
        pSUM(a[s.i1][s.j1][s.k1],d,a[s.i1][s.j1][s.k1]);
        // negate e and d
        pMULTIPLY(e,-1,e);
        pMULTIPLY(d,-1,d);
//        pPRINT(e);
//        pPRINT(d);
        pSUM(a[s.i2][s.j2][s.k2],e,a[s.i2][s.j2][s.k2]);
        pSUM(a[s.i2][s.j2][s.k2],d,a[s.i2][s.j2][s.k2]);
//        pPRINT(a[s.i1][s.j1][s.k1]);
//        pPRINT(a[s.i2][s.j2][s.k2]);
    }

    // external force
    int i, j, k;
    for (i=0; i<=7; i++)
        for (j=0; j<=7; j++)
            for (k=0; k<=7; k++)
            {
                point f;
                pCPY(jello->forceField[i * jello->resolution * jello->resolution + j * jello->resolution + k],f);
//                pPRINT(f);
                pMULTIPLY(f, invM, f);
//                pPRINT(f);
                pSUM(a[i][j][k],f, a[i][j][k]);
//                pPRINT(a[i][j][k]);
            }
}

/* performs one step of Euler Integration */
/* as a result, updates the jello structure */
void Euler(struct world * jello)
{
  int i,j,k;
  point a[8][8][8];

  computeAcceleration(jello, a);
  
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
