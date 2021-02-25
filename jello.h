/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#ifndef _JELLO_H_
#define _JELLO_H_

#include "openGL-headers.h"
#include "pic.h"

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <limits>

#define pi 3.141592653589793238462643383279
#define L (1.0/7.0)

// camera angles
extern double Theta;
extern double Phi;
extern double R;

// number of images saved to disk so far
extern int sprite;

// mouse control
extern int g_vMousePos[2];
extern int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

struct point 
{
   double x;
   double y;
   double z;
   point() :x(0), y(0), z(0){}
   point(double a, double b, double c) : x(a), y(b), z(c) {};
};

struct indices
{
    int ix;
    int iy;
    int iz;
    indices(int i, int j, int k) : ix(i), iy(j), iz(k){};
};

// these variables control what is displayed on the screen
extern int shear, bend, structural, pause, viewingMode, saveScreenToFile;

struct world
{
  char integrator[10]; // "RK4" or "Euler"
  double dt; // timestep, e.g.. 0.001
  int n; // display only every nth timepoint
  double kElastic; // Hook's elasticity coefficient for all springs except collision springs
  double dElastic; // Damping coefficient for all springs except collision springs
  double kCollision; // Hook's elasticity coefficient for collision springs
  double dCollision; // Damping coefficient collision springs
  double mass; // mass of each of the 512 control points, mass assumed to be equal for every control point
  int incPlanePresent; // Is the inclined plane present? 1 = YES, 0 = NO (always NO in this assignment)
  double a,b,c,d; // inclined plane has equation a * x + b * y + c * z + d = 0; if no inclined plane, these four fields are not used
  int resolution; // resolution for the 3d grid specifying the external force field; value of 0 means that there is no force field
  struct point * forceField; // pointer to the array of values of the force field
  struct point p[8][8][8]; // position of the 512 control points
  struct point v[8][8][8]; // velocities of the 512 control points
};

extern struct world jello;

// computes dotproduct of three vectors, which are given as points
// struct point vector1, vector2, dest
// result goes into dest
#define DOTPRODUCTp(vector1,vector2,val) \
    DOTPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (val) )

// computes dotproduct of three vectors, which are specified by floating-point coordinates
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define DOTPRODUCT(x1,y1,z1,x2,y2,z2,value) \
    value = ((x1) * (x2)) + ((y1) * (y2)) + ((z1) * (z2))

// computes crossproduct of three vectors, which are given as points
// struct point vector1, vector2, dest
// result goes into dest
#define CROSSPRODUCTp(vector1,vector2,dest)\
  CROSSPRODUCT( (vector1).x, (vector1).y, (vector1).z,\
                (vector2).x, (vector2).y, (vector2).z,\
                (dest).x, (dest).y, (dest).z )

// computes crossproduct of three vectors, which are specified by floating-point coordinates
// double x1,y1,z1,x2,y2,z2,x,y,z
// result goes into x,y,z
#define CROSSPRODUCT(x1,y1,z1,x2,y2,z2,x,y,z)\
\
  x = (y1) * (z2) - (y2) * (z1);\
  y = (x2) * (z1) - (x1) * (z2);\
  z = (x1) * (y2) - (x2) * (y1)

// normalizes vector dest
// struct point dest
// result returned in dest
// must declare a double variable called 'length' somewhere inside the scope of the NORMALIZE macrp
// macro will change that variable
#define pNORMALIZE(dest)\
\
  length = sqrt((dest).x * (dest).x + (dest).y * (dest).y + (dest).z * (dest).z);\
  (dest).x /= length;\
  (dest).y /= length;\
  (dest).z /= length;

// copies vector source to vector dest
// struct point source,dest
#define pCPY(source,dest)\
\
  (dest).x = (source).x;\
  (dest).y = (source).y;\
  (dest).z = (source).z;
  
// assigns values x,y,z to point vector dest
// struct point dest
// double x,y,z
#define pMAKE(a,b,c,dest)\
\
  (dest).x = (a);\
  (dest).y = (b);\
  (dest).z = (c);

// sums points src1 and src2 to dest
// struct point src1,src2,dest
#define pSUM(src1,src2,dest)\
\
  (dest).x = (src1).x + (src2).x;\
  (dest).y = (src1).y + (src2).y;\
  (dest).z = (src1).z + (src2).z;

// dest = src2 - src1
// struct point src1,src2,dest
#define pDIFFERENCE(src1,src2,dest)\
\
  (dest).x = (src1).x - (src2).x;\
  (dest).y = (src1).y - (src2).y;\
  (dest).z = (src1).z - (src2).z;

// mulitplies components of point src by scalar and returns the result in dest
// struct point src,dest
// double scalar
#define pMULTIPLY(src,scalar,dest)\
\
  (dest).x = (src).x * (scalar);\
  (dest).y = (src).y * (scalar);\
  (dest).z = (src).z * (scalar);

// output point in the following format
// point: (point.x, point.y, point.z)
#define pPRINT(point) \
    std::cout << (#point) << ": (" << (point).x << ", " << (point).y << ", " << (point).z << ")" << std::endl;

struct spring
{
    int i1; int j1; int k1;  // indices of 1st mass point
    int i2; int j2; int k2;  // indices of 2nd mass point
    double r;   // rest length
    spring(int i, int j, int k, int ip, int jp, int kp, double scale)
            : i1(i), j1(j), k1(k), i2(ip), j2(jp), k2(kp)
    {
        r = scale * L;
    }
};

struct collisionSpring
{
    indices pInd;       // indices of mass point that collide with plane
    point contactPoint; // contact point
    double r;           // rest length
    collisionSpring(indices pI, point cP) : pInd(pI.ix, pI.iy, pI.iz), contactPoint(cP.x, cP.y, cP.z), r(0.0){}
};

struct plane
{
    double a, b, c, d;
    plane(double pa, double pb, double pc, double pd) : a(pa), b(pb), c(pc), d(pd) {};
    plane() : a(0), b(0), c(0), d(0){};
    plane(point p1, point p2, point p3)
    {
        point v1, v2, n;
        pDIFFERENCE(p3, p2, v1);
        pDIFFERENCE(p1, p2, v2);
        CROSSPRODUCTp(v1, v2, n);
        double length;
        pNORMALIZE(n);
        a = n.x; b = n.y; c  = n.z;
        d = - (a * p1.x + b * p1.y + c * p1.z);
//        print();
    }
    void print()
    {
        std::cout << "plane: " << a << "x + " << b << "y + " << c << "z + " << d << " = 0" << std::endl;
    }
};

struct ray
{
    point origin;
    point dir;
    ray(point p1, point p2) : origin(p1){
        pDIFFERENCE(p2, p1, dir);
        double length;
        pNORMALIZE(dir);
    }
    ray() : origin(point()), dir(point()){}
};

struct bbox
{
    point min;
    point max;
    plane planes[6];
    ray rays[12];
    bbox(point pMin, point pMax) : min(pMin), max(pMax){
        planes[0] = plane(point(min.x, min.y, min.z), point(max.x, min.y, min.z), point(max.x, max.y, min.z));  // bottom
        planes[1] = plane(point(max.x, max.y, max.z), point(max.x, min.y, max.z), point(min.x, min.y, max.z));  // top
        planes[2] = plane(point(min.x, min.y, min.z), point(min.x, max.y, min.z), point(min.x, max.y, max.z));  // left
        planes[3] = plane(point(max.x, max.y, max.z), point(max.x, max.y, min.z), point(max.x, min.y, min.z));  // right
        planes[4] = plane(point(min.x, min.y, min.z), point(min.x, min.y, max.z), point(max.x, min.y, max.z));  // front
        planes[5] = plane(point(max.x, max.y, max.z), point(min.x, max.y, max.z), point(min.x, max.y, min.z));  // back


        // create 8 vertices
        std::vector<point> vertices;
        vertices.push_back(min);                        // 000
        vertices.push_back(point(min.x, min.y, max.z)); // 001
        vertices.push_back(point(min.x, max.y, min.z)); // 010
        vertices.push_back(point(min.x, max.y, max.z)); // 011
        vertices.push_back(point(max.x, min.y, min.z)); // 100
        vertices.push_back(point(max.x, min.y, max.z)); // 101
        vertices.push_back(point(max.x, max.y, min.z)); // 110
        vertices.push_back(max);                        // 111


        /*        Z
         *    1-------5
         *   /|      /|
         *  3-|-----7 |  Y
         *  | 0-----|-4
         *  |/      |/
         *  2-------6
         *      X
         */

        // create 12 edges (store index of vertex)
        std::vector<std::vector<int>> edgeMap
                {
                        {0, 4}, {1, 5}, {2, 6}, {3, 7}, // in x direction
                        {0, 2}, {1, 3}, {4, 6}, {5, 7}, // in y direction
                        {0, 1}, {2, 3}, {4, 5}, {6, 7}  // in z direction

//                        {0, 1}, {0, 2}, {0,4},
//                        {1, 3}, {1, 5},
//                        {2, 3}, {2, 6},
//                        {3, 7},
//                        {4, 5}, {4, 6},
//                        {5, 7},
//                        {6,7}
                };

        for (int i = 0; i < 12; i++)
        {
            rays[i] = ray(vertices[edgeMap[i][0]], vertices[edgeMap[i][1]]);
        }
    };
public:
    void print()
    {
        for (int i = 0; i < 6; i++)
        {
            planes[i].print();
        }
        for (int i = 0; i < 12; i++)
        {
            pPRINT(rays[i].origin);
            pPRINT(rays[i].dir);
        }
    }
};

extern bbox boundingBox;
extern point cellWidth;

extern std::vector<spring> structuralSprings, shearSprings, bendSprings;

extern double current_time;
extern bool stop;
extern bool debug;

#endif

