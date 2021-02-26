/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/

#include "jello.h"
#include "showCube.h"

int pointMap(int side, int i, int j)
{
  int r;

  switch (side)
  {
  case 1: //[i][j][0] bottom face
    r = 64 * i + 8 * j;
    break;
  case 6: //[i][j][7] top face
    r = 64 * i + 8 * j + 7;
    break;
  case 2: //[i][0][j] front face
    r = 64 * i + j;
    break;
  case 5: //[i][7][j] back face
    r = 64 * i + 56 + j;
    break;
  case 3: //[0][i][j] left face
    r = 8 * i + j;
    break;
  case 4: //[7][i][j] right face
    r = 448 + 8 * i + j;
    break;
  }

  return r;
}

void showCube(struct world * jello)
{
  int i,j,k,ip,jp,kp;
  point r1,r2,r3; // aux variables
  
  /* normals buffer and counter for Gourad shading*/
  struct point normal[8][8];
  int counter[8][8];

  int face;
  double faceFactor, length;

  if (fabs(jello->p[0][0][0].x) > 10)
  {
    printf ("Your cube somehow escaped way out of the box.\n");
    exit(0);
  }

  
  #define NODE(face,i,j) (*((struct point * )(jello->p) + pointMap((face),(i),(j))))

  
  #define PROCESS_NEIGHBOUR(di,dj,dk) \
    ip=i+(di);\
    jp=j+(dj);\
    kp=k+(dk);\
    if\
    (!( (ip>7) || (ip<0) ||\
      (jp>7) || (jp<0) ||\
    (kp>7) || (kp<0) ) && ((i==0) || (i==7) || (j==0) || (j==7) || (k==0) || (k==7))\
       && ((ip==0) || (ip==7) || (jp==0) || (jp==7) || (kp==0) || (kp==7))) \
    {\
      glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);\
      glVertex3f(jello->p[ip][jp][kp].x,jello->p[ip][jp][kp].y,jello->p[ip][jp][kp].z);\
    }\

 
  if (viewingMode==0) // render wireframe
  {
    glLineWidth(1);
    glPointSize(5);
    glDisable(GL_LIGHTING);
    for (i=0; i<=7; i++)
      for (j=0; j<=7; j++)
        for (k=0; k<=7; k++)
        {
          if (i*j*k*(7-i)*(7-j)*(7-k) != 0) // not surface point
            continue;

          glBegin(GL_POINTS); // draw point
            glColor4f(0,0,0,0);  
            glVertex3f(jello->p[i][j][k].x,jello->p[i][j][k].y,jello->p[i][j][k].z);        
          glEnd();

          //
          //if ((i!=7) || (j!=7) || (k!=7))
          //  continue;

          glBegin(GL_LINES);      
          // structural
          if (structural == 1)
          {
            glColor4f(0,0,1,1);
            PROCESS_NEIGHBOUR(1,0,0);
            PROCESS_NEIGHBOUR(0,1,0);
            PROCESS_NEIGHBOUR(0,0,1);
//            PROCESS_NEIGHBOUR(-1,0,0);
//            PROCESS_NEIGHBOUR(0,-1,0);
//            PROCESS_NEIGHBOUR(0,0,-1);
          }
          
          // shear
          if (shear == 1)
          {
            glColor4f(0,1,0,1);
            PROCESS_NEIGHBOUR(1,1,0);
            PROCESS_NEIGHBOUR(-1,1,0);
//            PROCESS_NEIGHBOUR(-1,-1,0);
//            PROCESS_NEIGHBOUR(1,-1,0);
            PROCESS_NEIGHBOUR(0,1,1);
            PROCESS_NEIGHBOUR(0,-1,1);
//            PROCESS_NEIGHBOUR(0,-1,-1);
//            PROCESS_NEIGHBOUR(0,1,-1);
            PROCESS_NEIGHBOUR(1,0,1);
            PROCESS_NEIGHBOUR(-1,0,1);
//            PROCESS_NEIGHBOUR(-1,0,-1);
//            PROCESS_NEIGHBOUR(1,0,-1);

            PROCESS_NEIGHBOUR(1,1,1)
            PROCESS_NEIGHBOUR(-1,1,1)
            PROCESS_NEIGHBOUR(-1,-1,1)
            PROCESS_NEIGHBOUR(1,-1,1)
//            PROCESS_NEIGHBOUR(1,1,-1)
//            PROCESS_NEIGHBOUR(-1,1,-1)
//            PROCESS_NEIGHBOUR(-1,-1,-1)
//            PROCESS_NEIGHBOUR(1,-1,-1)
          }
          
          // bend
          if (bend == 1)
          {
            glColor4f(1,0,0,1);
            PROCESS_NEIGHBOUR(2,0,0);
            PROCESS_NEIGHBOUR(0,2,0);
            PROCESS_NEIGHBOUR(0,0,2);
//            PROCESS_NEIGHBOUR(-2,0,0);
//            PROCESS_NEIGHBOUR(0,-2,0);
//            PROCESS_NEIGHBOUR(0,0,-2);
          }           
          glEnd();
        }
    glEnable(GL_LIGHTING);
  }
  
  else
  {
    glPolygonMode(GL_FRONT, GL_FILL); 
    
    for (face=1; face <= 6; face++) 
      // face == face of a cube
      // 1 = bottom, 2 = front, 3 = left, 4 = right, 5 = far, 6 = top
    {
      
      if ((face==1) || (face==3) || (face==5))
        faceFactor=-1; // flip orientation
      else
        faceFactor=1;
      

      for (i=0; i <= 7; i++) // reset buffers
        for (j=0; j <= 7; j++)
        {
          normal[i][j].x=0;normal[i][j].y=0;normal[i][j].z=0;
          counter[i][j]=0;
        }

      /* process triangles, accumulate normals for Gourad shading */
  
      for (i=0; i <= 6; i++)
        for (j=0; j <= 6; j++) // process block (i,j)
        {
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i,j),r1); // first triangle
          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i,j),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i][j],r3,normal[i][j]);
          counter[i][j]++;

          pDIFFERENCE(NODE(face,i,j+1),NODE(face,i+1,j+1),r1); // second triangle
          pDIFFERENCE(NODE(face,i+1,j),NODE(face,i+1,j+1),r2);
          CROSSPRODUCTp(r1,r2,r3); pMULTIPLY(r3,faceFactor,r3);
          pNORMALIZE(r3);
          pSUM(normal[i+1][j],r3,normal[i+1][j]);
          counter[i+1][j]++;
          pSUM(normal[i][j+1],r3,normal[i][j+1]);
          counter[i][j+1]++;
          pSUM(normal[i+1][j+1],r3,normal[i+1][j+1]);
          counter[i+1][j+1]++;
        }

      
        /* the actual rendering */
        for (j=1; j<=7; j++) 
        {

          if (faceFactor  > 0)
            glFrontFace(GL_CCW); // the usual definition of front face
          else
            glFrontFace(GL_CW); // flip definition of orientation
         
          glBegin(GL_TRIANGLE_STRIP);
          for (i=0; i<=7; i++)
          {
            glNormal3f(normal[i][j].x / counter[i][j],normal[i][j].y / counter[i][j],
              normal[i][j].z / counter[i][j]);
            glVertex3f(NODE(face,i,j).x, NODE(face,i,j).y, NODE(face,i,j).z);
            glNormal3f(normal[i][j-1].x / counter[i][j-1],normal[i][j-1].y/ counter[i][j-1],
              normal[i][j-1].z / counter[i][j-1]);
            glVertex3f(NODE(face,i,j-1).x, NODE(face,i,j-1).y, NODE(face,i,j-1).z);
          }
          glEnd();
        }
        
        
    }  
  } // end for loop over faces
  glFrontFace(GL_CCW);
}

void showBoundingBox(const bbox &box)
{
  int i,j;

  glColor4f(0.6,0.6,0.6,0);

  glBegin(GL_LINES);

  // front face
  for(i=box.min.x; i<=box.max.x; i++)  // vertical lines
  {
    glVertex3f(i,box.min.y,box.min.z);
    glVertex3f(i,box.min.y,box.max.z);
  }
  for(j=box.min.z; j<=box.max.z; j++)  // horizontal lines
  {
    glVertex3f(box.min.x,box.min.y,j);
    glVertex3f(box.max.x,box.min.y,j);
  }

  // back face
  for(i=box.min.x; i<=box.max.x; i++)   // vertical lines
  {
    glVertex3f(i,box.max.y,box.min.z);
    glVertex3f(i,box.max.y,box.max.z);
  }
  for(j=box.min.z; j<=box.max.z; j++)   // horizontal lines
  {
    glVertex3f(box.min.x,box.max.y,j);
    glVertex3f(box.max.x,box.max.y,j);
  }

  // left face
  for(i=box.min.y; i<=box.max.y; i++)  // vertical lines
  {
    glVertex3f(box.min.x,i,box.min.z);
    glVertex3f(box.min.x,i,box.max.z);
  }
  for(j=box.min.z; j<=box.max.z; j++)   // horizontal lines
  {
    glVertex3f(box.min.x,box.min.y,j);
    glVertex3f(box.min.x,box.max.y,j);
  }

  // right face
  for(i=box.min.y; i<=box.max.y; i++)  // vertical lines
  {
    glVertex3f(box.max.x,i,box.min.z);
    glVertex3f(box.max.x,i,box.max.z);
  }
  for(j=box.min.z; j<=box.max.z; j++)   // horizontal lines
  {
    glVertex3f(box.max.x,box.min.y,j);
    glVertex3f(box.max.x,box.max.y,j);
  }
  
  glEnd();

  return;
}

/**
 * check test a ray against with a plane
 * @param r - ray r = o + td
 * @param pl - plane np + D = 0;
 * @return -1 if no intersection, >= 0 has intersection
 */
double intersect(const ray &r, const plane &pl)
{
    // substitute r with p in plane equation, we have t = - (<n, o> + D) / <n, d>

    // get normal
    point n;
    n = point(pl.a, pl.b, pl.c);

    double dot0, dot1;
    DOTPRODUCTp(n, r.origin, dot0);
    DOTPRODUCTp(n, r.dir, dot1);

    if (dot1 == 0) return -1;   // ray parallel with plane

    double t;
    t = - (dot0 + pl.d) / dot1;

    if (t < 0) return -1;       // plane is behind the ray

    return t;   // has intersection
}

/**
 * render the inclined plane specify in the world file
 * @param jello - world state
 * @param box - boudning box
 */
void showInclinedPlane(const struct world & jello, const bbox &box)
{
    if (jello.incPlanePresent == 0)  // return if inclined plane do not exist
        return;

    // get inclined plane from world
    plane pl = plane(jello.a, jello.b, jello.c, jello.d);

    #define INTERSECT(start, end, axis) \
        for (int i = (start); i <= (end); i++)    \
        {                               \
            double t = intersect(box.rays[i],pl); \
            if (t == -1 || (t > (box.max.axis - box.min.axis)))  \
                continue;               \
            point intersection; \
            pMULTIPLY(box.rays[i].dir, t, intersection);\
            pSUM(intersection, box.rays[i].origin, intersection); \
            intersections.push_back(intersection); \
        } \

    // compute intersections between 12 edges and plane
    std::vector<point> intersections;
    INTERSECT(0, 3, x);
    INTERSECT(4, 7, y);
    INTERSECT(8, 11, z);

    // render inclined plane

    glColor4f(1,0,0,0);

    glDisable(GL_CULL_FACE);

    glBegin(GL_TRIANGLE_STRIP);

    // triangle mode
    for (auto i : intersections) {
        glVertex3f(i.x, i.y, i.z);
    }
    // line mode
//    for (int i = 0; i < intersections.size() - 1; i++) {
//        glVertex3f(intersections[i].x, intersections[i].y, intersections[i].z);
//        glVertex3f(intersections[i + 1].x, intersections[i + 1].y, intersections[i + 1].z);
//    }
//    glVertex3f(intersections[intersections.size() - 1].x, intersections[intersections.size() - 1].y, intersections[intersections.size() - 1].z);
//    glVertex3f(intersections[0].x, intersections[0].y, intersections[0].z);

    glEnd();

    glEnable(GL_CULL_FACE);

    return;

}
/**
 * Draw x, y z axis in Red, Green and Blue
 */
void showAxis()
{

    glBegin(GL_LINES);

    glColor4f(1,0,0,0);

    glVertex3f(0, 0, 0);
    glVertex3f(3, 0, 0);

    glColor4f(0,1,0,0);

    glVertex3f(0, 0, 0);
    glVertex3f(0, 3, 0);

    glColor4f(0,0,1,0);

    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 3);

    glEnd();

    return;
}

/**
 * Draw Force Field
 * @param jello - jello state
 * @param box - bounding box
 */
void showForceField(const struct world & jello, const bbox &box)
{
    #define GET_COORD(axis, ind) \
        (box.min.axis + (box.max.axis - box.min.axis) * (1.0 * ind / (jello.resolution-1))) \

    #define GET_FORCE(axis) \
        abs(jello.forceField[(i) * jello.resolution * jello.resolution + (j) * jello.resolution + (k)].axis) / jello.mass \

    int i,j,k;
//    double max = std::numeric_limits<double>::min();
//    for (i=0; i<= jello.resolution-1; i++)
//        for (j=0; j<= jello.resolution-1; j++)
//            for (k=0; k<= jello.resolution-1; k++)
//            {
//                if (jello.forceField[(i) * jello.resolution * jello.resolution + (j) * jello.resolution + (k)].x > max)
//                    max = jello.forceField[(i) * jello.resolution * jello.resolution + (j) * jello.resolution + (k)].x;
//            }

    glBegin(GL_POINTS);

    for (i=0; i<= 29; i++)
        for (j=0; j<= 29; j++)
            for (k=14; k<= 15; k++)
            {
                glColor4f(GET_FORCE(x), GET_FORCE(y), GET_FORCE(z),0);
                glVertex3f(GET_COORD(x, i), GET_COORD(y, j), GET_COORD(z, k));
            }

    glEnd();
}