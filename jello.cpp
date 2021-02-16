/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

  Your name:
  <Jingtao Huang>

*/

#include "jello.h"
#include "showCube.h"
#include "input.h"
#include "physics.h"

// camera parameters
double Theta = pi / 6;
double Phi = pi / 6;
double R = 6;

// mouse control
int g_iMenuId;
int g_vMousePos[2];
int g_iLeftMouseButton,g_iMiddleMouseButton,g_iRightMouseButton;

// number of images saved to disk so far
int sprite=0;

// these variables control what is displayed on screen
int shear=0, bend=0, structural=1, pause=0, viewingMode=0, saveScreenToFile=0;

struct world jello;

int windowWidth, windowHeight;

void myinit()
{
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90.0,1.0,0.01,1000.0);

  // set background color to grey
  glClearColor(0.5, 0.5, 0.5, 0.0);

  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);

  glShadeModel(GL_SMOOTH);
  glEnable(GL_POLYGON_SMOOTH);
  glEnable(GL_LINE_SMOOTH);

  return; 
}

void reshape(int w, int h)
{
  // Prevent a divide by zero, when h is zero.
  // You can't make a window of zero height.
  if(h == 0)
    h = 1;

  glViewport(0, 0, w, h);

  // Reset the coordinate system before modifying
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // Set the perspective
  double aspectRatio = 1.0 * w / h;
  gluPerspective(60.0f, aspectRatio, 0.01f, 1000.0f);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity(); 

  windowWidth = w;
  windowHeight = h;

  glutPostRedisplay();
}

void display()
{
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // camera parameters are Phi, Theta, R
  gluLookAt(R * cos(Phi) * cos (Theta), R * sin(Phi) * cos (Theta), R * sin (Theta),
	        0.0,0.0,0.0, 0.0,0.0,1.0);


  /* Lighting */
  /* You are encouraged to change lighting parameters or make improvements/modifications
     to the lighting model . 
     This way, you will personalize your assignment and your assignment will stick out. 
  */

  // global ambient light
  GLfloat aGa[] = { 0.0, 0.0, 0.0, 0.0 };

  // TODO: improve lighting environment
  // light 's ambient, diffuse, specular
  GLfloat lKa0[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd0[] = { 1.0, 1.0, 1.0, 1.0 };  // white
  GLfloat lKs0[] = { 1.0, 1.0, 1.0, 1.0 };  // white

  GLfloat lKa1[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd1[] = { 1.0, 0.0, 0.0, 1.0 };  // red
  GLfloat lKs1[] = { 1.0, 0.0, 0.0, 1.0 };  // red

  GLfloat lKa2[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd2[] = { 1.0, 1.0, 0.0, 1.0 };  // yellow
  GLfloat lKs2[] = { 1.0, 1.0, 0.0, 1.0 };  // yellow

  GLfloat lKa3[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd3[] = { 0.0, 1.0, 1.0, 1.0 };  // cyan
  GLfloat lKs3[] = { 0.0, 1.0, 1.0, 1.0 };  // cyan

  GLfloat lKa4[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd4[] = { 0.0, 0.0, 1.0, 1.0 };  // blue
  GLfloat lKs4[] = { 0.0, 0.0, 1.0, 1.0 };  // blue

  GLfloat lKa5[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd5[] = { 1.0, 0.0, 1.0, 1.0 };  // purple
  GLfloat lKs5[] = { 1.0, 0.0, 1.0, 1.0 };  // purple

  GLfloat lKa6[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd6[] = { 1.0, 1.0, 1.0, 1.0 };  // white
  GLfloat lKs6[] = { 1.0, 1.0, 1.0, 1.0 };  // white

  GLfloat lKa7[] = { 0.0, 0.0, 0.0, 1.0 };  // black
  GLfloat lKd7[] = { 0.0, 1.0, 1.0, 1.0 };  // cyan
  GLfloat lKs7[] = { 0.0, 1.0, 1.0, 1.0 };  // cyan

  // light positions and directions
  /*        Z
   *    4-------7
   *   /|      /|
   *  5-|-----6 |  Y
   *  | 0-----|-3
   *  |/      |/
   *  1-------2
   *      X
   */
  GLfloat lP0[] = { -1.999, -1.999, -1.999, 1.0 };
  GLfloat lP1[] = { 1.999, -1.999, -1.999, 1.0 };
  GLfloat lP2[] = { 1.999, 1.999, -1.999, 1.0 };
  GLfloat lP3[] = { -1.999, 1.999, -1.999, 1.0 };
  GLfloat lP4[] = { -1.999, -1.999, 1.999, 1.0 };
  GLfloat lP5[] = { 1.999, -1.999, 1.999, 1.0 };
  GLfloat lP6[] = { 1.999, 1.999, 1.999, 1.0 };
  GLfloat lP7[] = { -1.999, 1.999, 1.999, 1.0 };
  
  // jelly material color

  GLfloat mKa[] = { 0.0, 0.0, 0.0, 1.0 };   // black
  GLfloat mKd[] = { 0.3, 0.3, 0.3, 1.0 };   // grey
  GLfloat mKs[] = { 1.0, 1.0, 1.0, 1.0 };   // white
  GLfloat mKe[] = { 0.0, 0.0, 0.0, 1.0 };

  /* set up lighting */
  glLightModelfv(GL_LIGHT_MODEL_AMBIENT, aGa);
  glLightModelf(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);
  glLightModelf(GL_LIGHT_MODEL_TWO_SIDE, GL_FALSE);

  // set up cube color
  glMaterialfv(GL_FRONT, GL_AMBIENT, mKa);
  glMaterialfv(GL_FRONT, GL_DIFFUSE, mKd);
  glMaterialfv(GL_FRONT, GL_SPECULAR, mKs);
  glMaterialfv(GL_FRONT, GL_EMISSION, mKe);
  glMaterialf(GL_FRONT, GL_SHININESS, 120);
    
  // macro to set up light i
  #define LIGHTSETUP(i)\
  glLightfv(GL_LIGHT##i, GL_POSITION, lP##i);\
  glLightfv(GL_LIGHT##i, GL_AMBIENT, lKa##i);\
  glLightfv(GL_LIGHT##i, GL_DIFFUSE, lKd##i);\
  glLightfv(GL_LIGHT##i, GL_SPECULAR, lKs##i);\
  glEnable(GL_LIGHT##i)
  
  LIGHTSETUP (0);
  LIGHTSETUP (1);
  LIGHTSETUP (2);
  LIGHTSETUP (3);
  LIGHTSETUP (4);
  LIGHTSETUP (5);
  LIGHTSETUP (6);
  LIGHTSETUP (7);

  // enable lighting
  glEnable(GL_LIGHTING);    
  glEnable(GL_DEPTH_TEST);

  // show the cube
  showCube(&jello);

  glDisable(GL_LIGHTING);

  // show the bounding box
  showBoundingBox();
 
  glutSwapBuffers();
}

void doIdle()
{
  char s[20]="picxxxx.ppm";
  int i;
  
  // save screen to file
  s[3] = 48 + (sprite / 1000);
  s[4] = 48 + (sprite % 1000) / 100;
  s[5] = 48 + (sprite % 100 ) / 10;
  s[6] = 48 + sprite % 10;

  if (saveScreenToFile==1)
  {
    saveScreenshot(windowWidth, windowHeight, s);
    saveScreenToFile=0; // save only once, change this if you want continuos image generation (i.e. animation)
    sprite++;
  }

  if (sprite >= 300) // allow only 300 snapshots
  {
    exit(0);	
  }

  if (pause == 0)
  {
    // insert code which appropriately performs one step of the cube simulation:
    MoveDown(&jello);
  }

  glutPostRedisplay();
}

int main (int argc, char ** argv)
{
  if (argc<2)
  {  
    printf ("Oops! You didn't say the jello world file!\n");
    printf ("Usage: %s [worldfile]\n", argv[0]);
    exit(0);
  }

  // test dotproduct
  point v1, v2;
  double dotProduct1, dotProduct2;
  v1.x = -1; v1.y = 1; v1.z = 0;
  v2.x = 1; v2.y = 1; v2.z = 1;
  DOTPRODUCT(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, dotProduct1);
  DOTPRODUCTp(v1, v2, dotProduct2);
  std::cout << "dotProduct1: " << dotProduct1 << std::endl;
  std::cout << "dotProduct2: " << dotProduct2 << std::endl;

  // test Elastic Force
  double k_hook = 2.0, r = 0.5;
  point p1, p2;
  p1.x = 0; p1.y = 0; p1.z = 0;
  p2.x = 0.4; p2.y = 0; p2.z = 0;
  point e;
  computeElasticForce(k_hook, r, p1, p2, e);
  pPRINT(e);

  // test Damping
  double k_d = 1.0;
  p1.x = 0; p1.y = 0; p1.z = 0;
  p2.x = 2; p2.y = 0; p2.z = 0;
  v1.x = 1; v1.y = 0; v1.z = 0;
  v2.x = 0; v2.y = 0; v2.z = 0;
  point d;
  computeDamping(k_d, p1, p2, v1, v2, d);
  pPRINT(d);


  readWorld(argv[1],&jello);

  glutInit(&argc,argv);
  
  /* double buffered window, use depth testing, 640x480 */
  glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
  
  windowWidth = 1280 /*640*/;
  windowHeight = 960 /*480*/;
  glutInitWindowSize (windowWidth, windowHeight);
  glutInitWindowPosition (0,0);
  glutCreateWindow ("Jello cube");

  /* tells glut to use a particular display function to redraw */
  glutDisplayFunc(display);

  /* replace with any animate code */
  glutIdleFunc(doIdle);

  /* callback for mouse drags */
  glutMotionFunc(mouseMotionDrag);

  /* callback for window size changes */
  glutReshapeFunc(reshape);

  /* callback for mouse movement */
  glutPassiveMotionFunc(mouseMotion);

  /* callback for mouse button changes */
  glutMouseFunc(mouseButton);

  /* register for keyboard events */
  glutKeyboardFunc(keyboardFunc);

  /* do initialization */
  myinit();

  /* forever sink in the black hole */
  glutMainLoop();

  return(0);
}

