/*

  USC/Viterbi/Computer Science
  "Jello Cube" Assignment 1 starter code

*/


#ifndef _SHOWCUBE_H_
#define _SHOWCUBE_H_

void showCube(struct world * jello);

void showBoundingBox(const bbox &box);

double intersect(const ray &r, const plane &pl);
// TODO : sort polygon index
void showInclinedPlane(const struct world & jello, const bbox &box);
void showAxis();
void showForceField(const struct world & jello, const bbox &box);

void showText(int winW, int winH);

#endif
