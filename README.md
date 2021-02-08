# Assignment 1: Simulating a Jello Cube

---

- Full Name: Jingtao Huang
- USC ID: 2697735160
- [Assignment Page and Starter Code](http://barbic.usc.edu/cs520-s21/assign1/)
- Environment: macOS Big Sur 11.2, 4.2GHz Quad-Core Intel Core i7

---

## Building & Usage
```sh
# Compile
make

# Usage: <world file name>
./jello world/jello.w
./createWorld 
```

## Interaction
Key | Functionality
--- | ---
esc | exit application
 v  | switch wireframe/triangle mode
 s  | show/Hide structural springs
 h  | show/hide shear springs
 b  | show/hide bend springs
space| save save the current screen to a file, filename index increments automatically
 p  | pause/unpause
 z  | camera zoom in
 x  | camera zoom out
 e  | reset camera to default position and render mode to wireframe
right mouse | camera control

## Basic Features
[comment]: <> (<Description of what you have accomplished>)
![](https://trello-attachments.s3.amazonaws.com/5ecddb65d8c3a57013131003/6019f70f6a535a48ba386d95/bd6249fb7bc5927963795a0f1408cd8f/minion.png)

- [ ] Animate the movement of a jello cube based on a realistic physical model.
- [ ] Must incorporate a bounding box, including collision detection and response. 
- [ ] Must implement an arbitrary non-homogeneous time-independent force field, with appropriate force-field interpolation.
- [ ] Use the provided code to interactively position the camera, use proper lighting, and render the cube in wireframe and triangle mode. 
- [ ] Read the description of the world from a world file and simulate the cube according to this information. Your program should work with arbitrary world files. Your program, of course, need not work correctly for world files containing invalid data, bad integrator parameters (blow up effects and similar), bad elasticity, damping parameters and other physical parameters, or bad position or velocity parameters.
- [ ] Run at interactive frame rates (>15fps at 640x480)
- [ ] Be reasonably commented and written in an undercstandable manner
- [ ] Be submitted along with JPEG frames for the required animation.
- [ ] Be submitted along with a README file documenting your program's features and describing the approaches you took to each of the open-ended problems. This is especially important if you have done something spectacular for which you wish to receive extra credit!


## Extra Credit
[comment]: <> (<Also, explain any extra credit that you have implemented.>)
> Please note that the amount of extra credit awarded will not exceed 25% of this assignment's total value.
- [ ] Implement collision detection with some other interesting object, such as an inclined plane, sphere, a cone, a spiral, etc.
- [ ] Make the animation interactive. Allow the user to interactively (i.e. while the program is running) push the cube in a certain direction by dragging the mouse. The cube should move/deform accordingly. Apply the user force equally to all the simulation points of the cube, regardless of where on the screen mouse dragging occurred.
- [ ] Extra extra credit: Allow the user to push only a certain (surface) simulation point, by selecting that simulation point on the screen and then pulling on it by dragging the mouse. For this, you will need to make use of selection and feedback capabilities of OpenGL (see OpenGL Red Book, Chapter 13). Or, you can distribute the force, making it larger closer to the cube location where the user pulled with the mouse. You can make the force falloff radius into a parameter, potentially modifiable at runtime. This will allow you to control how localized deformations your system will produce (extremely localized if force only applies to one simulation point, or not localized at all if all simulation points always receive the same user force; and anything in between).



