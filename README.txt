<Please submit this file with your solution.>

CSCI 520, Assignment 1

Jingtao Huang
USC ID: 2697735160

================

# Core Credit
- The mass-spring system
    - Structural springs
    - Shear springs
    - Bend springs
- Dynamics
    - Elastic force based on Hook's law
    - Damping
    - Collision detection and response against bounding box
    - External force field with trilinear interpolation
- Integration
    - Euler
    - RK4

# Extra credit
- Collision detection with inclined plane
    - test in jello.w, rotate.w, gravity.w
- Mid point integrator
    - test by changing one of the world file's integrator to "MP"
    - all of the world files won't blow up, rotate.w can be a little bit unstable at first but it's better than Euler since Euler will below up
- FPS computation and display
    - use key 'd' to toggle on and off to show the text
- Rendering
    - Transparent cube using blending
    - Cornell box with boundary
    - Antialiasing
- Camera manipulation
    - use key 'r' to start/pause camera rotation
- Debug mode
    - use -d option (eg. `./jello world/jello -d`) to enter debug mode
    - the program will pause at the current time step
    - 3 axis will be drawn
    - use key 'n' to go to the next time step, current time step will be printed

# Submission Requirement
- Source code
    - current folder
- Mac OS X executable
    - current folder use eg. `./jello world/jello.w` to run
- Animation
    - Please find animation under "animation" folder
    - 300 jpeg images
    - anim.mp4
    - anim.gif

# Key mapping
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
 n  | when in debug mode, go to the next time step
 r  | rotate camera
 d  | display name, integrator and FPS

 # Comment
 Thank you for grading, hope you enjoy :)