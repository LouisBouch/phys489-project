# PHYS 489 project
## Dependencies
- [SFML3](https://www.sfml-dev.org/download/sfml/3.0.0/)(See how to install in the instructions below)
- [eigen](https://eigen.tuxfamily.org)(pacman -S eigen)
## Instructions to build the project (With linux)
- Clone the repo into a directory
- Create a sibling directory to the cloned repository named "sfml3"
- Add a directory inside "sfml3" called "build"
- Create an additional directory inside "build" called "release"
- Following this [tutorial](https://www.sfml-dev.org/tutorials/3.0/getting-started/build-from-source/#introduction), build the source code of sfml3 into the "release" directory.
- cd back into the cloned directory of this project
- cd to "build"
- Run 'cmake ..'
- Run 'make'
- Finally, run './SIS' to execute the binary



stuff::
1 thread for rendering, 1 thread for physics computation, 1 thread listening for window closing which then closes app.
RENDERING
window contains renderer
renderer contains simulation and a lambda expression which can be used to draw on the window
during each rendering loop inside the window, it calls renderer.render()
the renderer takes each polygon from the simulation and boundaries and draws them

after each render, the window adds to the time buffer 1/60 secs (if 60fps)

PHYSICS
physics engine contains simulation environment as well as reference to the window's time buffer
at each iteration, it "consumes" time from the window's time buffer to further the simulation
