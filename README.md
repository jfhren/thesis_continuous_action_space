thesis_continuous_action_space
==============================

Code used for continuous action space experiments in my PhD thesis.

## Dependencies

For compiling, the following libraries are needed:
 - libargtable2-dev
 - libgsl0-dev
 - libsdl1.2-dev
 - libsdl-gfx1.2-dev 

If you do not want to use SDL or can not use SDL you can uncomment the fourth line of the makefile

## How to use

Type `make` to build everything except the bench tools
Type `make tools` to build everything
The `bin/` directory contains every executable built
Execute a binary without arguments to see how to you use it
