# Person Detect and Track Project

## Purpose
- This project aims at combining the detection and tracking algorithm together. Only person is deteced now.

## Output:
- Number of existing persons
- Tracking

## Usage:
- Firstly, you should enter 3rdparty to compile caffe using "make -j8 && make dist -j8"
- For our project, two construction methods: Makefile and CMakeLists.txt are provided
#### Custome the include and lib paths as your own
- Makefile would produce execute project in build directory. You would find as ./build/bankSleep. 
- For CMakeLists, you are recommended to make a directory firstly, then enter the directory and run "cmake .. && make -j8"
- Download the model from 
