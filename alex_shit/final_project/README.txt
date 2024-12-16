CUDA Perceptions Pipeline
Authors: Alexander Blasberg, Alice Tran


This set of files is the implementation of a vehicle agnostic perceptions pipeline for an autonomous race car, taking in a LiDAR point
cloud, and returning a set of colored points representing the cones on track.


How to use:
This project compiles using CMake, so simply cd into the build folder and run "cmake --build ."
To execute the program, run the command "./perceptions <input file> <output file>"

For example: ./perceptions ../../point_clouds/intensity_1.csv output.csv

The current implemented coloring algorithm is midline. To swap to the clustering based approach, just comment out the existing code 
in "ColorCones.cu" and paste in the code from "cluster_color.cu".

If you would like to visualize the results, simply run new_vis.py.