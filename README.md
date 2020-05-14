# HRP-4-DART
HRP-4 DART simulation


Dynamic simulation of the position controlled HRP-4 robot performing a dynamic walk. The gait generation algorithm is designed to perform automatic footstep placement. The code is offered for benchmarking, review, education or as a simple example of simulation on Dart. Please contact the authors for any question.

DART REMARKS

Please install Dart (Dynamic Animation and Robotics Toolkit) https://dartsim.github.io/ It is strongly suggested to use Ubuntu 16.04/18.04 (in the sense that the simulation has been developed using these OS). ADVICE: install Dart from source by following the described procedure on Dart Website. Please type

         git checkout tags/v6.6.0       
              or
         git checkout tags/v6.6.1 

to not checkout the latest tag of DART 6, otherwise you may have issues in running the code. The problem is only related to Dart, so if you are already practiced with it, feel free to use any version.

Clone the repository. Type from terminal:

     git clone https://github.com/FilippoSmald1/HRP-4-DART.git

Decompress the the "Eigen.zip" and "qpOases.zip" folders.

Open "main.cpp" and change the path 

    auto ground = urdfLoader.parseSkeleton(
	"your current directory absolute path /urdf/ground.urdf");
    auto hrp4 = urdfLoader.parseSkeleton(
        "your current directory absolute path /urdf/hrp4.urdf"); 
        

Then type from terminal 
    
       mkdir build && cd build
       cmake .. && make
       
And eventually launch the simulation:

       ./Hrp4
       

SIMULATION REMARKS

The core component of the code are the Controller and the MPC solver. The MPC solver provides a CoM trajectory, expressed w.r.t. the support foot, to be kinematically tracked by the Controller, which is also responsible of the swing foot trajectory generation, the measurement collection and some further tools (data storage, external pushes management, etc.). Inverse Kinematic is addressed with a simple weighted pseudo inverse method. Jacobians are kindly provided by Dart.

The MPC gait generation is the peculiar part of the simulation. The simulation implements the IS-MPC (Intrinsically Stable MPC for humanoid gait generation developed at Diag Robotics Lab). 

    N. Scianca, M. Cognetti, D. De Simone, L. Lanari and G. Oriolo, "Intrinsically stable MPC for humanoid gait generation," 2016 IEEE-RAS 16th International Conference on Humanoid Robots (Humanoids), Cancun, 2016, pp. 601-606;

    N. Scianca, D. De Simone, L. Lanari and G. Oriolo, MPC for Humanoid Gait Generation: Stability and Feasibility.
