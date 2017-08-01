# Geometric Control on SE(3)

Central Repo for SE(3) geometric controller

## TODO

* Different programming languages
  * [X] C++/C
    * [X] [ROS Package](https://github.com/fdcl-gwu/uav_geometric_controller)
  * [X] Python
  * [X] Matlab
  * All languages equivalent (same function and variable names)
* [X] Unit testing/validtion
  * [X] Python Controller has some testing
  * [X] C++ version using `gtest` 
* [ ] Documentation
  * [X] Links to papers
    - [Geometric tracking control of a quadrotor UAV on SE (3)](https://pdfs.semanticscholar.org/2e83/b6f1d6da2694dd029597911599c03b690afc.pdf)
    - [Exponential Stability of an Attitude Tracking Control System on SO(3) for Large-Angle Rotational Maneuvers](http://s3.amazonaws.com/academia.edu.documents/45675158/j.sysconle.2011.10.01720160516-17935-cn7fix.pdf?AWSAccessKeyId=AKIAIWOWYYGZ2Y53UL3A&Expires=1501541417&Signature=q%2FGTrQ2F07OChQIr8jLb23sK5sg%3D&response-content-disposition=inline%3B%20filename%3DExponential_stability_of_an_attitude_tra.pdf)
    - [Geometric Nonlinear PID Control of a Quadrotor UAV on SE(3)](https://arxiv.org/pdf/1304.6765.pdf)
    - [Control of Complex Maneuvers for a Quadrotor UAV using Geometric Methods on SE(3)](https://arxiv.org/pdf/1003.2005.pdf)
  * [ ] References to work from lab that uses/modifies it
* [ ] Other variations of this controller
  * [ ] SO(3) Only for attitude
  * [ ] Translation only
  * [X] SO(3) avoidance
    * [Adaptive Control on SO(3) with Constraints](https://shankarkulumani.com/2016/08/2016ACC.html)
      * [Code](https://github.com/fdcl-gwu/2016_ACC_matlab)
      * [Paper](https://github.com/fdcl-gwu/2016_acc_manuscript)
  * [X] SE(3) Controller for a rigid dumbbell around an asteroid
    * [Python Code](https://github.com/fdcl-gwu/asteroid_dumbbell)
* [ ] Avoid complicated dependencies
  * [ ] Need to list dependencies - `virtualenv` or `conda env` etc.
* [X] Kinematics library
