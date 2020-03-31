# husky_path_follower

# Path generation
Path generation is done by making bezier curves between all points
Alternative method is to use pure quadratic curves.

Choice can be made by setting **Bezier** to true or false in *path_gains_config.yaml* config file

# Point selection
point1 **MUST** be [0,0] as that is the starting point of Husky
point2-point6 is configurable in *path_gains_config.yaml*
If Bezier is set to false, subsequent points must have an increasing x-value

# Control
Control is done using simple proportional gain on the linear velocity and angular velocity of Husky
Gains can be set using **linearGain** and **angularGain** in *path_gains_config.yaml*
