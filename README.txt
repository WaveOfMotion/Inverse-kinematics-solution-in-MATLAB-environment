The following inverse kinematics solver calculates the imported robot model .urdf joints positions and orientations;
Implementation relies on input trajectory written in .txt or .xlsx file in which XYZ and Roll, Pitch, Yaw values must be provided as the first row of the file;
The file must contain coordinate and orientation data, where example of the trajectory is added to files.

For example: homeConfiguration = [0 -70 45 0 25 15.8]; would give you endEffectorrPos = [2.1871, 0, 3.17]

Since a 6DOF robot arm maniuplator has 6 joints, 3 of its joints are responsible for position, and other 3 for orientation:
jointA1 motion - changes in X coordinate;
jointA2 motion - changes in Y coordinate;
jointA3 motion - changes in Z coordinate;
jointA4 motion - changes Roll;
jointA5 motion - changes Pitch;
jointA6 motion - changes Jaw;

You can learn more about inverse kinematics from MathWorks: https://www.mathworks.com/help/robotics/ref/generalizedinversekinematics-system-object.html

As an input for this solver is a matrix of target positions, a vector [jointA1, jointA2, jointA3, jointA4, jointA5, jointA6] is calculated. 

Testing the written code, robot end effector position off-coursed of the given target position, and was likely do to not computing forward kiinematics to minimize the error. Also the algorithm parameters were used default and don't include adaptive parameterization when robot pose singularities occur leading to robot loosing course. The code provided the results, that robot gets back to home position safely without exceeding max velocity or joint limits. 

As conclusion, an inverse kinematics implementation in C/C++ would ensure faster robot pose computation and by implementing algorithm logic inside the code would create a stronger mathematical model. In the early stages of the code development in MATLAB, writing DH parameters from finished robot model showed faster code execution. An imported robot model in .urdf format was used instead because extracting dh parameters from robot model took a long time and wasn't succesfull.
