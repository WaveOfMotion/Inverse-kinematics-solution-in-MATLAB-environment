### Developing Generalized Inverse Kinematics for Industrial robot Kuka KR600 R2830 Passenger
Me developping my first inverse kinematics application, where my first robot was KukaKR600 R2830 Passenger, I used MATLAB Robot System Toolbox and specifically
**GeneralizedKinematics**. For the robot import I used .urdf import :
```iecst
robot = importrobot("kukaKR600.urdf", "DataFormat", "row");
endEffector = "bodyGondola";
```
My goal was to generate a path, to which the 2,650 tonns heavy robot could follow. So I used EXCEL since its easy to use, and specified position and orientation :
```iecst
data = readmatrix(trajectory);
X = data(:,1);
Y = data(:,2);
Z = data(:,3);
Roll  = deg2rad(data(:,4));
Pitch = deg2rad(data(:,5));
Yaw   = deg2rad(data(:,6));
MatrixLenght = size(data, 1);
```
, then specifying start position, and call the object :
```iecst
GK = generalizedInverseKinematics('RigidBodyTree', robot, ...
  'ConstraintInputs', {'position', 'orientation', 'joint'}, ...
  'SolverAlgorithm', 'BFGSGradientProjection');
```
, then specify its parameters, robot joint limits and call the **main cycle**. I also define maximum joint speed, for instance, I don't want the robot joints
to move faster than 30 deg/s. Then I use scaling method - if any joint is moving faster than 30 deg/s, all joints are scaled by the difference above the limit:
```iecst
 for i = 1:MatrixLenght
    targetPosition.TargetPosition = [X(i); Y(i); Z(i)];
        Rotation = eul2rotm([Yaw(i), Pitch(i), Roll(i)], 'ZYX');
        quaternion = rotm2quat(Rotation);
        targetOrientation.TargetOrientation = quaternion;
        kinematicSolution = GK(lastvalidConfiguration, ...
                               targetPosition, ...
                               targetOrientation, ...
                               jointLimits);

    delta = kinematicSolution - lastvalidConfiguration;
    timeStep = 0.004;
    maxJointStep = deg2rad([30 30 30 30 30 30]) * timeStep;
    ratio = abs(delta) ./ maxJointStep;
    if any(ratio > 1)
        scale = 1 / max(ratio);
        kinematicSolution = lastvalidConfiguration + delta * scale;
    end
```
To make the generated joint angles smooth, I use 3rd order butterfiltering. To use it, you must install Signal Processing Toolbox from **Add-ons** which will take couple of seconds.
Every column of joint angle, you can think of as signal, because if you put joint angle in Y axes, and time in X axes in a graph, you get something a line with amplitude.
So I tell the code - filter the signal so that the spikes are under 1 Hz, and I do it in forward annd backward direction, so that the signal doesn't shift forward:
```iecst
Fc = 1;  % Hz
  Fs = 1 / timeStep;
  [b, a] = butter(3, Fc / (Fs/2));
  smoothedResult = filtfilt(b, a, result);
```
, and the output:
```iecst
output = [time, smoothedResult];
```
, where each 4 ms is a new vector of siix joint angles passed to UDP Send in Simulink. 

## Conclusion
Testing the application and the results, one was noticable that roboot end-effector drifted off from the XYZ giiven position. The application proved, that the robot
came back to the home position with the given initial configuration:
```iecst
homePosition = deg2rad([0, -70, 45, 0, 25, 15.8]);
```
, and didn't exceed the given velocity limits. I have to reminnd. that I used built-in **Levenberg-Marquardt** and **Broyden–Fletcher–Goldfarb–Shanno** when calling
the **generaliizedInverseKinematics** object, where the weights for joint limits where used as default:
```iecst
jointLimits.Weights = [1 1 1 1 1 1];
```
The application was a success, because it proved that inverse kinematics problem can be solved with using built-in Robot System Toolbox libraries.
