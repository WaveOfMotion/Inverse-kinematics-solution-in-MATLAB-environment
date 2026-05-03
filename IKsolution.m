function output = IKsolution(trajectory)
  
  % Upload robot model
  robot = importrobot("kukaKR600.urdf", "DataFormat", "row");
  endEffector = "bodyGondola";

  % Define position, orientation columns of input file
  data = readmatrix(trajectory);
  X = data(:,1);
  Y = data(:,2);
  Z = data(:,3);
  Roll  = deg2rad(data(:,4));
  Pitch = deg2rad(data(:,5));
  Yaw   = deg2rad(data(:,6));
  MatrixLenght = size(data, 1);

  % Define robot initial pose
  lastvalidConfiguration = deg2rad([0, -70, 45, 0, 25, 15.8]);
  alljointAngles = zeros(MatrixLenght, 6);

  % Define 'generalizedInverseKinematics' object parameters
  GK = generalizedInverseKinematics('RigidBodyTree', robot, ...
  'ConstraintInputs', {'position', 'orientation', 'joint'}, ...
  'SolverAlgorithm', 'BFGSGradientProjection');

  % Define max iteration and tolerance
  GK.SolverParameters.MaxIterations     = 100;
  GK.SolverParameters.SolutionTolerance = 1e-3;
  GK.SolverParameters.RelativeTolerance = 1e-3;

  % Define joints limits of imported robot model
  jointLimits = constraintJointBounds(robot);
  jointLimits.Bounds = deg2rad([ ...
   -167   165; % jointA1
   -93    -3; % jointA2
   -24    55; % jointA3
   -345   345; % jointA4
   -60    60; % jointA5
   -275   295]); % jointA6

  jointLimits.Weights = [1 1 1 1 1 1];

  targetPosition = constraintPositionTarget(endEffector);
  targetPosition.PositionTolerance = 1e-4;
  targetOrientation = constraintOrientationTarget(endEffector);
  targetOrientation.OrientationTolerance = deg2rad(1);   % ← 1 °

  % Compute spoition, orientation for each trajectory point
  for i = 1:MatrixLenght
    targetPosition.TargetPosition = [X(i); Y(i); Z(i)];
        Rotation = eul2rotm([Yaw(i), Pitch(i), Roll(i)], 'ZYX');
        quaternion = rotm2quat(Rotation);
        targetOrientation.TargetOrientation = quaternion;
        kinematicSolution = GK(lastvalidConfiguration, ...
                               targetPosition, ...
                               targetOrientation, ...
                               jointLimits);

    % Limit the max velocity of joint movement
    delta = kinematicSolution - lastvalidConfiguration;
    timeStep = 0.004;
    maxJointStep = deg2rad([30 30 30 30 30 30]) * timeStep;
    ratio = abs(delta) ./ maxJointStep;
    if any(ratio > 1)
        scale = 1 / max(ratio);
        kinematicSolution = lastvalidConfiguration + delta * scale;
    end

    alljointAngles(i, :) = kinematicSolution;
    alljointAngles  = max(alljointAngles, jointLimits.Bounds(:,1)');
    alljointAngles  = min(alljointAngles, jointLimits.Bounds(:,2)');
    lastvalidConfiguration = kinematicSolution;

    fprintf("Frame % d solved\n", i);
  end

  % Define outputt format
  time = (0:timeStep:(MatrixLenght - 1) * timeStep)';
  FinaljointAngles = rad2deg(alljointAngles);
  offset = FinaljointAngles(1, :);
  result = FinaljointAngles - offset;

  % Smooth each joint movement with butterworth filtering (Optional: for better results) 
  Fc = 1;  % Hz
  Fs = 1 / timeStep;
  [b, a] = butter(3, Fc / (Fs/2));
  smoothedResult = filtfilt(b, a, result);

  % Extract matrix-of-joint-angles
  output = [time, smoothedResult];
  writematrix(output, "angles.txt", "Delimiter", " ");
end