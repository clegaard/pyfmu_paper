model BikeModel
  // Taken from https://ieeexplore.ieee.org/abstract/document/7225830
  // Parameters are taken from there as well.
  parameter Real lf=1.105 "distance from the the center of mass to the front (m)";
  parameter Real lr=1.738 "distance from the the center of mass to the rear (m)";
  parameter Real x0=0 "initial position in x";
  parameter Real y0=0 "initial position in y";
  parameter Real psi0=0 "initial orientation";
  parameter Real v0=0 "initial velocity along psi0";
  output Real x "x coordinate (m)";
  output Real y "y coordinate (m)";
  output Real psi "Inertial orientation of the model (rad)";
  output Real v "speed along psi (m/s)";
  output Real beta "is the angle of the current velocity vector
of the center of mass with respect to the longitudinal axis of
the car (rad)";
  input Real a "acceleration along v (m/s^2)";
  input Real deltaf "steering angle at the front wheel (rad)";
initial equation
  x = x0;
  y = y0;
  v = v0;
  psi= psi0;
equation
  der(x) = v * cos(psi + beta);
  der(y) = v * sin(psi + beta);
  der(psi) = (v/lr) * sin(beta);
  der(v) = a;
  beta = atan( (lr/(lf + lr)) * tan(deltaf) );
end BikeModel;
