model KinematicsDriver
  input Real u;
  output Real y;
  parameter Real delayTime = 0.3;
  parameter Real k=0.9;
equation
  y = delay(k*u, delayTime);
end KinematicsDriver;
