model BikeDynamicModel
  // Taken from https://ieeexplore.ieee.org/abstract/document/7225830
  // Parameters are taken from there as well and from https://vtechworks.lib.vt.edu/bitstream/handle/10919/36615/Chapter2a.pdf.
  // Cornering coefficients taken from https://pdfs.semanticscholar.org/6644/63535e9404f78668acbe8cd99443c918cc30.pdf
  parameter Real lf=1.105 "distance from the the center of mass to the front (m)";
  parameter Real lr=1.738 "distance from the the center of mass to the rear (m)";
  parameter Real m=1292.2 "Vehicle's mass (kg)";
  parameter Real Iz = 1 "Yaw inertial (kgm^2) (Not taken from the book)";
  parameter Real Caf = 800 "Front Tire cornering stiffness";
  parameter Real Car = 800 "Rear Tire cornering stiffness";
  parameter Real x0=0 "initial longitudinal displacement";
  parameter Real y0=0 "initial lateral displacement";
  parameter Real X0=0 "initial position in x";
  parameter Real Y0=0 "initial position in y";
  parameter Real psi0=0 "initial yaw";
  parameter Real dpsi0=0 "initial yaw rate";
  parameter Real vx0=1.0 "initial velocity along x";
  parameter Real vy0=0 "initial velocity along x";
  output Real af "Front Tire slip angle";
  output Real ar "Rear Tire slip angle";
  output Real x "longitudinal displacement in the body frame";
  output Real X "x coordinate in the reference frame";
  output Real Y "x coordinate in the reference frame";
  output Real vx "velocity along x";
  output Real y "lateral displacement in the body frame";
  output Real vy "velocity along y";
  output Real psi "Yaw";
  output Real dpsi "Yaw rate";
  input Real a "longitudinal acceleration";
  input Real deltaf "steering angle at the front wheel";
  output Real Fcf "lateral tire force at the front tire in the frame of the front tire";
  output Real Fcr "lateral tire force at the rear tire in the frame of the rear tire";
initial equation
  x = x0;
  y = y0;
  X = X0;
  Y = Y0;
  dpsi = dpsi0;
  vx = vx0;
  vy = vy0;
  psi = psi0;
equation
  der(x) = vx;
  der(y) = vy;
  der(psi) = dpsi;
  
  der(vx) = dpsi*vy + a;
  
  der(vy) = -dpsi*vx + (1/m)*(Fcf * cos(deltaf) + Fcr);
  der(dpsi) = (2/Iz)*(lf*Fcf - lr*Fcr);
  
  der(X) = vx*cos(psi) - vy*sin(psi);
  der(Y) = vx*sin(psi) + vy*cos(psi);
  
  Fcf = Caf*af;
  Fcr = Car*(-ar);
  
  // Adapted from https://vtechworks.lib.vt.edu/bitstream/handle/10919/36615/Chapter2a.pdf
  af = deltaf - ( vy + lf*dpsi)/vx;
  ar = (vy - lr*dpsi)/vx;
  
end BikeDynamicModel;
