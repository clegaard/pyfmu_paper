model BikeModelScenarioDriver
  parameter Real speed = 1 "m/s";
  BikeModel kbike(v0=speed);
  Driver dDriver;
  BikeDynamicModel dbike(vx0=speed);
  KinematicsDriver kDriver();
equation
  kbike.a = 0;
  dbike.a = 0;
  kbike.deltaf = kDriver.y;
  dbike.deltaf = dDriver.steering;
  kDriver.u = dDriver.steering;
  annotation(
    experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.002));
end BikeModelScenarioDriver;
