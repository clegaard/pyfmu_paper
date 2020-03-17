model BikeModelScenario
  BikeModel bike(v0=1);
  BikeDynamicModel betterBike(vx0=1);
  Modelica.Blocks.Sources.Pulse pulse(width=100, period=1, nperiod=1, amplitude=amplitude);
  parameter Real amplitude = Modelica.Constants.pi/4;
equation
  bike.a = 0;
  bike.deltaf = pulse.y;
  betterBike.a = 0;
  betterBike.deltaf = pulse.y;
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
end BikeModelScenario;
