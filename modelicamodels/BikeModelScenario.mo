model BikeModelScenario
  BikeModel bike;
  Modelica.Blocks.Sources.Sine sine;
  Modelica.Blocks.Sources.Pulse pulse(width=50, period=1, amplitude=amplitude, offset=-amplitude/2);
  parameter Real amplitude = Modelica.Constants.pi/4;
equation
  bike.a = 0;
  bike.deltaf = pulse.y;
  annotation(
    experiment(StartTime = 0, StopTime = 10, Tolerance = 1e-06, Interval = 0.002));
end BikeModelScenario;
