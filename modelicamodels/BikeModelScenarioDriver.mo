model BikeModelScenarioDriver
  BikeModel bike(v0=1);
  Driver driver;
  BikeDynamicModel betterBike(vx0=1);
  Real kph_factor = 0.001 * 60^2;
  Real vkmh_k = bike.v * kph_factor;
  Real vkmh_d = betterBike.vx * kph_factor;
  parameter Real anglek = 1.0;
  parameter Real angled = 1.0;
equation
  bike.a = 0;
  bike.deltaf = anglek*driver.steering.y;
  betterBike.a = 0;
  betterBike.deltaf = angled*driver.steering.y;
  annotation(
    experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.002));
end BikeModelScenarioDriver;
