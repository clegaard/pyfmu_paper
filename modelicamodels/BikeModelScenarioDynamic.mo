model BikeModelScenarioDynamic
  BikeDynamicModel betterBike(vx0=1);
  Driver driver;
equation
  betterBike.a = 0;
  betterBike.deltaf = driver.steering;
  annotation(
    experiment(StartTime = 0, StopTime = 200, Tolerance = 1e-06, Interval = 0.002));
end BikeModelScenarioDynamic;
