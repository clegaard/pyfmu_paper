model Driver
  parameter Real width = 0.335;
  parameter Real amplitude = 0.8;
  parameter Real risingtime = 5.0;
  parameter Real starttime = 5.0;
  parameter Integer nperiod = 1;
  parameter Real period = 300;
  output Real steering;  
  Modelica.Blocks.Sources.Trapezoid trap(nperiod=nperiod, startTime=starttime, rising=risingtime, falling=risingtime, period=period, width=width, amplitude=amplitude);
equation
  
  steering = trap.y;
  
end Driver;
