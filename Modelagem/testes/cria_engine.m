sampleTime = 0.0625;
numSteps = 567;
time = sampleTime*(0:numSteps-1);
time = time';
grausPendlo = Integradorpendulodadosentregue.PosioAngularPndulorad
Engine=timeseries(grausPendlo,time);
