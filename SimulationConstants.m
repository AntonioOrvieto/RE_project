function const = SimulationConstants()
% const = SimulationConstants()
% 
% Define the constants used in the simulation.  These constants are not 
% accessible to the estimator.
%
%
% Class:
% Recursive Estimation
% Spring 2016
% Programming Exercise 1
%
% --
% ETH Zurich
% Institute for Dynamic Systems and Control
% Raffaello D'Andrea, Michael Muehlebach
% michaemu@ethz.ch
%
% --
% Revision history
% [15.04.11, ST]    first version
% [07.05.13, MH]    first version
% [24.04.15, MM]    2015 version
% [14.04.16, MM]    2016 version


%% Speed

% The approximate maximum forward speed, in m/s.
const.MaxSpeedTranslation = 1.0;

% The approximate maximum rotational speed, in rad/s.
const.MaxSpeedRotation = 1.0;

% Minimum time for a segment, in seconds.
% Should be multiple of sampling time.
const.minSegTime = 0.5;

% Maximum time for a segment, in seconds.
% Should be multiple of sampling time.
const.maxSegTime = 2;


%% Robot kinematic constants

% The nominal right and left wheel radius (W_0), in meter.
const.NominalWheelRadius = 0.1;

% The variability in the wheel radius is captured by a uniform distribution, 
% with the following bound (\bar{\gamma}), in meters.
const.WheelRadiusError = 0.05; % const.WheelRadiusRelativeError = \bar{gamma}

% The wheel base (B), in meters.
const.WheelBase = 0.5;

%% Noise properties

% The compass sensor noise (w_r), normally distributed with zero mean 
% and variance \sigma_r^2, units rad^2.
const.CompassNoise = 0.05; % const.CompassNoise = \sigma_r^2

% The distance sensor noise (w_d), normally distributed with zero mean
% and variance \sigma_d^2, units rad^2.
const.DistNoise = 0.05; % const.DistNoise = \sigma_d^2

% Power spectral density of noise on wheel angular velocity commands (Q_v); 
% multiplicative noise, unit (rad/s)^2/Hz
const.VelocityInputPSD = 0.1; % const.VelocityInputNoise = Q_v

% Power spectral density of noise on driving wheel angle commands (Q_r); 
% units rad^2/Hz.
const.AngleInputPSD = 0.1; % const.AngleInputNoise = Q_r

%% Starting point

% The robot nominally starts at the origin, uniformly distributed with the
% following bound (\bar{p}), in meters.
const.TranslationStartBound = 1.0; % const.TranslationStartBound = \bar{p}

% The nominal orientation is also 0, and has a uniform distribution with
% the following bound (\bar{r}), in rad.
const.RotationStartBound = pi/2; % const.RotationStartBound = \bar{r}

%% Times

% Number of samples of the simulation, the total duration of the simulation 
% is then const.N*const.sampleContinuous in seconds.
const.N = 500;

% The sample time for the continuous dynamics, in seconds.
const.sampleContinuous = 0.1;

% The min sample time for the compass, in seconds.
const.sampleCompassMin = 0.5;

% The max sample time for the compass, in seconds.
const.sampleCompassMax = 1.0;

% The min sample time for the position sensors, in seconds.
const.samplePosMin = 0.5;

% The max sample time for the position sensors, in seconds.
const.samplePosMax = 1.0;

