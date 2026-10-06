% Car Spec Sheet
% Nittany Motorsports
% CBS, SBR Spring 2025
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Spec sheet of car. All static components

% Edited 9/28/2026 by Joshua Todd
% ------------------------------------------------------------------------
%% Code

function car = carspecs()
% Vehicle
    car.vehicle.mass = 242.69611;               % Mass of car, kg
    car.vehicle.wheelbase = 1.580;              % Wheelbase, m
    car.vehicle.cgHeight = 0.239;               % Center of gravity height, m
    car.vehicle.rearWeightPercent = 50;         % Static rear weight distribution, %
    car.vehicle.tireRadius = 0.190;             % Effective tire radius, m

% Tires
    car.tireTest.frictionCoefficient = 1.2;     % Tire-road coefficient of friction REMOVE LATER

% Braking Condition
    car.braking.initialG = 1;                   % Initial guess for braking acceleration, g

% Rotors
    car.rotor.frontEffectiveRadius = 0.0795;    % Front effective rotor radius, m
    car.rotor.rearEffectiveRadius = 0.0762;     % Rear effective rotor radius, m

% Brake Pads - bp28 brake pad
    car.pad.frictionCoefficient = 0.48;         % Pad-rotor coefficient of friction

% Master Cylinders
    car.masterCylinder.front.diameter = 0.015875;     % Master cylinder piston diameter, m
    car.masterCylinder.rear.diameter  = 0.015875;     % Master cylinder piston diameter, m

% Front Calipers Wilwood gp320
    car.caliper.front.pistonDiameter = 0.032;   % Front caliper piston diameter, m
    car.caliper.front.pistonCount = 4;          % Total front caliper piston count

% Rear Calipers Wilwood gp200
    car.caliper.rear.pistonDiameter = 0.032;    % Rear caliper piston diameter, m
    car.caliper.rear.pistonCount = 2;           % Total rear caliper piston count

% Pedals
    car.pedal.heightForce           = 7.25;     % in
    car.pedal.heightBalanceBar      = 5.75;     % in
    car.pedal.heightMasterCylinder  = 0.25;     % in
    car.pedal.length                = 2.775;    % in

    car.pedal.ratio = pedalRatio(car.pedal, 0); % Pedal ratio of brake pedal at 0 degrees
  
% Tyres
    car.tire = MagicFormulaTyre();
    car.tire.loadTIR("C:\Users\joshu\Downloads\Hoosier_43070_16.0x6.0-10_R20_7_Sharp.tir");

    p = struct(car.tire.Parameters);
    version = car.tire.Version;
    
    car.tirepreset.wheelInertia = 0.2;                % kg /m^2  % WANRINNG UNEVERIFIED !!! CHANGE
    car.tirepreset.pressure     = 97000;              % Pa       % Unverified
end