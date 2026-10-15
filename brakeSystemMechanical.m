% Mechanical Brake Components Calculation
% Nittany Motorsports
% CBS, SBR Spring 2025
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Find requested force on tires from force on brake pedal. 

% Edited 9/28/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code

% load car specs
car = carspecs();

% define initial values
Fpedal = 400;
muBrakepad = 0.8; % CHANGE: Given here, should calculate in seperate file or include in specs
pedalRatio = 4;    % CHANGE: Given here, should calculate in results function or include in specs
bias = 0.5; % CHANGE: Given here, should be in car specs




% CHANGE: Inputs are dogshit, consolidate
function mechanical = brakesystemMechanical(Fpedal, car, muBrakepad, pedalRatio, bias)

% Initial Calculations
    masterCylinder.area = (car.masterCylinder.diameter).^2 .* (0.25) .* pi;
    caliperPiston.area.front = car.caliper.front.pistonCount .* (car.caliper.front.pistonDiameter).^2 .* (0.25) .* pi;
    caliperPiston.area.rear = car.caliper.rear.pistonCount .* (car.caliper.rear.pistonDiameter).^2 .* (0.25) .* pi;
    
 
% It is important to distinguish that this is a pedal forward calculation. The ultimate question we are asking is 
% "For our car setup and a given force on pedal, what force do we think the tires experience?"

% Pedal force after mechanical advantage of pedal
    pedal.forceOutput = Fpedal .* pedalRatio;

% Force applied to each master cylinder
    masterCylinder.force.front = bias .* pedal.forceOutput;
    masterCylinder.force.rear = (1 - bias) .* pedal.forceOutput;

% Hydraulic line pressures
    line.pressure.front = masterCylinder.force.front ./ masterCylinder.area;
    line.pressure.rear = masterCylinder.force.rear ./ masterCylinder.area;

% The pressure in the lines go to two calipers each. Here, we calculate the
% clamping force EACH caliper places on the brake pads
    calipers.clampForce.front = line.pressure.front .* caliperPiston.area.front;
    calipers.clampForce.rear = line.pressure.rear .* caliperPiston.area.rear;

% Now, knowing the force on the brake pads, we can find the force due to
% friction on the rotors. 
    pad.frictionForce.front = muBrakepad .* calipers.clampForce.front;
    pad.frictionForce.rear = muBrakepad .* calipers.clampForce.rear;

% Knowing what friction force is on the rotors, we can translate this to a
% torque. We use the effective radius of the rotor to find the torque on
% the axle.
    axle.torque.front = pad.frictionForce.front .* car.rotor.frontEffectiveRadius;
    axle.torque.rear = pad.frictionForce.rear .* car.rotor.rearEffectiveRadius;

    
% We now translate that torque into the force experienced by the tires
% from the brakes.
    tire.force.front = axle.torque.front ./ car.vehicle.tireRadius;
    tire.force.rear = axle.torque.rear ./ car.vehicle.tireRadius;

% There are two tires per side of the car! We don't need the total force of
% the tires, as the front and the rear are going to produce different
% normal forces due to weight distribution and transfer. We only need the force
% of the front and rear tires separately
    tire.force.frontTotal = tire.force.front .* 2;
    tire.force.rearTotal = tire.force.rear .* 2;


% Results
    mechanical.frontTires = tire.force.frontTotal;
    mechanical.rearTires = tire.force.rearTotal;
    mechanical.frontLinePressure = line.pressure.front;
    mechanical.rearLinePressure = line.pressure.rear;

end

results = brakesystemMechanical(Fpedal, car, muBrakepad, pedalRatio, bias);

fprintf('Front tires: %g\n', results.frontTires);
fprintf('Rear tires: %g\n', results.rearTires);
fprintf('Front line pressure: %g\n', results.frontLinePressure);
fprintf('Rear line pressure: %g\n', results.rearLinePressure);