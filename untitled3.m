% Brake Event Simulation
% Nittany Motorsports
% CBS, SBR Spring 2025
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Given initial speed and a pedal force, calculate the state of the car at
% one timestep

% Edited 9/28/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code

% load car specs
car = carspecs();

% define initial values
    muBrakepad = 0.8; % CHANGE: Given here, should calculate in separate file or include in specs
    pedalRatio = 4;    % CHANGE: Given here, should calculate in results function or include in specs
    bias = 0.5; % CHANGE: Given here, should be in car specs

    Fpedal = 100;
    Vinitial_mph = 60; 
    Vinitial = Vinitial_mph .* 0.44704;
    
    decel = 0;
  
function results = brakeStep(car, Fpedal, Vinitial, decel, muBrakepad, pedalRatio, bias)

    g = 9.81;
    
    m = car.vehicle.mass;
    L = car.vehicle.wheelbase;
    h = car.vehicle.cgHeight;
    frontFraction = 1 - car.vehicle.rearWeightPercent / 100;
    muTire = car.tire.frictionCoefficient; % This should be added to function inputs and be calculated separately per timestep


% What are we requesting the tires do at this moment?
    mechanical = brakeMechanical(Fpedal, car, muBrakepad, pedalRatio, bias);
    frontRequest = mechanical.frontTires;
    rearRequest = mechanical.rearTires;

% If this doesn't cause a lockup, what deceleration are we requesting?
    decelRequest = (frontRequest + rearRequest) ./ m;



% As a car brakes, some of the weight of the rear tires will be transferred
% to the front tires. This is only influenced (dynamically) by the current
% requested deceleration of the car. So we calculate
    weightTransfer = m .* decelRequest .* h ./ L; 

% Now we have a way of knowing the normal forces on each of the tires.
    frontNormal = m .* g .* frontFraction + weightTransfer;
    rearNormal = m .* g .* (1 - frontFraction) - weightTransfer;

% If tires lift off the ground, something is wrong!
    wheelLift = frontNormal <= 0 || rearNormal <= 0;

% Lets now find the limits of the tires
    frontLimit = frontNormal .* muTire;
    rearLimit = rearNormal .* muTire;

% Does the requested tire forces brake the limit of traction?
    frontAtLimit = frontRequest > 0 && frontRequest >= frontLimit;
    rearAtLimit = rearRequest > 0 && rearRequest >= rearLimit;

% Stop if it does!
    stopSimulation = wheelLift || frontAtLimit || rearAtLimit;

% Now we know if the car locks up or not, lets save everything for the next timestep.
    results.mechanical = mechanical;
    
    results.frontRequest = frontRequest;
    results.rearRequest = rearRequest;
    results.decelRequest = decelRequest;
    
    results.weightTransfer = weightTransfer;
    results.frontNormal = frontNormal;
    results.rearNormal = rearNormal;
    results.frontLimit = frontLimit;
    results.rearLimit = rearLimit;
    
    results.frontAtLimit = frontAtLimit;
    results.rearAtLimit = rearAtLimit;
    results.wheelLift = wheelLift;
    results.stopSimulation = stopSimulation;
    
    if stopSimulation
        % Actual force/deceleration beyond the limit is not modeled.
        results.frontForce = NaN;
        results.rearForce = NaN;
        results.decel = NaN;
    else
        results.frontForce = frontRequest;
        results.rearForce = rearRequest;
        results.decel = decelRequest;
    end

   

end

results = brakeStep(car, time, Fpedal, Vinitial, decel, muBrakepad, pedalRatio, bias)


