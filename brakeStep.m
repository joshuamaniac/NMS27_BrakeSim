% Brake Simulation (Step)
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Represents a timestep in a brake event.

% Edited 10/1/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code
function results = brakeStep(car, Fpedal, speed, omega, muBrakepad, pedalRatio, bias, decel)


g = 9.81;

m = car.vehicle.mass;
L = car.vehicle.wheelbase;
h = car.vehicle.cgHeight;
frontFraction = 1 - car.vehicle.rearWeightPercent / 100;
muTire = car.tireTest.frictionCoefficient; % This will be removed once MF is implemented. Aswell as alot of other things!


% What are we requesting the tires do at this moment? % Old? 
mechanical = brakeMechanical(Fpedal, car, muBrakepad, pedalRatio, bias);

frontRequest = mechanical.frontTires;
rearRequest = mechanical.rearTires;

% If this doesn't cause a lockup, what deceleration are we requesting?
%decelRequest = (frontRequest + rearRequest) ./ m;
decelRequest = decel; %OLD AND USELESS, STOP USING DECEL REQUEST

% As a car brakes, some of the weight of the rear tires will be transferred
% to the front tires. This is only influenced (dynamically) by the current
% requested deceleration of the car. So we calculate;
weightTransfer = m .* decelRequest .* h ./ L; 

% Now we have a way of knowing the normal forces on each of the tires.
% ASSUMING NO SIDE MOTION YET, JUST DIVIDE BY 2 TO GET NORMAL ON EACH TIRE
frontNormal = m .* g .* frontFraction + weightTransfer;
rearNormal = m .* g .* (1 - frontFraction) - weightTransfer;

% MAGIC FORMULA!
% First calculate slip
slip.FR = ( omega.FR .* car.vehicle.tireRadius - speed ) ./ speed;
slip.FL = ( omega.FL .* car.vehicle.tireRadius - speed ) ./ speed;
slip.RR = ( omega.RR .* car.vehicle.tireRadius - speed ) ./ speed;
slip.RL = ( omega.RL .* car.vehicle.tireRadius - speed ) ./ speed;

% Do the magic formula!
%fprintf('speed = %.6f m/s, omega.FR = %.6f rad/s, slip.FR = %.6f\n', speed, omega.FR, slip.FR);

% Order: FR, FL, RR, RL
slipVector = [slip.FR, slip.FL, slip.RR, slip.RL];
normalVector = [frontNormal, frontNormal, rearNormal, rearNormal] / 2;

FxVector = magicformula(car.tireParameters, slipVector, 0, ...
    normalVector, car.tirepreset.pressure, 0, speed, [1 0 1 0]);

Fx.FR = FxVector(1);
Fx.FL = FxVector(2);
Fx.RR = FxVector(3);
Fx.RL = FxVector(4);

%fprintf('Fx.FR = %.6f, Fx.FL = %.6f, Fx.RR = %.6f, Fx.RL = %.6f\n', Fx.FR, Fx.FL, Fx.RR, Fx.RL);

% Now we can find the change in wheelspeed
brakeTorqueVector = 0.5 .* [mechanical.brakeTorque.front, ...
                            mechanical.brakeTorque.front, ...
                            mechanical.brakeTorque.rear, ...
                            mechanical.brakeTorque.rear];

omegaDotVector = (-brakeTorqueVector - car.vehicle.tireRadius .* FxVector) ...
    ./ car.tirepreset.wheelInertia;

[omegaDot.FR, omegaDot.FL, omegaDot.RR, omegaDot.RL] = deal(omegaDotVector(1), ...
    omegaDotVector(2), omegaDotVector(3), omegaDotVector(4));

% Now lets find the requested decelerations % Should be changed per wheel?
decel = -(Fx.FR + Fx.FL + Fx.RR + Fx.RL) / m;

% If tires lift off the ground, something is wrong!
wheelLift = frontNormal <= 0 || rearNormal <= 0;

% Lets now find the limits of the tires
%frontLimit = frontNormal .* muTire;
%rearLimit = rearNormal .* muTire;

% Does the requested tire forces brake the limit of traction?
frontLock = [];
rearLock = [];

frontLock = slip.FR <= -1 || slip.FL <= -1;
rearLock = slip.RR <= -1 || slip.RL <= -1;

% Stop if it does!
stopSimulation = wheelLift || frontLock || rearLock;

% Now we know if the car locks up or not, lets save everything for the next timestep.
results.mechanical = mechanical;

results.frontForce = Fx.FR + Fx.FL;
results.rearForce = Fx.RR + Fx.RL;
results.decelRequest = decelRequest; % Change wording maybe?
results.decel = decel;

results.omegaDot.FR = omegaDot.FR;
results.omegaDot.FL = omegaDot.FL;
results.omegaDot.RR = omegaDot.RR;
results.omegaDot.RL = omegaDot.RL;

results.slip.FR = slip.FR;
results.slip.FL = slip.FL;
results.slip.RR = slip.RR;
results.slip.RL = slip.RL;

results.slip = slip;
results.Fx = Fx;

results.weightTransfer = weightTransfer;
results.frontNormal = frontNormal;
results.rearNormal = rearNormal;
results.frontLock = frontLock;
results.rearLock = rearLock;


results.wheelLift = wheelLift;
results.stopSimulation = stopSimulation;

if stopSimulation
    % Actual force/deceleration beyond the limit is not modeled.
    results.frontForce = NaN;
    results.rearForce = NaN;
    results.decel = NaN;
else
    results.frontForce = Fx.FR + Fx.FL;
    results.rearForce = Fx.RR + Fx.RL;
    results.decel = decel;
end



end

