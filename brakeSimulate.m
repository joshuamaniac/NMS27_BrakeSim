% Brake Event Simulation
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Simulates Braking! This is just the single function version 

% Edited 9/28/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code
function simulation = brakeSimulate(car, Fpedal, vInitial)
car.tireParameters = struct(car.tire.Parameters);

% define initial values i need to fix
    muBrakepad = 0.8; % CHANGE: Given here, should calculate in separate file or include in specs
    pedalRatio = car.pedal.ratio;    
    bias = 0.5; % CHANGE: Given here, should be in car specs
    
    maxTime = 10;


    tireRadius = car.vehicle.tireRadius;
       

% Initial state
    distance = 0;
    
    Vinitial_mph = vInitial; 
    Vinitial = Vinitial_mph .* 0.44704;
    speed = Vinitial;

    decel = 0;
    time = 0;

   

% Initial calculations  -   These should be initialized BEFORE the loop!
    omega = struct( ...
    'FR', speed / tireRadius, ...
    'FL', speed / tireRadius, ...
    'RR', speed / tireRadius, ...
    'RL', speed / tireRadius);
   

% simulation bullshit
    i = 0;
    dt = 0.01;

% shits cool thanks chatgpt i need to understand this
    history = struct([]);
    endReason = "Maximum time reached";

%% START LOOP
while true

%debug
%fprintf('Time: %.3f s\n', time);

% Choose current step length
stepDuration = min(dt, maxTime - time);
dt = stepDuration;

% did the car stop?
    if speed <= 0
        endReason = "Car is stopped";
        break
    end

% Did we hit the time limit?
    if time >= maxTime
        endReason = "Maximum time reached";
        break
    end


% Evaluate physics at current time
    FpedalNow = Fpedal(time);
    step = brakeStep(car, FpedalNow, speed, omega, muBrakepad, pedalRatio, bias, decel);

    decel = step.decel;
    slip = step.slip;

% Calculate distance and speed traveled beforehand, as it assumes acceleration is
% constant
    distance = distance + speed * stepDuration - 0.5 * decel * stepDuration^2;
    
    speed = max(0, speed - decel * stepDuration);

% Advance wheel speed
    omega.FR = max(0, omega.FR + step.omegaDot.FR * dt);
    omega.FL = max(0, omega.FL + step.omegaDot.FL * dt);
    omega.RR = max(0, omega.RR + step.omegaDot.RR * dt);
    omega.RL = max(0, omega.RL + step.omegaDot.RL * dt);


% Save into history
    i = i + 1;
    history(i).time = time;
    history(i).speed = speed;
    history(i).omega = omega;
    history(i).slip = step.slip;
    history(i).distance = distance;
    history(i).Fpedal = FpedalNow;
    history(i).brakes = step;
    history(i).decel = decel;

    

% Now, do we need to end the simulation?
    if step.stopSimulation
        if step.wheelLift
            endReason = "Wheel lift threshold reached";
        elseif step.frontLock
            endReason = "Front tires locked";
        elseif step.rearLock
            endReason = "Rear tires locked";
        else
            endReason = "what the fuck?";
        end
        break
    end

    if any([omega.FR, omega.FL, omega.RR, omega.RL] <= 0)
        endReason = "Wheel reached or crossed zero rotational speed";
        fprintf('Wheel speeds: FR=%.3f, FL=%.3f, RR=%.3f, RL=%.3f rad/s\n', ...
            omega.FR, omega.FL, omega.RR, omega.RL);
        break
    end

  
    
    % Shorten the final step if the car stops during it.
    stopsThisStep = false;
    
    if step.decel > 0
        timeToStop = speed / step.decel;
    
        if timeToStop <= stepDuration
            stepDuration = timeToStop;
            stopsThisStep = true;
        end
    end
    
    if stopsThisStep
        speed = 0;
    end

    % Next step!
    time = time + dt;
end

% Results
simulation.history = history;
simulation.endReason = endReason;

% Final state is stored separately because it may fall between samples.
simulation.finalTime = time;
simulation.finalSpeed = speed;
simulation.finalDistance = distance;

simulation.stoppingDistance = NaN;

if endReason == "Car is stopped"
    simulation.stoppingDistance = distance;
end

disp(simulation.endReason);
fprintf('Final time: %.3f s\n', time);
fprintf('Final speed: %.3f m/s\n', speed);
fprintf('Distance travelled: %.3f m\n', distance);


end


