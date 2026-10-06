% Brake Event Simulation
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Simulates Braking! 

% Edited 9/28/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code
clc, clear%, close all
% load car specs and tire specs
car = carspecs();
car.tireParameters = struct(car.tire.Parameters);

% define initial values i need to fix
    muBrakepad = 0.8; % CHANGE: Given here, should calculate in separate file or include in specs
    pedalRatio = car.pedal.ratio;    
    bias = 0.5; % CHANGE: Given here, should be in car specs
    
    maxTime = 10;


    tireRadius = car.vehicle.tireRadius;
       

% Initial state
    distance = 0;
    
    Vinitial_mph = 30; 
    Vinitial = Vinitial_mph .* 0.44704;
    speed = Vinitial;

    decel = 0;
    time = 0;

    Fpedal = @(t) 100 .* min(max(t - 2, 0), 1);

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


%% Lets do some graphs! Thanks ChatGPT!

% Collect quantities available for plotting
names = {'Time [s]', 'Vehicle speed [m/s]', 'Distance [m]', ...
    'Front wheel speed [rad/s]', 'Rear wheel speed [rad/s]', ...
    'Front slip', 'Rear slip', ...
    'Front tyre Fx [N]', 'Rear tyre Fx [N]', 'Deceleration [g]'};

data = {
    [history.time]
    [history.speed]
    [history.distance]
    arrayfun(@(s) s.omega.FR, history)
    arrayfun(@(s) s.omega.RR, history)
    arrayfun(@(s) s.brakes.slip.FR, history)
    arrayfun(@(s) s.brakes.slip.RR, history)
    arrayfun(@(s) s.brakes.Fx.FR, history)
    arrayfun(@(s) s.brakes.Fx.RR, history)
    arrayfun(@(s) s.brakes.decel, history) / 9.81
};

% Create the window
fig = uifigure('Name', 'Brake Simulation Explorer');
layout = uigridlayout(fig, [3 2]);
layout.RowHeight = {22, 30, '1x'};
layout.ColumnWidth = {'1x', '1x'};

uilabel(layout, 'Text', 'X axis');
uilabel(layout, 'Text', 'Y axis');

xChoice = uidropdown(layout, 'Items', names, 'Value', names{1});
yChoice = uidropdown(layout, 'Items', names, 'Value', names{2});

ax = uiaxes(layout);
ax.Layout.Row = 3;
ax.Layout.Column = [1 2];

% Refresh whenever either dropdown changes
refreshPlot = @(~,~) updateBrakePlot(ax, xChoice, yChoice, names, data);
xChoice.ValueChangedFcn = refreshPlot;
yChoice.ValueChangedFcn = refreshPlot;

refreshPlot([], []);

function updateBrakePlot(ax, xChoice, yChoice, names, data)
xIndex = find(strcmp(names, xChoice.Value), 1);
yIndex = find(strcmp(names, yChoice.Value), 1);

plot(ax, data{xIndex}, data{yIndex}, 'LineWidth', 1.5);
xlabel(ax, xChoice.Value);
ylabel(ax, yChoice.Value);
title(ax, [yChoice.Value ' vs ' xChoice.Value]);
grid(ax, 'on');
end