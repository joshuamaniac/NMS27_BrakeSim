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

    Fpedal = @(t) 800 .* min(max(t - 2, 0), 1);

% Initial calculations  -   These should be initialized BEFORE the loop!
    omega = struct( ...
    'FR', speed / tireRadius, ...
    'FL', speed / tireRadius, ...
    'RR', speed / tireRadius, ...
    'RL', speed / tireRadius);
   

% simulation bullshit
    i = 0;
    dt = 0.001;

% 
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
    step = brakeStep(car, FpedalNow, speed, omega, pedalRatio, bias, decel);

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

if isempty(history)
    warning('No simulation history is available to plot.');
    return
end

names = {'Vehicle speed [m/s]', 'Distance [m]', ...
    'Front wheel speed [rad/s]', 'Rear wheel speed [rad/s]', ...
    'FR slip', 'FL slip', 'RR slip', 'RL slip', ...
    'FR tyre Fx [N]', 'FL tyre Fx [N]', ...
    'RR tyre Fx [N]', 'RL tyre Fx [N]', ...
    'Front axle Fx [N]', 'Rear axle Fx [N]', ...
    'Deceleration [g]'};

timeData = reshape([history.time], 1, []);

data = {
    [history.speed]
    [history.distance]
    arrayfun(@(s) s.omega.FR, history)
    arrayfun(@(s) s.omega.RR, history)
    arrayfun(@(s) s.brakes.slip.FR, history)
    arrayfun(@(s) s.brakes.slip.FL, history)
    arrayfun(@(s) s.brakes.slip.RR, history)
    arrayfun(@(s) s.brakes.slip.RL, history)
    arrayfun(@(s) s.brakes.Fx.FR, history)
    arrayfun(@(s) s.brakes.Fx.FL, history)
    arrayfun(@(s) s.brakes.Fx.RR, history)
    arrayfun(@(s) s.brakes.Fx.RL, history)
    arrayfun(@(s) s.brakes.frontForce, history)
    arrayfun(@(s) s.brakes.rearForce, history)
    arrayfun(@(s) s.brakes.decel, history) / 9.81
};

% Normal load per tire
frontFz = arrayfun(@(s) s.brakes.frontNormal / 2, history);
rearFz  = arrayfun(@(s) s.brakes.rearNormal / 2, history);

% Normalized force is undefined for an unloaded tire.
frontFz(frontFz <= 0 | ~isfinite(frontFz)) = NaN;
rearFz(rearFz <= 0 | ~isfinite(rearFz)) = NaN;

% Add normalized force to both quantity dropdowns.
names = [names, {
    'FR normalized force [Fx/Fz]', ...
    'FL normalized force [Fx/Fz]', ...
    'RR normalized force [Fx/Fz]', ...
    'RL normalized force [Fx/Fz]'
    }];

data = [data; {
    arrayfun(@(s) s.brakes.Fx.FR, history) ./ frontFz
    arrayfun(@(s) s.brakes.Fx.FL, history) ./ frontFz
    arrayfun(@(s) s.brakes.Fx.RR, history) ./ rearFz
    arrayfun(@(s) s.brakes.Fx.RL, history) ./ rearFz
    }];

data = cellfun(@(values) reshape(values, 1, []), data, ...
    'UniformOutput', false);

if any(cellfun(@numel, data) ~= numel(timeData))
    error('BrakeSimulation:PlotDataMisaligned', ...
        'Each plotted quantity must have one value per time sample.');
end

% Window and controls
fig = uifigure('Name', 'Brake Simulation Explorer', ...
    'Position', [100 100 900 650]);

layout = uigridlayout(fig, [6 2]);
layout.RowHeight = {22, 30, 30, 22, 30, '1x'};
layout.ColumnWidth = {'1x', '1x'};

label1 = uilabel(layout, ...
    'Text', 'First quantity (X when plotting against each other)');
label1.Layout.Row = 1;
label1.Layout.Column = 1;

label2 = uilabel(layout, ...
    'Text', 'Second quantity (Y when plotting against each other)');
label2.Layout.Row = 1;
label2.Layout.Column = 2;

firstChoice = uidropdown(layout, ...
    'Items', names, 'Value', 'FR slip');
firstChoice.Layout.Row = 2;
firstChoice.Layout.Column = 1;

secondChoice = uidropdown(layout, ...
    'Items', [{'None'}, names], 'Value', 'FR tyre Fx [N]');
secondChoice.Layout.Row = 2;
secondChoice.Layout.Column = 2;

modeLabel = uilabel(layout, 'Text', 'Plot mode');
modeLabel.Layout.Row = 3;
modeLabel.Layout.Column = 1;

modeChoice = uidropdown(layout, ...
    'Items', {'Against time', 'Against each other'}, ...
    'Value', 'Against time');
modeChoice.Layout.Row = 3;
modeChoice.Layout.Column = 2;

startLabel = uilabel(layout, 'Text', 'Start time [s]');
startLabel.Layout.Row = 4;
startLabel.Layout.Column = 1;

endLabel = uilabel(layout, 'Text', 'End time [s]');
endLabel.Layout.Row = 4;
endLabel.Layout.Column = 2;

startTimeField = uieditfield(layout, 'numeric', ...
    'Value', min(timeData));
startTimeField.Layout.Row = 5;
startTimeField.Layout.Column = 1;

endTimeField = uieditfield(layout, 'numeric', ...
    'Value', max(timeData));
endTimeField.Layout.Row = 5;
endTimeField.Layout.Column = 2;

% Separate container lets each refresh completely replace the old axes.
plotPanel = uipanel(layout, 'BorderType', 'none');
plotPanel.Layout.Row = 6;
plotPanel.Layout.Column = [1 2];

plotLayout = uigridlayout(plotPanel, [1 1]);
plotLayout.Padding = [5 5 5 5];

refreshPlot = @(~,~) updateBrakePlot( ...
    plotLayout, firstChoice, secondChoice, modeChoice, ...
    startTimeField, endTimeField, names, data, timeData);

firstChoice.ValueChangedFcn = refreshPlot;
secondChoice.ValueChangedFcn = refreshPlot;
modeChoice.ValueChangedFcn = refreshPlot;
startTimeField.ValueChangedFcn = refreshPlot;
endTimeField.ValueChangedFcn = refreshPlot;

refreshPlot([], []);

function updateBrakePlot(plotLayout, firstChoice, secondChoice, ...
    modeChoice, startTimeField, endTimeField, names, data, timeData)

% Remove both old Y axes, plotted lines, labels, and legend.
delete(plotLayout.Children);
ax = uiaxes(plotLayout);
grid(ax, 'on');

startTime = startTimeField.Value;
endTime = endTimeField.Value;

if startTime > endTime
    title(ax, 'Start time must be less than or equal to end time.');
    return
end

% Apply one shared time mask to both quantities.
% This keeps X and Y paired at the same recorded timestep.
selected = timeData >= startTime & timeData <= endTime;

if ~any(selected)
    title(ax, 'No samples fall within the selected time interval.');
    return
end

firstIndex = find(strcmp(names, firstChoice.Value), 1);
secondIndex = find(strcmp(names, secondChoice.Value), 1);

t = timeData(selected);
firstValues = data{firstIndex}(selected);

if strcmp(modeChoice.Value, 'Against time')
    if isempty(secondIndex)
        plot(ax, t, firstValues, '.-', 'LineWidth', 1.5);
        ylabel(ax, firstChoice.Value);
        title(ax, [firstChoice.Value ' vs Time']);
    else
        secondValues = data{secondIndex}(selected);

        yyaxis(ax, 'left');
        firstLine = plot(ax, t, firstValues, '.-', ...
            'LineWidth', 1.5);
        ylabel(ax, firstChoice.Value);

        yyaxis(ax, 'right');
        secondLine = plot(ax, t, secondValues, '.-', ...
            'LineWidth', 1.5);
        ylabel(ax, secondChoice.Value);

        legend(ax, [firstLine, secondLine], ...
            {firstChoice.Value, secondChoice.Value}, ...
            'Location', 'best', 'Interpreter', 'none');

        title(ax, [firstChoice.Value ' and ' ...
            secondChoice.Value ' vs Time']);
    end

    xlabel(ax, 'Time [s]');

else
    if isempty(secondIndex)
        title(ax, 'Choose a second quantity to compare.');
        return
    end

    secondValues = data{secondIndex}(selected);

    % Points stay in timestep order within the selected interval.
    plot(ax, firstValues, secondValues, '.-', 'LineWidth', 1.5);
    xlabel(ax, firstChoice.Value);
    ylabel(ax, secondChoice.Value);

    title(ax, sprintf('%s vs %s | %.4g to %.4g s', ...
        secondChoice.Value, firstChoice.Value, startTime, endTime));
end

grid(ax, 'on');
end