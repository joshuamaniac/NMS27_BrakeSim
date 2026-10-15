% Iterative Sizing Simulation
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% chatgpt master cylinder sizing

% Edited 9/28/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Code

%% Master-cylinder sizing sweep
clear;
clc;

% Candidate bore diameters [m]
% 5/8", 7/10", 3/4", 13/16", 7/8", 15/16", 1"
frontBores = [0.015875, 0.017780, 0.019050, 0.0206375, ...
              0.022225, 0.0238125, 0.025400];

rearBores = frontBores;

% Identical braking input for every combination
Fpedal = @(t) 300 .* min(max(t - 2, 0), 1);
vInitial = 60; % mph

baseCar = carspecs();

%% Allocate results
n = numel(frontBores) * numel(rearBores);

frontDiameter = zeros(n, 1);
rearDiameter  = zeros(n, 1);
lockedAxle    = repmat("None", n, 1);

lockScore      = nan(n, 1);
otherAxleSpeed = nan(n, 1);
pedalAtLock    = nan(n, 1);
frontPressure = nan(n, 1);
rearPressure  = nan(n, 1);
vehicleSpeed  = nan(n, 1);

omegaFR = nan(n, 1);
omegaFL = nan(n, 1);
omegaRR = nan(n, 1);
omegaRL = nan(n, 1);

endReason = strings(n, 1);
row = 0;

%% Run all combinations
for f = 1:numel(frontBores)
    for r = 1:numel(rearBores)

        row = row + 1;

        car = baseCar;
        car.masterCylinder.front.diameter = frontBores(f);
        car.masterCylinder.rear.diameter  = rearBores(r);

        frontDiameter(row) = frontBores(f);
        rearDiameter(row)  = rearBores(r);

        sim = brakeSimulate(car, Fpedal, vInitial);
        endReason(row) = string(sim.endReason);

        if isempty(sim.history)
            continue
        end

        % First recorded wheel lock while the vehicle is moving
        eventIndex = find(arrayfun(@(s) ...
            s.speed > 0 && any([s.omega.FR, s.omega.FL, ...
                               s.omega.RR, s.omega.RL] <= 0), ...
            sim.history), 1, 'first');

        if isempty(eventIndex)
            continue
        end

        event = sim.history(eventIndex);

        frontLocked = event.omega.FR <= 0 || event.omega.FL <= 0;
        rearLocked  = event.omega.RR <= 0 || event.omega.RL <= 0;

        omegaFR(row) = event.omega.FR;
        omegaFL(row) = event.omega.FL;
        omegaRR(row) = event.omega.RR;
        omegaRL(row) = event.omega.RL;

        % Average angular speed of each axle [rad/s]
        frontOmega = mean([event.omega.FR, event.omega.FL]);
        rearOmega  = mean([event.omega.RR, event.omega.RL]);

        % Angular speed corresponding to free rolling [rad/s]
        rollingOmega = event.speed / car.vehicle.tireRadius;

        % Signed lock score:
        % Positive = front first
        % Negative = rear first
        % Zero = both axles lock in the same saved timestep
        if frontLocked && rearLocked
            lockedAxle(row) = "Both";
            lockScore(row) = 0;

        elseif frontLocked
            lockedAxle(row) = "Front";
            otherAxleSpeed(row) = rearOmega;
            lockScore(row) = rearOmega / rollingOmega;

        else
            lockedAxle(row) = "Rear";
            otherAxleSpeed(row) = frontOmega;
            lockScore(row) = -frontOmega / rollingOmega;
        end

        pedalAtLock(row) = event.Fpedal;

        frontPressure(row) = ...
            event.brakes.mechanical.frontLinePressure / 1e5;

        rearPressure(row) = ...
            event.brakes.mechanical.rearLinePressure / 1e5;

        vehicleSpeed(row) = event.speed;
    end
end

%% Full results table
results = table( ...
    frontDiameter, rearDiameter, lockedAxle, lockScore, ...
    otherAxleSpeed, pedalAtLock, frontPressure, rearPressure, ...
    vehicleSpeed, omegaFR, omegaFL, omegaRR, omegaRL, endReason, ...
    'VariableNames', { ...
    'FrontBore_m', 'RearBore_m', 'FirstLockedAxle', 'LockScore', ...
    'OtherAxle_rad_s', 'PedalAtLock_N', ...
    'FrontPressure_bar', 'RearPressure_bar', ...
    'VehicleSpeed_m_s', ...
    'FR_rad_s', 'FL_rad_s', 'RR_rad_s', 'RL_rad_s', ...
    'EndReason'});

% Exclude runs without a lockup score
validResults = results(~isnan(results.LockScore), :);

% Sort by distance from zero, preserving the sign
[~, order] = sort(abs(validResults.LockScore));

% Select up to 10 closest results
top10 = validResults(order(1:min(10, height(validResults))), :);

disp(top10(:, { ...
    'FrontBore_m', 'RearBore_m', 'FirstLockedAxle', ...
    'LockScore', 'PedalAtLock_N', ...
    'FrontPressure_bar', 'RearPressure_bar'}));

%% Display comparison
disp(results(:, { ...
    'FrontBore_m', 'RearBore_m', 'FirstLockedAxle', ...
    'LockScore', 'OtherAxle_rad_s', 'PedalAtLock_N', ...
    'FrontPressure_bar', 'RearPressure_bar'}));