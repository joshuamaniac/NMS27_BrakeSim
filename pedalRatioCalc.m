% Pedal Ratio
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Calculates the pedal ratio as it's angle changes

% Edited 10/1/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Change History 
%% Code
car = carspecs()
pedal = car.pedal

function pedalRatioCalculator = pedalRatioCalculator (pedal, angle)
    L = pedal.heightForce
    b = pedal.heightBalanceBar
    h = pedal.heightMasterCylinder
    m = pedal.length


    pedalRatio = L ./ (b .* cosd (arctand((bcos(angle) - h) ./ (m - bsin(angle)) - angle )) )

    results.pedalRatio = pedalRatio

end
    
pedalRatioCalculator(pedal, 0)