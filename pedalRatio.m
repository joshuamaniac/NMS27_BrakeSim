% Pedal Ratio
% Nittany Motorsports
% CBS, SBR Fall 2026
% Lead: Ryan Baker
% PM: Taylor Markl
%______________________________________________________________________
% Calculates the pedal ratio as it's angle changes

% Edited 10/2/2026 by Joshua Todd
% Addition: 
% ------------------------------------------------------------------------
%% Change History 
%% Code
function PR = pedalRatio(pedal, angle)
    L = pedal.heightForce;
    b = pedal.heightBalanceBar;
    h = pedal.heightMasterCylinder;
    m = pedal.length;

    phi = atan2d(b .* cosd(angle) - h, m - b .* sind(angle));

    PR = L ./ (b .* abs(cosd(phi - angle)));
end
    
