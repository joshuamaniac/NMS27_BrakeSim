% Version Record: v2 Kai 10/20/2026 
% What is changed: the scripted is convert into a function, 
function [V, a, Fx_tires, Nf, Nr, DF, Drag, dW] = brakingStep(...
    V, dt, CL, CD, A_ref, A_front, aeroBalFront, ...
    staticFront, staticRear, mu, brakeForceFront, brakeForceRear, ...
    Fx_tires, h, h_drag, L, m)

% Inputs:
%   V               - Current velocity [m/s]
%   dt              - Timestep size [s]
%   CL              - Downforce coefficient
%   CD              - Drag coefficient
%   A_ref           - Reference aerodynamic downforce area [m^2]
%   A_front         - Frontal cross-sectional area [m^2]
%   aeroBalFront    - Aerodynamic balance distribution to front axle [fraction, 0-1]
%   staticFront     - Static normal load per front wheel [N]
%   staticRear      - Static normal load per rear wheel [N]
%   mu              - Tire-road friction coefficient
%   brakeForceFront - Demanded/applied brake force at front axle [N]
%   brakeForceRear  - Demanded/applied brake force at rear axle [N]
%   Fx_tires        - Total tire braking force from previous timestep [N]
%   h               - Center of gravity (CG) height [m]
%   h_drag          - Height of aerodynamic drag center of pressure [m]
%   L               - Wheelbase length [m]
%   m               - Total vehicle mass [kg]

% Outputs:
%   V               - Updated velocity at next timestep [m/s]
%   a               - Vehicle deceleration [m/s^2]
%   Fx_tires        - Total tire braking force achieved this timestep [N]
%   Nf              - Total front normal force [N]
%   Nr              - Total rear normal force [N]
%   DF              - Total aerodynamic downforce [N]
%   Drag            - Aerodynamic drag force [N]
%   dW              - Dynamic longitudinal load transfer [N]

    rho = 1.225;     % air density
    q    = 0.5*rho*V^2;  
    DF= q*CL*A_ref;  
    DF_front = DF*aeroBalFront;  
    DF_rear = DF - DF_front;         % Recalculate downforce instead of DF/4
    Drag = q*CD*A_front;                            
    dW = (Fx_tires*h + Drag*(h - h_drag)) / L;     
    % dW = load transfer from rear to front (positive shifts load to front)
    Nf = 2*staticFront + DF_front + dW;  % Normal front
    Nr = 2*staticRear  + DF_rear - dW;  % Normal Rear
    Fx_f = min(mu*Nf, brakeForceFront);            % Brake Force front
    Fx_r = min(mu*Nr, brakeForceRear);             % Brake Force rear
    Fx_tires = Fx_f + Fx_r;                        
    a = (Fx_tires + Drag) / m;                     
    V = V - a*dt;          
end