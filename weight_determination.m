% Weight Fractions
W_takeoff = 0.97;
W_climb = 0.985;
W_cruise = exp( - (Range * SpecificFuelConsumption_Cruise) / (v_cruise * LD_cruise_est));
W_loiter = exp( - (Loiter_time * SpecificFuelConsumption_Loiter) / (LD_max_est)); 
W_land = 0.995; 

W_mission_total = W_takeoff * W_climb * W_cruise * W_loiter * W_land; 

WfW0 = (1 + ReserveFuelFraction) * (1 - W_mission_total);
fprintf('Fuel Fraction: %.6f\n', WfW0);

W_crew = 7 * 100; % in kg  
W_payload = 23500; % in kg 

W_empty_model = @(A, C, K_vs, W0) A * W0^C * K_vs;

K_vs = 1; % for fixed sweep 
A = 1.02; % for jet transport 
C = -0.06; % for jet transport 

% Maximum Takeoff Weight Estimation 

takeoff_eqn = @(W0) W0 - WfW0 * W0 - W_empty_model(A, C, K_vs, W0) * W0 - W_crew - W_payload;

% 'Display' = 'iter-detailed' for detailed iteration statistics, 'off' for
% no printing 
opts = optimoptions('fsolve', 'Display', 'off');
W0_initial_guess = 100000; % in kg 

MaximumTakeoffWeight_est = fsolve(takeoff_eqn, W0_initial_guess, opts); % in kg 

fprintf('Maximum Takeoff Weight: %.6f kg\n', MaximumTakeoffWeight_est);

WeW0 = W_empty_model(A, C, K_vs, MaximumTakeoffWeight_est);
