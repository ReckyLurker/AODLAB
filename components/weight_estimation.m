% Design Choices 
W_crew = 7 * 100; % in kg  
W_payload = 22800; % in kg 

% Empty Weight Fraction Model 
W_empty_model = @(A, C, K_vs, W0) A * W0^C * K_vs; % We/W0 model

% Design Choices 
K_vs = 1; % for fixed sweep wing 
A = 1.02; % for jet transport 
C = -0.06; % for jet transport 

% Weight Fractions [Esimates]
W_takeoff = 0.97; % Estimate 
W_climb = 0.985; % Estimate 
W_land = 0.995; % Estimate 

% Weight Fractions [Computed]
W_cruise = exp( - (Range * SpecificFuelConsumption_Cruise) / (v_cruise * LD_cruise_est));
W_loiter = exp( - (LoiterTime * SpecificFuelConsumption_Loiter) / (LD_max_est)); 

% w.r.t takeoff weight 
W10 = W_takeoff;
W20 = W10 * W_climb;
W30 = W20 * W_cruise;
W40 = W30 * W_loiter;
W50 = W40 * W_land;

WfW0 = (1 + ReserveFuelFraction) * (1 - W50);

% Maximum Takeoff Weight Estimation 

takeoff_eqn = @(W0) W0 - WfW0 * W0 - W_empty_model(A, C, K_vs, W0) * W0 - W_crew - W_payload;

% 'Display' = 'iter-detailed' for detailed iteration statistics, 'off' for no printing
opts = optimoptions('fsolve', 'Display', 'off');
W0_initial_guess = 100000; % in kg 

MaximumTakeoffWeight = fsolve(takeoff_eqn, W0_initial_guess, opts); % in kg 
fprintf('Weight Estimation:\n');
fprintf('(1) Maximum Takeoff Weight (MTW): %.6f kg\n', MaximumTakeoffWeight);

WeW0 = W_empty_model(A, C, K_vs, MaximumTakeoffWeight);
fprintf('(2) Operating Empty Weight Fraction: %.6f\n', WeW0);
fprintf('(3) Operating Empty Weight (OEW): %.6f kg\n', WeW0*MaximumTakeoffWeight);
fprintf('(4) Fuel Fraction: %.6f\n', WfW0);
fprintf('(5) Fuel Weight: %.6f kg\n\n', WfW0*MaximumTakeoffWeight);