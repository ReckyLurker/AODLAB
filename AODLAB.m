clear all; 
clc; 

addpath('components/')

% Conversion 
m2ft = 3.28084;
ft2m = 1/m2ft;
km2m = 1000;
min2sec = 60; 
hr2min = 60;
sec2min = 1 / min2sec;
sec2hr = 1 / ( hr2min * min2sec);
deg2rad = pi / 180; 
rad2deg = 180 / pi; 
ms2kmhr = 18/5;
kmhr2ms = 5/18;
knot2ms = 0.514444;
ms2knot = 1/knot2ms;
lb2kg = 0.453592;
kg2lb = 1/lb2kg;

% Constants 
g = 9.81; % in m/s^2 
OswaldEfficiency = 0.8;
v_sound = 981.6273 * ft2m; % (in m/s) [at 33000 ft]
rho_0ft = 1.225;
C_fe = 0.003; % skin friction coefficient of flat plate

% Design Parameters 
v_cruise = 795.1181 * ft2m; % (in m/s)
LoiterTime = 30 * min2sec; % (in sec)
ReserveFuelFraction = 0.06; % percentage of extra fuel
Range = 5000 * km2m; % (in m) 
Altitude = 33000 * ft2m; % (in m)
SpecificFuelConsumption_Cruise = 0.5 * sec2hr; % (in (kg/sec)/ kg for a high bypass turbofan engine)
SpecificFuelConsumption_Loiter = 0.4 * sec2hr; % (in (kg/sec)/ kg for a high bypass turbofan engine)

% Estimates 
LD_max_est = 18; % (L/D)_max_estimated 
LD_cruise_est = LD_max_est * 0.866;

% Useful Quantities 
M_cruise = v_cruise / v_sound;

% Individual Parts 
weight_estimation;
stall_speed;
drag_estimation;

% Thrust/Weight Ratio
TW_cruise = 1/LD_cruise_est;

fprintf('\n(T/W)_cruise = %.6f\n', TW_cruise);

TW_takeoff = W_cruise*TW_cruise/0.17;   

fprintf('(T/W)_takeoff = %.6f\n', TW_takeoff);

% Crude Estimation of CL_max [With Flaps]
CL_max_flapped = 2.0;
AreaRatioUnflapped = 0.65; % S_unflapped / S_planform
AreaRatioFlapped = 0.35; % S_flapped / S_planform 

CL_max_with_flaps = CL_max_no_flaps*AreaRatioUnflapped + 0.9*(CL_max_flapped*AreaRatioFlapped);
v_stall_with_flaps = sqrt((2*MaximumTakeoffWeight*g)/(rho_0ft*CL_max_with_flaps*PlanformArea_wing));

fprintf('Stall Speed with flaps: %.6f\n', v_stall_with_flaps);

v_approach = 1.3*v_stall_no_flaps;
v_approach_with_flaps = 1.3*v_stall_with_flaps;

fprintf('Approach Speed: %.6f\n', v_approach);
fprintf('Approach Speed with flaps: %.6f\n', v_approach_with_flaps);

WS_stall_speed_with_flaps = 0.5*rho_0ft*CL_max_with_flaps*v_stall_with_flaps^2;
WS_stall_speed = 0.5*rho_0ft*CL_max_no_flaps*v_stall_no_flaps^2;

fprintf('\nW/S Estimation:\n');
fprintf('(1) W/S from stall speed with flaps: %.6f N/m^2 = %.6f kg/m^2\n', WS_stall_speed_with_flaps, WS_stall_speed_with_flaps/g);
fprintf('(2) W/S from stall speed without flaps: %.6f N/m^2 = %.6f kg/m^2\n', WS_stall_speed, WS_stall_speed/g);

rho_33000ft = 0.3345*rho_0ft;
WS_cruise = 0.5*rho_33000ft*(v_cruise^2)*sqrt((pi/3)*AspectRatio_wing*OswaldEfficiency*CD0);

fprintf('(3) W/S for cruise: %.6f N/m^2 = %.6f kg/m^2\n', WS_cruise/W20, WS_cruise/(g*W20));

CL_takeoff = CL_max_with_flaps / (1.1*1.1);
V_liftoff = 1.1*v_stall_with_flaps;
TOP = 374;

WS_takeoff = TOP * CL_takeoff * TW_takeoff * lb2kg/(ft2m^2) * g;
fprintf('(4) W/S for takeoff: %.6f N/m^2 = %.6f kg/m^2\n', WS_takeoff, WS_takeoff/g);

rho_loiter = 0.4481*rho_0ft;
WeightLoiter = W_takeoff*W_climb*W_cruise*W_loiter*MaximumTakeoffWeight;
v_loiter = ((4*CDi*(WeightLoiter*g)^2)/(rho_loiter^2*CD0*PlanformArea_wing^2))^0.25;
WS_loiter = 0.5*rho_loiter*(v_loiter^2)*sqrt(pi*AspectRatio_wing*OswaldEfficiency*CD0);
fprintf('(5) W/S for loiter: %.6f N/m^2 = %.6f kg/m^2\n', WS_loiter/(W30), WS_loiter/(g*W30));
