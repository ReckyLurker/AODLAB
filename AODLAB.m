clear all; 
clc; 

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
Range = 5000 * km2m * m2ft; % (in feet) 
Altitude = 39000; % (in feet)
SpecificFuelConsumption_Cruise = 0.5 * sec2hr; % (in (kg/sec)/ kg for a high bypass turbofan engine)
SpecificFuelConsumption_Loiter = 0.4 * sec2hr; % (in (kg/sec)/ kg for a high bypass turbofan engine)
v_sound = 968.1; % (in feet/sec)
v_cruise = 755.1; % (in feet / sec)
M_cruise = v_cruise / v_sound;
Loiter_time = 30 * min2sec; % (in sec)
LD_max_est = 18; % (L/D)_max_estimated 
LD_cruise_est = LD_max_est * 0.866;
ReserveFuelFraction = 0.06; % percentage of extra fuel
g = 9.81; % in m/s^2 
OswaldEfficiency = 0.8;


% Stall Speed 
LeadingEdgeSweep_wing = 25 * deg2rad; % in rad 

C_root_wing = 6; % in m 
TaperRatio = 0.2;
C_tip_wing = TaperRatio * C_root_wing; % in m  
WingSpan = 34.2; % in m 
PlanformArea_wing = 0.5 * WingSpan * (C_tip_wing + C_root_wing); % in m^2 

AspectRatio = (WingSpan*WingSpan)/PlanformArea_wing; 

TrailingEdgeSweep_wing = atan2(C_tip_wing - C_root_wing+ 0.5*WingSpan*tan(LeadingEdgeSweep_wing), 0.5*WingSpan); 

QuarterEdgeSweep_wing = 0.75 * LeadingEdgeSweep_wing + 0.25 * TrailingEdgeSweep_wing;

fprintf('Quarter Edge Sweep for wing: %.6f degrees\n', QuarterEdgeSweep_wing * rad2deg);

CL_max = 1.6;
CL_alpha = 0.1;
Alpha_zero_lift = -1 * deg2rad; % in rad 
Alpha_stall = 15 * deg2rad; % in rad 
 
Rho_sealevel = 1.225; % in kg/m^3

v_stall = sqrt((2 * MaximumTakeoffWeight_est * g) / (Rho_sealevel * 0.9*CL_max * PlanformArea_wing));

fprintf('Stall Speed: %.6f m/s\n', v_stall);

% Drag Estimation

% Fuselage 
FuselageLength = 44.51; % in m 
FuselageDiameter = 4; % in m 
FuselageFinenessRatio = FuselageLength / FuselageDiameter; 

C_fe = 0.003; % skin friction coefficient of flat plate 
Q_fuselage = 1; % Assuming no interference on fuselage 

FuselageArea = pi * FuselageLength * FuselageDiameter; % Curved Surface Area of Fuselage 
S_wet_fuselage = FuselageArea * (1 - (2/FuselageFinenessRatio))^(2/3) * (1 + 1/FuselageFinenessRatio^2);

FuselageFormFactor = 1 + 60/FuselageFinenessRatio^3 + FuselageFinenessRatio/400;

CD0_fuselage_local = FuselageFormFactor * Q_fuselage * C_fe * (S_wet_fuselage / FuselageArea);

% Wing [Airfoil - NACA 23015]
Q_wing = 1; % no interference in wing 
MaxThicknessRatio = 0.1501; % (t/c)_max
MaxThicknessLocation = 0.3;

MeanAerodynamicChord = (2/3) * (1 + TaperRatio + TaperRatio^2)/(1 + TaperRatio) * C_root_wing; 
MaxThicknessLocation_MAC = MaxThicknessLocation * MeanAerodynamicChord;

C_trapezoidal_model = @(x) C_root_wing - 2*(C_root_wing - C_tip_wing)/(WingSpan)*x; 
C_root_exposed = C_trapezoidal_model(FuselageDiameter/2); % Chord length exposed 

MaxThicknessSweep = (1 - MaxThicknessLocation)*LeadingEdgeSweep_wing + MaxThicknessLocation*TrailingEdgeSweep_wing;
FormFactorWing = (1 + 0.6*MaxThicknessRatio/MaxThicknessLocation_MAC + 100*MaxThicknessRatio^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep))^(0.28);

ExposedArea_wing = 0.5 * (WingSpan - FuselageDiameter) * (C_tip_wing + C_root_exposed);

Tau = 1; % (t/c)_root / (t/c)_tip Uniform Airfoil over wing 

S_wet_wing = 2*ExposedArea_wing*(1 + 0.25*MaxThicknessRatio*(1 + Tau*TaperRatio)/(1+TaperRatio));

CD0_wing_local = FormFactorWing*Q_wing*C_fe*(S_wet_wing/PlanformArea_wing);
CDi_wing_local = 1/(pi*OswaldEfficiency*AspectRatio);

% Horizon Tail [NACA 0012]
MaxThicknessRatio_ht = 0.12;
MaxThicknessLocation_ht = 0.3;
TaperRatio_ht = 0.4;
Incidence_ht = -2.5 * deg2rad;  

Q_ht = 1.06; % Interference Factor 
C_root_ht = 3.18;
C_tip_ht = C_root_ht * TaperRatio_ht;
WingSpan_ht = 8.92;

PlanformArea_ht = 0.5*WingSpan_ht*(C_root_ht + C_tip_ht);
AspectRatio_ht = (WingSpan_ht*WingSpan_ht)/PlanformArea_ht;

C_trapezoidal_model = @(x) C_root_ht - 2*(C_root_ht - C_tip_ht)/WingSpan_ht * x;
C_root_exposed_ht = C_trapezoidal_model(FuselageDiameter/2);
ExposedArea_ht = 0.5*(WingSpan_ht-FuselageDiameter)*(C_root_exposed_ht + C_tip_ht);

Tau_ht = 1;
S_wet_ht = 2*ExposedArea_ht*(1+0.25*MaxThicknessRatio_ht*(1+Tau_ht*TaperRatio_ht)/(1+TaperRatio_ht));

FormFactorWing = (1 + 0.6*MaxThicknessRatio/MaxThicknessLocation_MAC + 100*MaxThicknessRatio^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep))^(0.28);

MeanAerodynamicChord_ht = (2/3)*(1+TaperRatio_ht+TaperRatio_ht^2)/(1+TaperRatio_ht);
MaxThicknessLocation_MAC_ht = MaxThicknessLocation_ht * MeanAerodynamicChord_ht;

LeadingEdgeSweep_ht = 30 * deg2rad; % in rad 
TrailingEdgeSweep_ht = atan2(C_tip_ht - C_root_ht + 0.5*WingSpan_ht*tan(LeadingEdgeSweep_ht), 0.5*WingSpan_ht);
MaxThicknessSweep_ht = (1 - MaxThicknessLocation_ht)*LeadingEdgeSweep_ht + MaxThicknessLocation_ht*TrailingEdgeSweep_ht;
FormFactorHT = (1 + 0.6*MaxThicknessRatio_ht/MaxThicknessLocation_MAC_ht + 100*MaxThicknessRatio_ht^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep_ht))^(0.28);

CD0_ht_local = FormFactorHT*Q_ht*C_fe*(S_wet_ht/ExposedArea_ht);
CDi_ht_local = 1/(pi*OswaldEfficiency*AspectRatio_ht);

% Vertical Tail [NACA 0012]
MaxThicknessRatio_vt = 0.12;
MaxThicknessLocation_vt = 0.3;
TaperRatio_vt = 0.5;

Q_vt = 1.06; % Interference Factor 
C_root_vt = 4.12;
C_tip_vt = TaperRatio_vt * C_root_vt;
WingSpan_vt = 5.27;

PlanformArea_vt = 0.5*WingSpan_vt*(C_root_vt + C_tip_vt);
AspectRatio_vt = (WingSpan_vt*WingSpan_vt)/PlanformArea_vt;


C_trapezoidal_model = @(x) C_root_vt - 2*(C_root_vt - C_tip_vt)/WingSpan_vt * x;
C_root_exposed_vt = C_trapezoidal_model(FuselageDiameter/2);
ExposedArea_vt = 0.5 * (WingSpan_vt - FuselageDiameter) * (C_root_exposed_vt + C_tip_vt);
Tau_vt = 1;
S_wet_vt = 2*ExposedArea_vt*(1+0.25*MaxThicknessRatio_vt*(1+Tau_vt*TaperRatio_vt)/(1+TaperRatio_vt));


MeanAerodynamicChord_vt = (2/3)*(1+TaperRatio_vt+TaperRatio_vt^2)/(1+TaperRatio_vt);
MaxThicknessLocation_MAC_vt = MaxThicknessLocation_vt * MeanAerodynamicChord_vt;

LeadingEdgeSweep_vt = 45 * deg2rad; % in rad 
TrailingEdgeSweep_vt = atan2(C_tip_vt - C_root_vt + 0.5*WingSpan_vt*tan(LeadingEdgeSweep_vt), 0.5*WingSpan_vt);
MaxThicknessSweep_vt = (1 - MaxThicknessLocation_vt)*LeadingEdgeSweep_vt + MaxThicknessLocation_vt*TrailingEdgeSweep_vt;
FormFactorVT = (1 + 0.6*MaxThicknessRatio_vt/MaxThicknessLocation_MAC_vt + 100*MaxThicknessRatio_vt^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep_vt))^(0.28);

CD0_vt_local = FormFactorVT*Q_vt*C_fe*(S_wet_vt/ExposedArea_vt);
CDi_vt_local = 1/(pi*OswaldEfficiency*AspectRatio_vt);

% Engine (Nacelle) 
CowlDiameter = 2.6; % in m (Dn) 
CowlDiameter_inlet = 2.52; % in m (Dhl) 
CowlLength = 2.25; % in m (Ln)
CowlDiameter_outlet = 1.98; % in m (Def)
FanLocation = 0.67; % in m (l1) 

GasGenLength = 1.98; % in m (lg)
GasGenOutletDiameter = 1.64; % in m (Deg)
GasGenInletDiameter = 1.93; % in m (Dg)

PlugLength = 1.21; % in m (lp) 
PlugDiameter = 1.24; % in m (Dp) 

S_wet_cowl = CowlLength*CowlDiameter*(2 + 0.35*(FanLocation/CowlLength) + ...
                                        0.8*(FanLocation/CowlLength)*(CowlDiameter_inlet/CowlDiameter) + ...
                                        1.15*(1 - (FanLocation/CowlLength))*(CowlDiameter_outlet/CowlDiameter));
S_wet_gas_gen = pi*GasGenLength*GasGenInletDiameter*(1 - (1/3)*(1 - GasGenOutletDiameter/GasGenInletDiameter)*(1 - 0.18*(GasGenInletDiameter/GasGenLength)^(5/3)));
S_wet_plug = 0.7*pi*PlugLength*PlugDiameter;

S_wet_nacelle = S_wet_cowl + S_wet_plug + S_wet_gas_gen;

FormFactorNacelle = 1 + 0.35/(CowlLength/CowlDiameter);
Q_nacelle = 1.5; % Interference Factor for Nacelle 


CowlCurvedSurfaceArea = pi*CowlDiameter*CowlLength; % in m^2 

GasGenTheta = atan2(GasGenInletDiameter - GasGenOutletDiameter, 2*GasGenLength); % in rad 

GasGenCurvedSurfaceArea = 0.5*pi*cos(GasGenTheta)*(GasGenInletDiameter+GasGenOutletDiameter)*GasGenLength; % in m^2 

PlugTheta = atan2(PlugDiameter, 2*PlugLength);
PlugCurvedSurfaceArea = 0.5*pi*cos(PlugTheta)*PlugDiameter*PlugLength; % in m^2 
                                            
NacelleSurfaceArea = CowlCurvedSurfaceArea + GasGenCurvedSurfaceArea + PlugCurvedSurfaceArea;

CD0_Nacelle_local = FormFactorNacelle*Q_nacelle*C_fe*(S_wet_nacelle/NacelleSurfaceArea);


% TOTAL DRAG % 
CD0 = CD0_wing_local + CD0_ht_local*(ExposedArea_ht/PlanformArea_wing) + CD0_vt_local*(ExposedArea_vt/PlanformArea_wing) + ...
        CD0_fuselage_local*(FuselageArea/PlanformArea_wing) + CD0_Nacelle_local*(NacelleSurfaceArea/PlanformArea_wing);
    
CDi = Q_wing*CDi_wing_local + Q_ht*CDi_ht_local*(ExposedArea_ht/PlanformArea_wing) + Q_vt*CDi_vt_local*(ExposedArea_vt/PlanformArea_wing);

fprintf('\nDrag Estimation:\n');
fprintf('(1) Wing: CD0 = %.6f CDi = %.6f\n', CD0_wing_local, CDi_wing_local);
fprintf('(2) Horizontal Tail: CD0 = %.6f CDi = %.6f\n', CD0_ht_local*(ExposedArea_ht/PlanformArea_wing), CDi_ht_local*(PlanformArea_ht/PlanformArea_wing));
fprintf('(3) Vertical Tail: CD0 = %.6f CDi = %.6f\n', CD0_vt_local*(ExposedArea_vt/PlanformArea_wing), CDi_vt_local*(PlanformArea_vt/PlanformArea_wing));
fprintf('(4) Fuselage: CD0 = %.6f\n', CD0_fuselage_local*(FuselageArea/PlanformArea_wing));
fprintf('(5) Nacelle: CD0 = %.6f\n', CD0_Nacelle_local*(NacelleSurfaceArea/PlanformArea_wing));
fprintf('Total: CD0 = %.6f CDi = %.6f\n', CD0, CDi);

% Thrust/Weight Ratio
TW_cruise = 1/LD_cruise_est;

fprintf('\n(T/W)_cruise = %.6f\n', TW_cruise);

TW_takeoff = W_cruise*TW_cruise/0.17;   

fprintf('(T/W)_takeoff = %.6f\n', TW_takeoff);

% Crude Estimation of CL_max [With Flaps]
CL_max_flapped = 2.0;
AreaRatioUnflapped = 0.65; % S_unflapped / S_planform
AreaRatioFlapped = 0.35; % S_flapped / S_planform 

CL_max_with_flaps = 0.9*(CL_max*AreaRatioUnflapped + CL_max_flapped*AreaRatioFlapped);
v_stall_with_flaps = sqrt((2*MaximumTakeoffWeight_est*g)/(Rho_sealevel*CL_max_with_flaps*PlanformArea_wing));

fprintf('Stall Speed with flaps: %.6f\n', v_stall_with_flaps);

v_approach = 1.3*v_stall;
v_approach_with_flaps = 1.3*v_stall_with_flaps;

fprintf('Approach Speed: %.6f\n', v_approach);
fprintf('Approach Speed with flaps: %.6f\n', v_approach_with_flaps);

WS_stall_speed_with_flaps = 0.5*Rho_sealevel*CL_max_with_flaps*v_stall_with_flaps^2;
WS_stall_speed = 0.5*Rho_sealevel*0.9*CL_max*v_stall^2;

fprintf('\nW/S Estimation:\n');
fprintf('(1) W/S from stall speed with flaps: %.6f N/m^2 = %.6f kg/m^2\n', WS_stall_speed_with_flaps, WS_stall_speed_with_flaps/g);
fprintf('(2) W/S from stall speed without flaps: %.6f N/m^2 = %.6f kg/m^2\n', WS_stall_speed, WS_stall_speed/g);

rho_39000ft = 0.3164;
WS_cruise = 0.5*rho_39000ft*(v_cruise*ft2m)^2*sqrt((pi/3)*AspectRatio*OswaldEfficiency*CD0);

fprintf('(3) W/S for cruise: %.6f N/m^2 = %.6f kg/m^2\n', WS_cruise, WS_cruise/g);

CL_takeoff = CL_max_with_flaps / (1.1*1.1);
V_liftoff = 1.1*v_stall_with_flaps;
TOP = 400;

WS_takeoff = TOP * CL_takeoff * TW_takeoff * lb2kg/(ft2m^2) * g;
fprintf('(4) W/S for takeoff: %.6f N/m^2 = %.6f kg/m^2\n', WS_takeoff, WS_takeoff/g);

rho_loiter = 0.46;
WeightLoiter = W_takeoff*W_climb*W_cruise*W_loiter*MaximumTakeoffWeight_est;
v_loiter = ((4*CDi*(WeightLoiter*g)^2)/(rho_loiter^2*CD0*PlanformArea_wing^2))^0.25;
WS_loiter = 0.5*rho_loiter*(v_loiter^2)*sqrt(pi*AspectRatio*OswaldEfficiency*CD0);
fprintf('(5) W/S for loiter: %.6f N/m^2 = %.6f kg/m^2\n', WS_loiter, WS_loiter/g);


