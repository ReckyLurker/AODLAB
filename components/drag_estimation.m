% Trapezoidal model for Wing, Horizon Tail and Vertical Tail 
C_trapezoidal = @(x, C_root, TaperRatio, Span) C_root - (2*C_root*(1-TaperRatio))/(Span) * x;

%%% Fuselage 

% Estimates
Q_fuselage = 1; % Qf = 1 assuming no interference

% Design Choices 
FuselageLength = 44.51; % in m 
FuselageDiameter = 4; % in m 

% Computed Quantities 
FuselageFinenessRatio = FuselageLength/FuselageDiameter;

FuselageArea = pi * FuselageLength * FuselageDiameter;
FuselageWettedArea = FuselageArea * (1 - (2/FuselageFinenessRatio)^(2/3)) * (1 + (1/FuselageFinenessRatio)^2);
FuselageFormFactor = 1 + 60/FuselageFinenessRatio^3 + FuselageFinenessRatio/400;
CD0_fuselage_local = FuselageFormFactor*Q_fuselage*C_fe*(FuselageWettedArea/FuselageArea);

%%% Wing [NACA 23015 airfoil]

% Estimates 
Q_wing = 1; % Qw = 1 assuming no interference at wing

% Design Choices 
MaxThicknessRatio_wing = 0.1501;
MaxThicknessLocation_wing = 0.3;
Tau_wing = 1;

% Computed Quantities
MeanAerodynamicChord_wing = (2/3)*(1+TaperRatio_wing+TaperRatio_wing^2)/(1+TaperRatio_wing)*C_root_wing;
MaxThicknessLocation_MAC = MaxThicknessLocation_wing*MeanAerodynamicChord_wing;

% Wing Computations
C_root_exposed_wing = C_trapezoidal(FuselageDiameter/2, C_root_wing, TaperRatio_wing, Span_wing);
MaxThicknessSweep_wing = (1 - MaxThicknessLocation_wing)*LeadingEdgeSweep_wing + MaxThicknessLocation_wing*TrailingEdgeSweep_wing;
FormFactor_wing = (1 + 0.6*MaxThicknessRatio_wing/MaxThicknessLocation_MAC + 100*MaxThicknessRatio_wing^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep_wing))^0.28;
ExposedArea_wing = 0.5*(Span_wing - FuselageDiameter) * (C_tip_wing + C_root_exposed_wing);
WingWettedArea = 2*ExposedArea_wing*(1+0.25*MaxThicknessRatio_wing*(1+Tau_wing*TaperRatio_wing)/(1+TaperRatio_wing));

% Drag Estimation
CD0_wing_local = FormFactor_wing*Q_wing*C_fe*(WingWettedArea/PlanformArea_wing);
CDi_wing_local = 1/(pi*OswaldEfficiency*AspectRatio_wing);

%%% Horizontal Tail [NACA 0012]

% Design Choices 
MaxThicknessRatio_ht = 0.12;
MaxThicknessLocation_ht = 0.3; 
TaperRatio_ht = 0.4;
Incidence_ht = -2.5*deg2rad; % in rad 
C_root_ht = 3.18; % in m 
Span_ht = 8.92; % in m 
Tau_ht = 1;
LeadingEdgeSweep_ht = 30*deg2rad; % in rad

% Estimates 
Q_ht = 1.06; % Horizontal Tail Interference Factor

% Computed Quantities 
C_tip_ht = C_root_ht*TaperRatio_ht;
PlanformArea_ht = 0.5*Span_ht*(C_root_ht+C_tip_ht);
AspectRatio_ht = (Span_ht^2)/PlanformArea_ht;
WettedArea_ht = 2*PlanformArea_ht*(1+0.25*MaxThicknessRatio_ht*(1+Tau_ht*TaperRatio_ht)/(1+TaperRatio_ht));
MeanAerodynamicChord_ht = (2/3)*(1+TaperRatio_ht+TaperRatio_ht^2)/(1+TaperRatio_ht)*C_root_ht;
MaxThicknessLocation_MAC_ht = MaxThicknessLocation_ht*MeanAerodynamicChord_ht;
TrailingEdgeSweep_ht = atan2(C_tip_ht - C_root_ht + 0.5*Span_ht*tan(LeadingEdgeSweep_ht), 0.5*Span_ht);
MaxThicknessSweep_ht = (1 - MaxThicknessLocation_ht)*LeadingEdgeSweep_ht + MaxThicknessLocation_ht*TrailingEdgeSweep_ht;
FormFactor_ht = (1 + 0.6*MaxThicknessRatio_ht/MaxThicknessLocation_MAC_ht + 100*MaxThicknessLocation_ht^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep_ht)^(0.28));

CD0_ht_local = FormFactor_ht*Q_ht*C_fe*(WettedArea_ht/PlanformArea_ht);
CDi_ht_local = 1/(pi*OswaldEfficiency*AspectRatio_ht);

%%% Vertical Tail [NACA 0012]

% Design Choices 
MaxThicknessRatio_vt = 0.12;
MaxThicknessLocation_vt = 0.3;
TaperRatio_vt = 0.5;
C_root_vt = 4.12; % in m 
Span_vt = 5.27; % in m 
Tau_vt = 1;
LeadingEdgeSweep_vt = 45*deg2rad; % in rad;

% Estimates 
Q_vt = 1.06; % Interference of vertical tail

% Computed Quantities 
C_tip_vt = TaperRatio_vt*C_root_vt;
PlanformArea_vt = 0.5*Span_vt*(C_root_vt+C_tip_vt);
AspectRatio_vt = (Span_vt^2)/PlanformArea_vt;
WettedArea_vt = 2*PlanformArea_vt*(1+0.25*MaxThicknessLocation_vt*(1+Tau_vt*TaperRatio_vt)/(1+TaperRatio_vt));
MeanAerodynamicChord_vt = (2/3)*(1+TaperRatio_vt+TaperRatio_vt^2)/(1+TaperRatio_vt)*C_root_vt;
MaxThicknessLocation_MAC_vt = MaxThicknessLocation_vt*MeanAerodynamicChord_vt;
TrailingEdgeSweep_vt = atan2(C_tip_vt - C_root_vt + Span_vt*tan(LeadingEdgeSweep_vt), Span_vt);
MaxThicknessSweep_vt = (1 - MaxThicknessLocation_vt)*LeadingEdgeSweep_vt + MaxThicknessLocation_vt*TrailingEdgeSweep_vt;
FormFactor_vt = (1+0.6*MaxThicknessRatio_vt/MaxThicknessLocation_MAC_vt + 100*MaxThicknessRatio_vt^4)*(1.34*M_cruise^0.18)*(cos(MaxThicknessSweep_vt)^0.28);

CD0_vt_local = FormFactor_vt*Q_vt*C_fe*(WettedArea_vt/PlanformArea_vt);
CDi_vt_local = 1/(pi*OswaldEfficiency*AspectRatio_vt);

%%% Engine [Nacelle]

% Design Choices
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

% Estimates
Q_nacelle = 1.3; % Interference Factor for Nacelle 

% Computed Quantities
WettedArea_cowl = CowlLength*CowlDiameter*(2 + 0.35*(FanLocation/CowlLength) + 0.8*(FanLocation/CowlLength)*(CowlDiameter_inlet/CowlDiameter) + 1.15*(1 - (FanLocation/CowlLength))*(CowlDiameter_outlet/CowlDiameter));
WettedArea_gasgen = pi*GasGenLength*GasGenInletDiameter*(1 - (1/3)*(1 - GasGenOutletDiameter/GasGenInletDiameter)*(1 - 0.18*(GasGenInletDiameter/GasGenLength)^(5/3)));
WettedArea_plug = 0.7*pi*PlugLength*PlugDiameter;
WettedArea_nacelle = WettedArea_cowl + WettedArea_gasgen + WettedArea_plug;
FormFactor_nacelle = 1 + 0.35/(CowlLength/CowlDiameter);
CowlCurvedSurfaceArea = pi*CowlDiameter*CowlLength; % in m^2 
GasGenTheta = atan2(GasGenInletDiameter - GasGenOutletDiameter, 2*GasGenLength); % in rad 
GasGenCurvedSurfaceArea = 0.5*pi*cos(GasGenTheta)*(GasGenInletDiameter+GasGenOutletDiameter)*GasGenLength; % in m^2 
PlugTheta = atan2(PlugDiameter, 2*PlugLength);
PlugCurvedSurfaceArea = 0.5*pi*cos(PlugTheta)*PlugDiameter*PlugLength; % in m^2 
NacelleSurfaceArea = CowlCurvedSurfaceArea + GasGenCurvedSurfaceArea + PlugCurvedSurfaceArea; % in m^2 
CD0_nacelle_local = FormFactor_nacelle*Q_nacelle*C_fe*(WettedArea_nacelle/NacelleSurfaceArea);

%%% TOTAL DRAG 
CD0 = CD0_wing_local + CD0_ht_local*(PlanformArea_ht/PlanformArea_wing) + CD0_vt_local*(PlanformArea_vt/PlanformArea_wing) + CD0_fuselage_local*(FuselageArea/PlanformArea_wing) + CD0_nacelle_local*(NacelleSurfaceArea/PlanformArea_wing);
    
CDi = Q_wing*CDi_wing_local; % + Q_ht*CDi_ht_local*(PlanformArea_ht/PlanformArea_wing) + Q_vt*CDi_vt_local*(PlanformArea_vt/PlanformArea_wing);

frac_wing = CD0_wing_local / CD0*100;
frac_fuselage = CD0_fuselage_local*(FuselageArea/PlanformArea_wing)/CD0*100;
frac_nacelle = CD0_nacelle_local*(NacelleSurfaceArea/PlanformArea_wing)/CD0*100;
frac_ht = CD0_ht_local*(PlanformArea_ht/PlanformArea_wing)/CD0*100;
frac_vt = CD0_vt_local*(PlanformArea_vt/PlanformArea_wing)/CD0*100;

fprintf('Drag Estimation:\n');
fprintf('(1) Wing: CD0 = %.6f CDi = %.6f (%.3f %%)\n', CD0_wing_local, CDi_wing_local, frac_wing);
fprintf('(2) Horizontal Tail: CD0 = %.6f CDi = %.6f ( %.3f %%)\n', CD0_ht_local*(PlanformArea_ht/PlanformArea_wing), CDi_ht_local*(PlanformArea_ht/PlanformArea_wing), frac_ht);
fprintf('(3) Vertical Tail: CD0 = %.6f CDi = %.6f (%.3f %%)\n', CD0_vt_local*(PlanformArea_vt/PlanformArea_wing), CDi_vt_local*(PlanformArea_vt/PlanformArea_wing), frac_vt);
fprintf('(4) Fuselage: CD0 = %.6f (%.3f %%)\n', CD0_fuselage_local*(FuselageArea/PlanformArea_wing), frac_fuselage);
fprintf('(5) Nacelle: CD0 = %.6f (%.3f %%)\n', CD0_nacelle_local*(NacelleSurfaceArea/PlanformArea_wing), frac_nacelle);
fprintf('Total: CD0 = %.6f CDi = %.3f\n', CD0, CDi);
fprintf('Induced Drag Contribution of HT and VT neglected.\n\n');