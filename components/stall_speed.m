% Design Choices [Geometry]
LeadingEdgeSweep_wing = 25 * deg2rad; % in rad 
TaperRatio_wing = 0.2;
C_root_wing = 6; % in m 
Span_wing = 34.2; % in m 

% Computed Quantities 
C_tip_wing = TaperRatio_wing * C_root_wing;
PlanformArea_wing = 0.5 * Span_wing * (C_root_wing + C_tip_wing);
AspectRatio_wing = (Span_wing^2) / PlanformArea_wing;
TrailingEdgeSweep_wing = atan2(C_tip_wing - C_root_wing + 0.5 * Span_wing * tan(LeadingEdgeSweep_wing), 0.5*Span_wing); % in rad 

fprintf('Geometry Estimation:\n');
fprintf('(1) Wing C_root: %.6f m\n', C_root_wing);
fprintf('(2) Wing C_tip: %.6f m\n', C_tip_wing);
fprintf('(3) Wing Planform Area: %.6f m^2\n', PlanformArea_wing);
fprintf('(4) Aspect Ratio: %.6f\n', AspectRatio_wing);
fprintf('(5) Wing Leading Edge Sweep: %.6f rad = %.6f deg\n', LeadingEdgeSweep_wing, LeadingEdgeSweep_wing*rad2deg);
fprintf('(6) Wing Trailing Edge Sweep: %.6f rad = %.6f deg\n', TrailingEdgeSweep_wing, TrailingEdgeSweep_wing*rad2deg);

QuarterEdgeSweep_wing = 0.75*LeadingEdgeSweep_wing + 0.25*TrailingEdgeSweep_wing;

fprintf('(7) Wing Quarter Edge Sweep: %.6f rad = %.6f deg\n\n', QuarterEdgeSweep_wing, QuarterEdgeSweep_wing*rad2deg);

% Airfoil Design [NACA 23015]
Cl_max = 1.6;
Cl_alpha = 0.1; % dCl/d alpha 
alpha_0 = -1 * deg2rad; % in rad
alpha_stall = 15 * deg2rad; % in rad 

fprintf('Stall Speed Esimation [without flaps]:\n')

CL_max_no_flaps = 0.9*Cl_max;
v_stall_no_flaps = sqrt((2*MaximumTakeoffWeight*g)/(rho_0ft * CL_max_no_flaps * PlanformArea_wing));

fprintf('(1) Stall Speed without flaps: %.6f m/s\n\n', v_stall_no_flaps);