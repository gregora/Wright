% Main_simple
clear all
close all

Model_name = 'Model_aircraft_simple';
Total_mass = 0.573;
Tot_mom_inertia = diag([0.0494,0.0583,0.1012]);

% Lookup tables

Alpha_param_vector_NACA2415 = (-10:1:10)*pi/180;

CL_param_vector_NACA2415 = [ ...
   -0.8149, -0.8321, -0.7840, -0.7190, -0.6428, ...
   -0.5654, -0.4003, -0.2362, -0.0978,  0.0426, ...
    0.2682,  0.4986,  0.5740,  0.6560,  0.7416, ...
    0.8285,  0.9164,  0.9954,  1.0672,  1.1188, ...
    1.1582];

CD_param_vector_NACA2415 = [ ...
    0.04936, 0.03894, 0.03255, 0.02786, 0.02500, ...
    0.02336, 0.02238, 0.02115, 0.02007, 0.01951, ...
    0.01916, 0.01762, 0.01743, 0.01763, 0.01839, ...
    0.01933, 0.02028, 0.02149, 0.02292, 0.02518, ...
    0.02903];


Alpha_param_vector_NACA0006 = (-7:1:7)*pi/180;

CL_param_vector_NACA0006 = [ ...
   -0.6891, -0.6320, -0.5404, -0.4379, -0.3625, ...
   -0.1733, -0.0859,  0.0000,  0.0859,  0.1734, ...
    0.3624,  0.4379,  0.5404,  0.6320,  0.6893];

CD_param_vector_NACA0006 = [ ...
    0.06863, 0.04340, 0.02585, 0.01784, 0.01103, ...
    0.01079, 0.01026, 0.01012, 0.01026, 0.01078, ...
    0.01103, 0.01784, 0.02585, 0.04340, 0.06864];

% Surface positions

xyz_wing = [-0.0207, 0, -0.016];
S_wing = 0.2196;
Alpha_0_wing = 0;
Alpha_param_vector_wing = Alpha_param_vector_NACA2415;
CL_param_vector_wing = CL_param_vector_NACA2415;
CD_param_vector_wing = CD_param_vector_NACA2415;

xyz_aileron_L = [ -0.0207 -0.45 -0.016];
S__aileron_L = 0.0125/2;
Alpha_0_aileron_L = 0;
Alpha_param_vector_aileron_L = Alpha_param_vector_NACA0006;
CL_param_vector_aileron_L = CL_param_vector_NACA0006;
CD_param_vector_aileron_L = CD_param_vector_NACA0006;

xyz_aileron_R = [ -0.0207 0.45 -0.016];
S__aileron_R = 0.0125/2;
Alpha_0_aileron_R =  0;
Alpha_param_vector_aileron_R =  Alpha_param_vector_NACA0006;
CL_param_vector_aileron_R = CL_param_vector_NACA0006;
CD_param_vector_aileron_R = CD_param_vector_NACA0006;


xyz_horiz_stabilizer = [-0.491 0, -0.016];
S_horiz_stabilizer = 0.0320;
Alpha_0_horiz_stabilizer = 0;
Alpha_param_vector_horiz_stabilizer = Alpha_param_vector_NACA0006;
CL_param_vector_horiz_stabilizer =  CL_param_vector_NACA0006;
CD_param_vector_horiz_stabilizer = CD_param_vector_NACA0006;





xyz_elevator = [-0.5907 0, 0];
S_elevator = 0.0224;
Alpha_0_elevator = 0;
Alpha_param_vector_elevator = Alpha_param_vector_NACA0006;
CL_param_vector_elevator =  CL_param_vector_NACA0006;
CD_param_vector_elevator = CD_param_vector_NACA0006;



xyz_vert_stabilizer = [-0.491 0, 0];
S_vert_stabilizer = 0.018;
Beta_param_vector_vert_stabilizer = Alpha_param_vector_NACA0006;
CY_param_vector_vert_stabilizer =  CL_param_vector_NACA0006;
CD_param_vector_vert_stabilizer = CD_param_vector_NACA0006;

xyz_rudder = [-0.591, 0, 0];
S_rudder = 0.018;
Beta_param_vector_rudder = Alpha_param_vector_NACA0006;
CY_param_vector_rudder =  CL_param_vector_NACA0006;
CD_param_vector_rudder = CD_param_vector_NACA0006;

% Static parameters

g = 9.81;
rho_0 = 1.293;
uvw_0 = [12.37 ;0; 0];
pqr_0 = [0; 0; 0];
Ref_altitude = 0;
Euler_0 = [0; 0; 0];
% u_0 = [aileron, elevator, thrust, rudder]
%u_0 = [0; -1.07 ;0.58 ;0];
u_0 = [0; 0; 0; 0];

%% S temi podatki je nestabilen

%% pri višji hitrosti postanr stabilen
%uvw_0 = [19.53 0 0];

%% Stabilen je tudi ce pomaknemo krilo nazaj za 3cm
% xyz_wing = [-0.05, 0, 0];
% xyz_aileron_L = [ -0.08 -0.815 0];
% xyz_aileron_R = [-0.08 0.815 0];
T_fin = 200;

Script_trim_AI
Script_linearizacija_simple
Script_simulacija
Model_name = 'Model_aircraft_simple_fly';
Script_simulacija




