% Script_linearizacija_simple.m


%
% Linearization of the airplane model
[A,B,C,D] = linmod(Model_name,x_trim,[0 0 0 0]);
%%
% *Longitudinal system* 

% State order: 5643 =  p q r  u v w  phi theta psi x y h
% Longitudinal states: u w theta h q 
% Longitudinal indeces
i_long = [4 6 8 12 2 10];
%
A_long = A(i_long,i_long);
B_long = B(i_long,2);
C_long = eye(6);
D_long = zeros(6,1);
%
% Create state space system:
sys_long = ss(A_long,B_long,C_long,D_long);
%
% Calculate dampings and frequencies
disp('Longitudionalnipoli, dušenja frevence in časovne konstante')
damp(sys_long)

%% 
% *Lateral system* 
 
% State order: 5643 =  p q r  u v w  phi theta psi x y h
% Lateral states: v phi psi p r 
% Lateral indeces
i_lat = [5 7 9 1 3 11];
% 
A_lat = A(i_lat,i_lat);
B_lat = B(i_lat,1);
C_lat = eye(6);
D_lat = zeros(6,1);
%
% Create state space system:
sys_lat = ss(A_lat,B_lat,C_lat,D_lat);
%
% Calculate dampings and frequencies
disp('Lateralni poli, dušenja frevence in časovne konstante')
damp(sys_lat)

