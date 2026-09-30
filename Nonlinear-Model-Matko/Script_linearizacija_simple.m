% Script_linearizacija_simple.m


%
% Linearization of the airplane model
[A,B,C,D] = linmod(Model_name,x_trim,[0 0 0 0]);
% Matrices are converted to the states used in static_analysis.ipynb
% (valid for trim at alpha = beta = 0). Inputs are in rad.
V = x_trim(4);

%%
% *Longitudinal system* 

% State order: 5643 =  p q r  u v w  phi theta psi x y h
% Longitudinal states (notebook): V alpha q theta
%   V ~ u, alpha ~ w/V
i_long = [4 6 2 8];
T_long = diag([1 1/V 1 1]);
%
A_long = T_long*A(i_long,i_long)/T_long;
B_long = T_long*B(i_long,2)*180/pi;      % elevator
C_long = [1 0 0 0; 0 0 1 0; 0 0 0 1];
D_long = zeros(3,1);
disp('A_long [V alpha q theta]:'), disp(A_long)
disp('B_long [elevator]:'), disp(B_long)
%
% Calculate dampings and frequencies
disp('Longitudionalnipoli, dušenja frevence in časovne konstante')
izpis_polov(A_long)

%% 
% *Lateral system* 
 
% State order: 5643 =  p q r  u v w  phi theta psi x y h
% Lateral states (notebook): beta p r phi
%   beta ~ -v/V, aileron and rudder have opposite sign in the notebook
i_lat = [5 1 3 7];
T_lat = diag([-1/V 1 1 1]);
% 
A_lat = T_lat*A(i_lat,i_lat)/T_lat;
B_lat = -T_lat*B(i_lat,[1 4])*180/pi;    % aileron, rudder
C_lat = [0 1 0 0; 0 0 1 0; 0 0 0 1];
D_lat = zeros(3,2);
disp('A_lat [beta p r phi]:'), disp(A_lat)
disp('B_lat [aileron rudder]:'), disp(B_lat)
%
% Calculate dampings and frequencies
disp('Lateralni poli, dušenja frevence in časovne konstante')
izpis_polov(A_lat)

%%
% Replacement for damp() so the Control System Toolbox is not needed
function izpis_polov(A)
    p = eig(A);
    [~, i] = sort(abs(p));
    p = p(i);
    wn = abs(p);                 % natural frequency [rad/s]
    zeta = -real(p)./wn;         % damping
    zeta(wn == 0) = -1;          % same as damp() for poles at 0
    tau = -1./real(p);           % time constant [s]
    tau(real(p) == 0) = Inf;
    fprintf('%24s %12s %14s %14s\n', 'Pole', 'Damping', 'Frequency', 'Time Constant')
    fprintf('%24s %12s %14s %14s\n', '', '', '(rad/s)', '(s)')
    for k = 1:numel(p)
        fprintf('%11.3e %+11.3ei %12.3e %14.3e %14.3e\n', real(p(k)), imag(p(k)), zeta(k), wn(k), tau(k))
    end
    fprintf('\n')
end

