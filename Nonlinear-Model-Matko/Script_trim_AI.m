%% 1. ZAČETNA UGIBANJA (Initial guesses)


% Želimo leteti naprej s hitrostjo u = 50 m/s na višini z = -1000 m (v letalstvu je z navzdol)
x0 = [15; 0; 0;  0; 0; 0;  0; 0; 0;  0; 0; 0]; 

commm_str = ['[sizes,x0,str,ts] = ',Model_name,'([], [], [], ''sizes'');'];
eval(commm_str);

%% Ce se sesuje
%  Model_aircraft_simple([],[],[],'term');


% Začetno ugibanje za vhode (npr. 30% potiska, krmila poravnana)
u0 = [0.3; 0; 0; 0];
% Popravil ker je v modelu že thrust
u0 = [0; 0; 0; 0];

% Začetno ugibanje za izhode (če jih imaš definirane v modelu, npr. želena hitrost in višina)
y0 = []; 


%% 2. INDEKSI OMEJITEV (Fiksiranje želenih vrednosti)
% Želimo, da algoritem nujno obdrži hitrost u=50 (1. stanje) in višino z=-1000 (12. stanje).
% Želimo tudi, da sta phi (7. stanje) in v (2. stanje) natančno 0 (Raven let).
ix = [5; 6; 7; 11]; 

% Vhodov ne fiksiramo, naj jih algoritem sam izračuna, zato je iu prazen.
iu = []; 
iy = [];

%% 3. OMEJITVE ODVODOV STANOV (Derivatives constraints)
% Za ravnovesno stanje želimo, da so odvodi večine stanj enaki 0.
% dx0 predstavlja želene vrednosti odvodov stanj.
dx0 = zeros(12, 1); 

% Indeks idx pove, kateri odvodi MORAJO biti natančno enaki vrednostim v dx0.
% Za raven let morajo biti vsi odvodi stabilizirani (enaki 0), 
% razen pozicije x, y, z (saj se letalo premika skozi prostor).
% Torej fiksiramo odvode stanj od 1 do 9 (hitrosti in koti se ne spreminjajo):

% dodal sem še 12 da bolj utežim višino
idx = [1; 2; 3; 4; 5; 6; 7; 8; 9;12];



% 4. NASTAVITVE OPTIMIZACIJE
options = zeros(1, 18);
options(1) = 1;     % 1 pomeni, da bo MATLAB v oknu prikazal potek optimizacije
options(2:4) = 1e-10;  % Natančnost stanj

%% 5. KLIC FUNKCIJE TRIM
[x_trim, u_trim, y_trim, dx_trim] = trim(Model_name, x0, u0, y0, ix, iu, iy, dx0, idx, options);

%% 6. PRIKAZ REZULTATOV
disp('--- Izračunano ravnovesno stanje (Stanja) ---');
disp(x_trim);
disp('--- Izračunani krmilni vhodi ---');
disp(u_trim);

% Ključna pravila za uspeh:
% ​Model mora biti pripravljen: Preden poženeš trim, se prepričaj, da se tvoj Simulink model lahko požene za čas t=0 brez napak.
% ​Koti v radianih: MATLAB-ovi fizikalni modeli skoraj vedno zahtevajo kote (phi, theta, psi) ter kotne hitrosti (p, q, r) v radianih in rad/s, ne v stopinjah.
% ​Smer osi Z: Če uporabljaš standardni aerodinamični koordinatni sistem (NED - North-East-Down), je os Z usmerjena navzdol proti središču zemlje. Višina 1000 metrov nad tlemi pomeni, da mora biti koordinata z = -1000.
% ​Če želiš namesto ravnega leta letalo strimati v stalnem zavoju (Coordinated Turn) ali vzpenjanju, je treba ustrezno spremeniti vektor dx0. Na primer, za vzpenjanje s hitrostjo 5 \text{ m/s} bi nastavil odvod položaja Z na -5 (dx0(12) = -5) in dodal 12 v indeks idx.


% Set trimmed input (aileron, elevator, thrust,rudder):
u_0 = u_0+u_trim;

uvw_0 = x_trim(4:6);
Euler_0 =  x_trim(7:9);