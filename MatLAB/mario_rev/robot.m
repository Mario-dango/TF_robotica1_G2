clc; clear; close all;

% =========================================================================
% 1. PARÁMETROS DENAVIT-HARTENBERG
% =========================================================================
% Formato: [theta, d, a, alpha]
dh = [
    0,  0.45,   0.15,  -pi/2;
    0,     0,   0.60,      0;
    0,     0,   0.20,  -pi/2;
    0,  0.67,      0,   pi/2;
    0,     0,      0,  -pi/2;
    0,  0.10,      0,      0
];

% Preasignación de memoria y creación de eslabones
L = Link.empty(0, 6);
for i = 1:6
    L(i) = Link(dh(i, 1:4));
end

% Límites articulares
L(1).qlim = deg2rad([-180, 180]);
L(2).qlim = deg2rad([-90,  150]);
L(3).qlim = deg2rad([-180, 180]);
L(4).qlim = deg2rad([-180, 180]);
L(5).qlim = deg2rad([-180, 180]);
L(6).qlim = deg2rad([-360, 360]);

% Creación del objeto SerialLink
R = SerialLink(L, 'name', 'FANUC M-10iD');

% Offset en q2 (Alinea la postura cero del brazo FANUC)
R.offset = [0, -pi/2, 0, 0, 0, 0];

% =========================================================================
% 2. BASE DEL ROBOT
% =========================================================================
% Pedestal elevador de 600mm
R.base = transl(0, 0, 0.60); 

% =========================================================================
% 3. HERRAMIENTA (TCP) Y OPCIONES DE MONTAJE
% =========================================================================
% Geometría de la torcha PSF 315M a 25 grados
torcha_Z = 0.1963; % Dimensión E (196.3 mm)
torcha_X = 0.0634; % Dimensión F (63.4 mm)
ang_torcha = deg2rad(25); 

% -- Descomenta la opción de montaje que prefieras --

% OPCIÓN A: Montaje axial (Acople cilíndrico de 100mm en el eje Z)
acople_Z = 0.100; 
R.tool = transl(-torcha_X, 0, acople_Z + torcha_Z) * troty(ang_torcha);

% OPCIÓN B: Montaje en Escuadra (Desplaza en Z y quiebra 90° hacia abajo)
% acople_L = 0.120; % 120mm de largo de escuadra
% R.tool = transl(acople_L + torcha_Z, 0, -torcha_X) * troty(ang_torcha) * troty(pi/2);

% =========================================================================
% 4. VISUALIZACIÓN INICIAL
% =========================================================================
workspace = [-2, 2, -2, 2, 0, 3];
% q_inicial = [q1, q2, q3, q4, q5, q6]
q_inicial = [0, 0, 0, 0, -pi/2, 0];

figure('Name', 'Estructura FANUC M-10iD', 'Position', [100 100 800 600]);
R.teach(q_inicial, 'workspace', workspace, 'jointdiam', 0.4, 'scale', 0.5);