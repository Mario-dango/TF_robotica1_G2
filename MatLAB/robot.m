% =========================================================================
% Definición del robot FANUC M-10iD 
% =========================================================================

L(1) = Link('d', 0.45, 'a', 0.15, 'alpha', -pi/2, 'qlim', [-180 180]*pi/180);
L(2) = Link('d', 0,    'a', 0.60, 'alpha', 0,      'qlim', [-90 150]*pi/180);
L(3) = Link('d', 0,    'a', 0.20, 'alpha', -pi/2, 'qlim', [-180 180]*pi/180);
L(4) = Link('d', 0.67, 'a', 0,    'alpha', pi/2,  'qlim', [-180 180]*pi/180);
L(5) = Link('d', 0,    'a', 0,    'alpha', -pi/2, 'qlim', [-180 180]*pi/180);
L(6) = Link('d', 0.10, 'a', 0,    'alpha', 0,      'qlim', [-360 360]*pi/180);

R = SerialLink(L, 'name', 'FANUC_M10iD');

workspace = [-2, 2, -2, 2, 0, 3];
% q_inicial = [q1, q2, q3, q4, q5, q6]
q_inicial = [0, 0, 0, 0, -pi/2, 0];

R.teach(q_inicial, 'workspace', workspace, 'jointdiam', 0.4, 'scale', 0.5);