%% JACOBIANO SIMBOLICO - FANUC M-10iD/8L
clc; clear; close all;
robot;   % crea R (se usa solo para verificar numericamente al final)

tic
syms q1 q2 q3 q4 q5 q6 real
q = [q1, q2, q3, q4, q5, q6];

%% Parametros DH exactos 
% trabajamos con funciones simbolicas, para que no queden numeros excesivamente grandes

% valores del parámetro d
d     = [sym(9)/20, 0, sym(1239)/10000, sym(11)/10, 0, sym(3)/40];

% valores del parámetro a
a     = [sym(3)/40, sym(21)/25, sym(39)/200, 0, 0, 0];

% valores del parametro alpha
alpha = [sym(pi)/2, 0, sym(pi)/2, sym(pi)/2, -sym(pi)/2, 0];

off   = [0, sym(pi)/2, 0, 0, 0, 0];   % offset de q2

% Matriz DH estandar
Ai = @(th, d, a, al) [cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th);
                      sin(th),  cos(th)*cos(al), -cos(th)*sin(al), a*sin(th);
                            0,          sin(al),          cos(al),         d;
                            0,                0,                0,        1];

%% Cinematica directa acumulada: guardamos z_{i-1} y o_{i-1}
T = sym(eye(4));
z = sym(zeros(3,6));   % z(:,i) = eje z del frame i-1
o = sym(zeros(3,6));   % o(:,i) = origen del frame i-1

for i = 1:6
    z(:,i) = T(1:3,3);
    o(:,i) = T(1:3,4);
    T = simplify(T * Ai(q(i) + off(i), d(i), a(i), alpha(i)));
    if i == 5
        p = T(1:3,4);      % centro de la muñeca (origen del frame 5)
    end
    fprintf('Frame %d listo (%.1f s)\n', i, toc);
end

%% Jacobiano geometrico 
J = sym(zeros(6,6));
for i = 1:6
    J(1:3,i) = cross(z(:,i), p - o(:,i));
    J(4:6,i) = z(:,i);
end
J = simplify(J);
disp('Jacobiano calculado');

% verificacion: J(1:3,4:6) debe ser cero
disp(J(1:3,4:6));

Jv = J(1:3,1:3);   % brazo   - velocidades lineales
Jw = J(4:6,4:6);   % muñeca  - velocidades angulares

det_arm   = simplify(det(Jv));
det_wrist = simplify(det(Jw));

disp('det(Jv) ='); disp(det_arm);
disp('det(Jw) ='); disp(det_wrist);

detJ_fact = simplify(det_arm * det_wrist);
disp('det(J) ='); disp(detJ_fact);

toc

%% Verificacion numerica contra el toolbox (R.jacob0)
R.tool = transl(0, 0, -0.075);          % punto de referencia = centro de muñeca
qn   = [0.3, -0.5, 0.8, 0.4, 0.7, -0.2]; % configuracion de prueba
Jnum = R.jacob0(qn);
Jsym = double(subs(J, q, qn));
fprintf('Error maximo vs jacob0: %g\n', max(abs(Jnum(:) - Jsym(:))));

