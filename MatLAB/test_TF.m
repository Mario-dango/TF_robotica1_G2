
clear; clc; close all;

robot;

fprintf('=================================================================\n');
fprintf('  PRUEBA EJERCICIO TF - FANUC M-10iD\n');
fprintf('=================================================================\n\n');

N_puntos = 10;
y_vec = linspace(-0.4, 0.4, N_puntos);
poses = zeros(4, 4, N_puntos);

for k = 1:N_puntos
    poses(:,:,k) = transl(0.80, y_vec(k), 0.50) * trotx(pi);
end

q0 = zeros(1, R.n);

fprintf('---> MODO q_mejor = true:\n');
[exito1, q_sol1, err1] = resolver_CI_trayectoria(R, poses, true, q0);

if exito1
    fprintf('   [EXITO] Trayectoria resuelta correctamente.\n');
    fprintf('   Matriz de posiciones articulares: [%d x %d]\n\n', size(q_sol1, 1), size(q_sol1, 2));
else
    fprintf('   [ERROR] %s\n\n', err1);
end

fprintf('---> MODO q_mejor = false:\n');
[exito2, q_sol2, err2] = resolver_CI_trayectoria(R, poses, false, q0);

if exito2
    fprintf('   [EXITO] Se encontraron %d trayectorias unicas posibles.\n', length(q_sol2));
else
    fprintf('   [ERROR] %s\n', err2);
end
