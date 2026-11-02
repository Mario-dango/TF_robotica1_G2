clc; clear; close all;

% 1. Cargamos el robot en memoria
robot;

%% --- PREPARACIÓN DE LOS PARÁMETROS DE ENTRADA ---

% a) semilla inicial
q0 = [0, 0, 0, 0.3, 0.5, 0.2];

% b) sucesión de poses
q_inicio = [0, 0, 0, 0.3, 0.5, 0.2];
q_fin    = [0, pi/4, pi/2, 0.3, 0.5, 0.2];
trayectoria_real = jtraj(q_inicio, q_fin, 30); 

%% --- CINEMATICA DIRECTA ---
% Obtenemos las poses cartesianas (T será de tamaño 4x4x30)
T = R.fkine(trayectoria_real);

% posiciones (x,y,z) del objetivo, una fila por pose (30x3).
% T(1:3,4,:) es la columna de traslación de cada una de las 30 matrices;
% squeeze la acomoda y (') la transpone a formato 30x3.
Tm = double(T);
xyz_objetivo = squeeze(Tm(1:3,4,:))';

%% --- PRUEBA 1: q_mejor = true (Única solución) ---
disp('--- PROBANDO q_mejor = true ---');
q_mejor = true;

[exito_1, q_resultado_1, mensaje_1] = IK_numeric(R, T, q_mejor, q0);

disp(['Éxito: ', num2str(exito_1)]);
disp(['Mensaje: ', mensaje_1]);

if exito_1
    figure('Name', 'Articulaciones - Única solución');
    plot(q_resultado_1, 'LineWidth', 1.5);
    grid on; xlabel('Muestra'); ylabel('q [rad]');
    legend('q1','q2','q3','q4','q5','q6');

    % robot + trayectoria cartesiana objetivo
    figure('Name', 'Prueba Única Solución');
    R.plot(q_resultado_1(1,:), 'workspace', workspace, 'scale', 0.5, 'notiles');
    hold on;
    plot3(xyz_objetivo(:,1), xyz_objetivo(:,2), xyz_objetivo(:,3), 'r--', 'LineWidth', 2);
    R.plot(q_resultado_1, 'workspace', workspace, 'scale', 0.5, 'notiles');
end

%% --- PRUEBA 2: q_mejor = false (Múltiples soluciones) ---
disp('--- PROBANDO q_mejor = false ---');
q_mejor = false;

[exito_2, q_resultado_2, mensaje_2] = IK_numeric(R, T, q_mejor, q0);

disp(['Éxito: ', num2str(exito_2)]);
disp(['Mensaje: ', mensaje_2]);

if exito_2
    num_soluciones = length(q_resultado_2);
    disp(['¡Se encontraron ', num2str(num_soluciones), ' soluciones distintas!']);
    for i = 1:num_soluciones
        figure('Name', ['Articulaciones - Configuración #', num2str(i)]);
        plot(q_resultado_2{i}, 'LineWidth', 1.5);
        grid on; xlabel('Muestra'); ylabel('q [rad]');
        legend('q1','q2','q3','q4','q5','q6');

        R_i = SerialLink(R, 'name', ['FANUC M-10iD/8L #', num2str(i)]);
        figure('Name', ['Configuración #', num2str(i)]);

        % robot + trayectoria objetivo, y luego animación
        R_i.plot(q_resultado_2{i}(1,:), 'workspace', workspace, 'scale', 0.5, 'notiles');
        hold on;
        plot3(xyz_objetivo(:,1), xyz_objetivo(:,2), xyz_objetivo(:,3), 'r--', 'LineWidth', 2);

        % Animar la trayectoria completa
        R_i.plot(q_resultado_2{i}, 'workspace', workspace, 'scale', 0.5, 'notiles');

        disp(['Solución #', num2str(i), ' (primera fila, en grados):']);
        disp(rad2deg(q_resultado_2{i}(1,:)));
    end
end