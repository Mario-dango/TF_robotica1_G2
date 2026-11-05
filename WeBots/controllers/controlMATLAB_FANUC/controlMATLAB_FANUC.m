function controlMATLAB_FANUC

% =========================================================================
% CONTROLADOR WEBOTS + MATLAB — FANUC M-10iD/12
% Integra robot.m + IK_numeric.m + robot_traj_validado.m con Webots
% =========================================================================

TIME_STEP = 32;                  % [ms] paso de simulación de Webots
dt = TIME_STEP / 1000;           % [s]  paso temporal (0.032 s)

wb_robot_init();

% 1. Obtener dispositivos de los 6 motores de Webots
motores = zeros(1, 6);
for j = 1:6
    motores(j) = wb_robot_get_device(['motor', num2str(j)]);
end

% 2. Cargar modelo cinemático (robot.m define R, R.base, R.tool, R.qlim, R.offset)
% Si robot.m e IK_numeric.m están en otra carpeta, descomentar y ajustar:
% addpath('C:/ruta/a/tus/scripts_matlab');
robot;

% 3. Configurar velocidades máximas de los motores según el 80% nominal del FANUC
velocidad_nominal_deg_s = [260 240 260 430 450 720];
lim_velocidad_rad_s     = deg2rad(velocidad_nominal_deg_s * 0.80);

for j = 1:6
    wb_motor_set_velocity(motores(j), lim_velocidad_rad_s(j));
end

% Vector de ajuste de sentido de giro por si algún HingeJoint en Webots
% tiene el eje invertido respecto al eje Z de Denavit-Hartenberg (+1 o -1)
signos_webots = [1, 1, 1, 1, 1, 1];

disp('¡Conexión con Webots establecida! Calculando y validando trayectoria...');

% =========================================================================
% ANTES DEL WHILE — Planificación y Validación Cinemática Completa
% =========================================================================
T_base = double(R.base);

% Pose Home: Pose "L" del datasheet (cero articular gracias a R.offset en robot.m)
q_home = [0, 0, 0, 0, 0, 0];

% Puntos cartesianos del cordón (relativos a la base del robot)
X_cord  =  0.90;
Y_ini   = -0.20;
Y_fin   =  0.20;
Z_cord  =  0.65;
Z_aprox =  Z_cord + 0.05;

ori = troty(pi) * trotx(deg2rad(15));

T_aprox  = T_base * transl(X_cord, Y_ini, Z_aprox) * ori;
T_inicio = T_base * transl(X_cord, Y_ini, Z_cord)  * ori;
T_fin    = T_base * transl(X_cord, Y_fin, Z_cord)  * ori;
T_retiro = T_base * transl(X_cord, Y_fin, Z_aprox) * ori;

% Semilla inicial coherente con R.offset de robot.m
q_seed = [0, deg2rad(45), deg2rad(-60), deg2rad(160), deg2rad(-125), 0];

% Resolución de puntos críticos con IK_numeric
[ok, q_tmp, msg] = IK_numeric(R, T_aprox, true, q_seed);
if ~ok, error('CI de T_aprox rechazada: %s', msg); end
q_aprox = q_tmp(end,:);

[ok, q_tmp, msg] = IK_numeric(R, T_inicio, true, q_aprox);
if ~ok, error('CI de T_inicio rechazada: %s', msg); end
q_inicio = q_tmp(end,:);

% Tiempos de cada fase sincronizados con TIME_STEP de Webots
t_aprox  = 3.0;  pasos_aprox  = round(t_aprox  / dt);
t_desc   = 1.0;  pasos_desc   = round(t_desc   / dt);
t_cord   = 5.0;  pasos_cord   = round(t_cord   / dt);
t_retiro = 1.0;  pasos_retiro = round(t_retiro / dt);
t_vuelta = 3.0;  pasos_vuelta = round(t_vuelta / dt);

tv_aprox  = linspace(0, t_aprox,  pasos_aprox);
tv_desc   = linspace(0, t_desc,   pasos_desc);
tv_retiro = linspace(0, t_retiro, pasos_retiro);
tv_vuelta = linspace(0, t_vuelta, pasos_vuelta);

% Fase 1: Home -> Aprox -> Inicio (interpolación articular jtraj)
qB_aprox = jtraj(q_home,  q_aprox,  tv_aprox);
qB_desc  = jtraj(q_aprox, q_inicio, tv_desc);
q_ida    = [qB_aprox; qB_desc];

% Fase 2: Cordón de soldadura Inicio -> Fin (interpolación cartesiana ctraj + IK)
T_tray_B = ctraj(T_inicio, T_fin, pasos_cord);
[ok, q_cordon, msg] = IK_numeric(R, T_tray_B, true, q_inicio);
if ~ok, error('Trayectoria cartesiana rechazada: %s', msg); end

% Fase 3: Fin -> Retiro -> Home (interpolación articular jtraj)
q_fin_B = q_cordon(end,:);
[ok, q_tmp, msg] = IK_numeric(R, T_retiro, true, q_fin_B);
if ~ok, error('CI de retiro rechazada: %s', msg); end
q_retiro_B = q_tmp(end,:);

qB_retiro = jtraj(q_fin_B,    q_retiro_B, tv_retiro);
qB_vuelta = jtraj(q_retiro_B, q_home,     tv_vuelta);
q_regreso = [qB_retiro; qB_vuelta];

fprintf('Trayectoria lista: Ida=%d pasos | Cordón=%d pasos | Regreso=%d pasos\n', ...
    size(q_ida,1), size(q_cordon,1), size(q_regreso,1));

% Inicializar máquina de estados
% Estado 1: Ir a inicio de recta | Estado 2: Seguir recta | Estado 3: Volver a Home | Estado 4: Fin
estado = 1;
paso_actual = 1;

% =========================================================================
% BUCLE PRINCIPAL DE WEBOTS
% =========================================================================
while wb_robot_step(TIME_STEP) ~= -1

    switch estado
        case 1 % Estado 1: Moverse de Home a Aproximación y descender al Inicio del cordón
            if paso_actual <= size(q_ida, 1)
                q_cmd = q_ida(paso_actual, :) .* signos_webots;
                for j = 1:6
                    wb_motor_set_position(motores(j), q_cmd(j));
                end
                paso_actual = paso_actual + 1;
            else
                disp('>> Inicio del cordón alcanzado. Comenzando soldadura (ctraj)...');
                estado = 2;
                paso_actual = 1;
            end

        case 2 % Estado 2: Seguir la trayectoria recta del cordón paso a paso
            if paso_actual <= size(q_cordon, 1)
                q_cmd = q_cordon(paso_actual, :) .* signos_webots;
                for j = 1:6
                    wb_motor_set_position(motores(j), q_cmd(j));
                end
                paso_actual = paso_actual + 1;
            else
                disp('>> Cordón finalizado. Retirando antorcha y volviendo a Home...');
                estado = 3;
                paso_actual = 1;
            end

        case 3 % Estado 3: Retiro vertical y regreso a la pose Home ("L")
            if paso_actual <= size(q_regreso, 1)
                q_cmd = q_regreso(paso_actual, :) .* signos_webots;
                for j = 1:6
                    wb_motor_set_position(motores(j), q_cmd(j));
                end
                paso_actual = paso_actual + 1;
            else
                disp('>> Ciclo completado en Pose Home.');
                estado = 4;
            end

        case 4 % Estado 4: Espera en Home
            % Mantiene la posición Home sin recalcular
    end
end

wb_robot_cleanup();
end