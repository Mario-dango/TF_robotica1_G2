function controlMATLAB_FANUC

% =========================================================================
% 0. CONFIGURACIÓN DE RUTAS RELATIVAS (Estructura GitHub)
% =========================================================================
% Guardamos la ruta actual del controlador para volver al final si es necesario
ruta_controlador = pwd; 

% Buscamos la carpeta madre del proyecto (TF_robotica1_G2) subiendo 3 niveles
% Desde: \TF_robotica1_G2\WeBots\controllers\controlMATLAB_FANUC\
cd('..\..\..\');
ruta_raiz_proyecto = pwd;

% Construimos la ruta hacia la carpeta de tus scripts de MATLAB
ruta_scripts_matlab = fullfile(ruta_raiz_proyecto, 'MatLAB');

% Añadimos la carpeta de tus scripts y todas sus subcarpetas al PATH de MATLAB
% Esto incluye automáticamente tu archivo 'robot.m' e 'IK_numeric.m'
addpath(genpath(ruta_scripts_matlab));

% Regresamos a la carpeta del controlador para que Webots trabaje correctamente
cd(ruta_controlador);

% =========================================================================
% 1. INICIALIZACIÓN DE WEBOTS
% =========================================================================
TIME_STEP = 32;
wb_robot_init();

% Obtener referencias de los 6 motores en Webots
motores = zeros(1, 6);
for i = 1:6
    motores(i) = wb_robot_get_device(['motor' num2str(i)]);
    wb_motor_set_velocity(motores(i), 1.0); 
end

% =========================================================================
% 2. CÁLCULO DE LA TRAYECTORIA CARTESIANA (Solución B)
% =========================================================================
disp('=== Calculando trayectoria con rvctools ===');

% Ejecutamos tu script que define el robot (FANUC M-10iD/8L)
robot;

% Definición de puntos clave en base a tu archivo robot_traj.m
q_home = [0, -pi/4, -pi/2, -pi/2, -pi/4, 0];
X_cord = 0.80; Y_ini = -0.20; Y_fin = 0.20; Z_cord = 0.65; Z_aprox = Z_cord + 0.05;
ori = troty(pi) * trotx(deg2rad(15));

% --- CORRECCIÓN DE TIPOS (Matrices 4x4 puras) ---
% Forzamos a que tanto R.base, transl y ori sean tratados como matrices numéricas puras de 4x4

% R.base ya la definiste como double, pero aseguramos las funciones del toolbox
T_base   = double(R.base); 
T_transl = @(x,y,z) double(transl(x, y, z));
T_ori    = double(ori);

% Ahora multiplicamos matrices homogéneas puras sin conflicto de objetos
T_aprox  = T_base * T_transl(X_cord, Y_ini,  Z_aprox) * T_ori;
T_inicio = T_base * T_transl(X_cord, Y_ini,  Z_cord)  * T_ori;
T_fin    = T_base * T_transl(X_cord, Y_fin,  Z_cord)  * T_ori;
T_retiro = T_base * T_transl(X_cord, Y_fin,  Z_aprox) * T_ori;


% Semilla y cálculo de los puntos críticos de articulación
q_seed = [0, pi/4, -pi/4, 0, -pi/2, 0];
[ok, q_tmp, msg] = IK_numeric(R, T_aprox, true, q_seed);
if ~ok, error('CI de T_aprox rechazada por IK_numeric: %s', msg); end
q_aprox = q_tmp(end,:);

[ok, q_tmp, msg] = IK_numeric(R, T_inicio, true, q_aprox);
if ~ok, error('CI de T_inicio rechazada por IK_numeric: %s', msg); end
q_inicio = q_tmp(end,:);

[ok, q_tmp, msg] = IK_numeric(R, T_fin, true, q_inicio);
if ~ok, error('CI de T_fin rechazada por IK_numeric: %s', msg); end
q_fin = q_tmp(end,:);

[ok, q_tmp, msg] = IK_numeric(R, T_retiro, true, q_fin);
if ~ok, error('CI de T_retiro rechazada por IK_numeric: %s', msg); end
q_retiro = q_tmp(end,:);

% Parámetros temporales (Tramos idénticos a tus especificaciones de soldadura)
pasos_aprox = 30; pasos_desc = 10; pasos_cord = 50; pasos_retiro = 10; pasos_vuelta = 30;
tv_aprox = linspace(0, 3, pasos_aprox); tv_desc = linspace(0, 1, pasos_desc);
tv_retiro = linspace(0, 1, pasos_retiro); tv_vuelta = linspace(0, 3, pasos_vuelta);

% Generación de perfiles articulares y cartesiano
qB_aprox  = jtraj(q_home, q_aprox, tv_aprox);
qB_desc   = jtraj(q_aprox, q_inicio, tv_desc);

T_tray_B  = ctraj(T_inicio, T_fin, pasos_cord);
[ok, qB, msg] = IK_numeric(R, T_tray_B, true, q_inicio);
if ~ok, error('Trayectoria cartesiana rechazada por IK_numeric: %s', msg); end

% Resolución del retiro desde la configuración real alcanzada por la Solución B
q_fin_B = qB(end,:);
[ok, q_tmp, msg] = IK_numeric(R, T_retiro, true, q_fin_B);
if ~ok, error('CI de retiro para Solución B rechazada por IK_numeric: %s', msg); end
q_retiro_B = q_tmp(end,:);

qB_retiro = jtraj(q_fin_B, q_retiro_B, tv_retiro);
qB_vuelta = jtraj(q_retiro_B, q_home, tv_vuelta);

% Concatenación de toda la trayectoria
q_total = [qB_aprox; qB_desc; qB; qB_retiro; qB_vuelta];
total_muestras = size(q_total, 1);

disp('¡Trayectoria calculada con éxito! Iniciando simulación en Webots...');

% =========================================================================
% 3. BUCLE DE EJECUCIÓN EN WEBOTS
% =========================================================================
paso_actual = 1;

while wb_robot_step(TIME_STEP) ~= -1
    if paso_actual <= total_muestras
        q_enviar = q_total(paso_actual, :);
        
        % NOTA DE OFFSET: Si ves que en Webots el robot dobla al revés o asume una postura
        % incorrecta, descomentá la siguiente línea para restar el desfase de Peter Corke:
        % q_enviar = q_enviar - R.offset;
        
        % Asignar la posición calculada a cada motor correspondiente
        for j = 1:6
            wb_motor_set_position(motores(j), q_enviar(j));
        end
        
        paso_actual = paso_actual + 1;
    else
        disp('Trayectoria de soldadura finalizada por completo.');
        break; 
    end
end

% Limpieza al terminar
wb_robot_cleanup();
end
