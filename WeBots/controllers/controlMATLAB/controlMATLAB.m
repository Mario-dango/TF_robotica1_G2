function controlMATLAB

% Script adaptado para ejecución interna/genérica
TIME_STEP = 32;
wb_robot_init();

% Obtener motores
motor_hombro = wb_robot_get_device('motor1'); 
motor_brazo = wb_robot_get_device('motor2');
motor_codo = wb_robot_get_device('motor3');
motor_antebrazo = wb_robot_get_device('motor4');
motor_muneca = wb_robot_get_device('motor5');
motor_falange = wb_robot_get_device('motor6');

% Configurar velocidades básicas
wb_motor_set_velocity(motor_hombro, 0.20);
wb_motor_set_velocity(motor_brazo, 1.0);
wb_motor_set_velocity(motor_codo, 1.0);
wb_motor_set_velocity(motor_antebrazo, 1.0);
wb_motor_set_velocity(motor_muneca, 1.0);
wb_motor_set_velocity(motor_falange, 1.0);

% Posición de destino
wb_motor_set_position(motor_hombro, 0.5); 
wb_motor_set_position(motor_brazo, -0.5); 

disp('¡Conexión interna establecida con éxito!');
% --- ANTES DEL WHILE (Planificación) ---
% 1. Definir los puntos clave en el espacio cartesiano (X, Y, Z, Roll, Pitch, Yaw)
P_home  = [0.3, 0.0, 0.5, 0, 0, 0]; 
P_inicio_recta = [0.4, -0.2, 0.2, 0, pi/2, 0];
P_final_recta  = [0.4,  0.2, 0.2, 0, pi/2, 0];

% 2. Generar discretización de la trayectoria recta (ej. 100 pasos)
num_pasos = 100;
trayectoria_X = linspace(P_inicio_recta(1), P_final_recta(1), num_pasos);
trayectoria_Y = linspace(P_inicio_recta(2), P_final_recta(2), num_pasos);
trayectoria_Z = linspace(P_inicio_recta(3), P_final_recta(3), num_pasos);
% (Hacer lo mismo para las orientaciones si es necesario)

% 3. Inicializar variables de estado de la máquina de estados
% Estado 1: Ir a inicio de recta | Estado 2: Seguir recta | Estado 3: Volver a Home
estado = 1;
paso_actual = 1;

% --- BUCLE PRINCIPAL DE WEBOTS ---
while wb_robot_step(TIME_STEP) ~= -1
    
    switch estado
        case 1 % Moverse de Home al inicio de la recta
            % Calculás IK para P_inicio_recta -> Obtenés [q1, q2, q3, q4, q5, q6]
            % Enviás posiciones con wb_motor_set_position a los 6 motores
            % Si los sensores de posición (PositionSensors) indican que llegó:
            estado = 2; 
            
        case 2 % Seguir la trayectoria recta paso a paso
            if paso_actual <= num_pasos
                P_actual = [trayectoria_X(paso_actual), trayectoria_Y(paso_actual), trayectoria_Z(paso_actual), 0, pi/2, 0];
                
                % [q1, q2, q3, q4, q5, q6] = mi_funcion_IK(P_actual);
                
                % wb_motor_set_position(motor1, q1);
                % ... wb_motor_set_position(motor6, q6);
                
                paso_actual = paso_actual + 1;
            else
                estado = 3; % Terminó la recta, ir a Home
            end
            
        case 3 % Volver a Home
            % Calculás IK para P_home -> Obtenés [q1...q6]
            % Enviás posiciones a los motores
            % Al llegar, podés detener el bucle o pausar el estado
    end
end

wb_robot_cleanup();
