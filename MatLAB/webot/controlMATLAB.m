% Script de MATLAB para controlar el robot FANUC en Webots
TIME_STEP = 32; % Tiempo de actualización de la simulación en milisegundos

% 1. Inicializar la conexión con el robot
% Nota: Webots carga automáticamente las librerías necesarias al ejecutar
try
    wb_robot_init();
catch
    error('Asegúrate de ejecutar este script presionando el botón "Run" desde Webots o configurando el entorno.');
end

% 2. Obtener los identificadores (handles) de los motores
% Reemplaza "motor1" y "motor2" por los nombres exactos que les diste en la propiedad 'name' del RotationalMotor
motor_hombro = wb_robot_get_device('motor1'); 
motor_brazo = wb_robot_get_device('motor2');

% 3. Configurar la velocidad máxima de los movimientos (en radianes/segundo)
wb_motor_set_velocity(motor_hombro, 1.0);
wb_motor_set_velocity(motor_brazo, 1.0);

% Definir las posiciones objetivo (en RADIANES)
% Ejemplo: 45 grados (0.785 rad) para el hombro y -30 grados (-0.523 rad) para el brazo
target_pos_1 = 0.785; 
target_pos_2 = -0.523;

% Enviar la orden de posición a ambos motores (se ejecutarán en paralelo)
wb_motor_set_position(motor_hombro, target_pos_1);
wb_motor_set_position(motor_brazo, target_pos_2);

disp('Enviando comandos de movimiento coordinado...');

% 4. Bucle principal de simulación
% Este ciclo mantiene el script vivo e informa a Webots que siga avanzando el tiempo
while wb_robot_step(TIME_STEP) ~= -1
    
    % (Opcional) Aquí podrías leer sensores de posición o añadir condiciones de parada
    % Por ahora, el robot simplemente se moverá hacia las posiciones indicadas arriba.
    
    % Si deseas detener el script tras unos segundos, puedes descomentar o añadir lógica aquí.
end

% Finalizar la simulación limpiando la memoria de la API
wb_robot_cleanup();
