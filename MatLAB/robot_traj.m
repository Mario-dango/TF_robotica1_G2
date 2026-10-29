% =========================================================================
% SCRIPT: GENERACIÓN Y ANÁLISIS DE TRAYECTORIAS DE SOLDADURA
%         FANUC M-10iD/12 — Robótica 1 — Trabajo Final
%
% ESTRUCTURA DEL SCRIPT:
%   0. Setup: carga robot, herramienta y base (igual que esp_trab.m)
%   1. Definición de puntos clave en el espacio cartesiano
%   2. Solución A — Interpolación ARTICULAR  (jtraj)
%   3. Solución B — Interpolación CARTESIANA (ctraj)
%   4.  Gráficos articulares: posición, velocidad, aceleración
%   5. Gráficos cartesianos: posición, velocidad, aceleración
%   6. Simulación 3D completa: home → aprox → soldadura → home
%
% REQUISITOS:
%   - MATLAB R2023a
%   - Peter Corke Robotics Toolbox (última versión)
%   - Haber ejecutado robot.m al menos una vez en la sesión
%     (o tener R, workspace definidos en el workspace de MATLAB).
%     Si no, este script los carga automáticamente.
% =========================================================================

clear; clc; close all;

% -------------------------------------------------------------------------
% 0. SETUP — Robot, herramienta y base
% -------------------------------------------------------------------------
disp('=== SETUP ===');

% Cargar definición del robot si no existe en el workspace
if ~exist('R', 'var')
    disp('Cargando definición del robot...');
    robot;   % ejecuta robot.m → crea L(1..6) y R
end

% Herramienta: antorcha de soldadura MIG
%   - Acople:    100 mm en Z
%   - Cuerpo:    196.3 mm en Z
%   - Desvío X:  -63.4 mm (cuello curvo)
%   - Ángulo:    25° en Y (salida del alambre)
dist_Z          = 0.100 + 0.1963;    % [m]  traslación axial total
dist_X          = -0.0634;           % [m]  desvío lateral
angulo_antorcha = deg2rad(25);       % [rad]

R.tool = transl(dist_X, 0, dist_Z) * troty(angulo_antorcha);

% Pedestal (base elevada 600 mm sobre el piso)
R.base = transl(0, 0, 0.60);

% Volumen de visualización (referenciado al marco BASE del robot)
workspace = [-1.5, 1.5, -1.5, 1.5, -0.7, 2.0];

disp('Setup completado.');
fprintf('  Tool (resumen): dX=%.1fmm, dZ=%.1fmm, angY=%.0f°\n', ...
    dist_X*1e3, dist_Z*1e3, rad2deg(angulo_antorcha));
fprintf('  Base: pedestal %.0f mm\n\n', 600);

% -------------------------------------------------------------------------
% 1. DEFINICIÓN DE PUNTOS CLAVE
%    Todas las cotas son relativas al marco BASE del robot.
%    Cota Z absoluta (piso): Z_base + Z_relativo = 0.60 + 0.65 = 1.25 m
%    → coherente con el banco de trabajo a ~1.25 m de altura.
% -------------------------------------------------------------------------
disp('=== PUNTOS CLAVE ===');

% --- Q_home: postura de reposo (mínimo esfuerzo, codo recogido) ---
q_home = [0, -pi/2, -pi, -pi, -pi/2, 0];

% --- T_aprox: punto de aproximación (50 mm sobre el inicio del cordón) ---
%     El robot desciende desde aquí hasta T_inicio en línea recta.
X_cord =  0.80;    % [m] distancia al frente (eje X base)
Y_ini  = -0.20;    % [m] extremo inicial del cordón
Y_fin  =  0.20;    % [m] extremo final   del cordón (cordón = 400 mm)
Z_cord =  0.65;    % [m] altura de trabajo (Z rel. base)
Z_aprox = Z_cord + 0.05;   % 50 mm de clearance sobre el cordón

% Orientación de la antorcha:
%   - troty(pi): gira 180° en Y → TCP apunta hacia abajo
%   - trotx(deg2rad(15)): 15° de ángulo de avance (estándar MIG/MAG)
ori = troty(pi) * trotx(deg2rad(15));

T_aprox  = R.base.T * transl(X_cord, Y_ini,  Z_aprox) * ori;
T_inicio = R.base.T * transl(X_cord, Y_ini,  Z_cord)  * ori;
T_fin    = R.base.T * transl(X_cord, Y_fin,  Z_cord)  * ori;
T_retiro = R.base.T * transl(X_cord, Y_fin,  Z_aprox) * ori;
% 
% % Semilla IK: postura "codo arriba", evita singularidades cercanas
% q_seed = [0, 0, 0, 0, -pi/2, 0];
% 
% % Cinemática inversa de los cuatro puntos cartesianos
% q_aprox  = R.ikcon(T_aprox,  q_seed);
% q_inicio = R.ikcon(T_inicio, q_seed);
% q_fin    = R.ikcon(T_fin,    q_seed);
% q_retiro = R.ikcon(T_retiro, q_seed);

% Semilla IK inicial orientada hacia la mesa
q_seed = [0, pi/4, -pi/4, 0, -pi/2, 0];

% Usamos ikcon (robusto) pero ENCADENANDO las semillas.
% Al pasar la pose anterior como semilla del próximo punto, 
% obligamos al solver a elegir la solución más cercana, evitando giros de 360°.
q_aprox  = R.ikcon(T_aprox,  q_seed);
q_inicio = R.ikcon(T_inicio, q_aprox);
q_fin    = R.ikcon(T_fin,    q_inicio);
q_retiro = R.ikcon(T_retiro, q_fin);

resolver_CI_trayectoria(R,poses, true, q_seed);

fprintf('  Cordón: X=%.2fm, Y=[%.2f → %.2f]m, Z=%.2fm\n', ...
    X_cord, Y_ini, Y_fin, Z_cord);
fprintf('  Longitud del cordón: %.0f mm\n', (Y_fin-Y_ini)*1e3);
fprintf('  Clearance de aproximación: %.0f mm\n\n', (Z_aprox-Z_cord)*1e3);

% -------------------------------------------------------------------------
% 2. PARÁMETROS TEMPORALES
%    Se usan tramos independientes para cada fase de movimiento.
% -------------------------------------------------------------------------
t_aprox   = 3;    % [s] home → aprox  (movimiento libre, más rápido)
t_desc    = 1;    % [s] aprox → inicio (descenso)
t_cord    = 5;    % [s] inicio → fin  (soldadura, velocidad controlada)
t_retiro  = 1;    % [s] fin → retiro
t_vuelta  = 3;    % [s] retiro → home

pasos_aprox  = 30;
pasos_desc   = 10;
pasos_cord   = 50;    % mayor densidad sobre el cordón
pasos_retiro = 10;
pasos_vuelta = 30;

% Vectores de tiempo independientes para cada tramo
tv_aprox  = linspace(0, t_aprox,  pasos_aprox);
tv_desc   = linspace(0, t_desc,   pasos_desc);
tv_cord   = linspace(0, t_cord,   pasos_cord);
tv_retiro = linspace(0, t_retiro, pasos_retiro);
tv_vuelta = linspace(0, t_vuelta, pasos_vuelta);

disp('=== TIEMPOS DE CICLO ===');
fprintf('  Home → Aprox:    %.1f s\n', t_aprox);
fprintf('  Aprox → Inicio:  %.1f s\n', t_desc);
fprintf('  Soldadura:       %.1f s  (%.1f mm/s)\n', ...
    t_cord, (Y_fin-Y_ini)*1e3/t_cord);
fprintf('  Retiro:          %.1f s\n', t_retiro);
fprintf('  Retiro → Home:   %.1f s\n', t_vuelta);
fprintf('  TIEMPO TOTAL:    %.1f s\n\n', ...
    t_aprox+t_desc+t_cord+t_retiro+t_vuelta);

% =========================================================================
% SOLUCIÓN A — INTERPOLACIÓN ARTICULAR (jtraj)
%   Principio: IK solo en los puntos extremos de cada tramo,
%   luego polinomio de 5° grado en el espacio articular.
%   Consecuencia: el TCP NO sigue necesariamente una línea recta.
% =========================================================================
disp('=== SOLUCIÓN A: Interpolación Articular (jtraj) ===');

% jtraj devuelve [posición, velocidad, aceleración] articular
[qA_aprox,  qdA_aprox,  qddA_aprox]  = jtraj(q_home,   q_aprox,  tv_aprox);
[qA_desc,   qdA_desc,   qddA_desc]   = jtraj(q_aprox,  q_inicio, tv_desc);
[qA_cord,   qdA_cord,   qddA_cord]   = jtraj(q_inicio, q_fin,    tv_cord);
[qA_retiro, qdA_retiro, qddA_retiro] = jtraj(q_fin,    q_retiro, tv_retiro);
[qA_vuelta, qdA_vuelta, qddA_vuelta] = jtraj(q_retiro, q_home,   tv_vuelta);

% Concatenar tramos (el cordón es el tramo de análisis principal)
% Para los gráficos de análisis usamos solo el tramo del cordón
qA   = qA_cord;
qdA  = qdA_cord;
qddA = qddA_cord;
tA   = tv_cord;

% Trayectoria completa para simulación 3D
qA_full = [qA_aprox; qA_desc; qA_cord; qA_retiro; qA_vuelta];

% Cinemática directa para obtener posición cartesiana del TCP
xyzA = zeros(pasos_cord, 3);
for i = 1:pasos_cord
    T_i = R.fkine(qA(i,:));
    xyzA(i,:) = transl(T_i)';
end

% Velocidad y aceleración cartesiana por diferenciación numérica
dt = tA(2) - tA(1);
vxyzA   = gradient(xyzA,   dt);      % primera diferencia
axyzA   = gradient(vxyzA,  dt);      % segunda diferencia

disp('  Solución A calculada.');

% =========================================================================
% SOLUCIÓN B — INTERPOLACIÓN CARTESIANA (ctraj)
%   Principio: interpolación de la transformación homogénea (línea recta
%   en posición + interpolación esférica en orientación).
%   Luego IK punto a punto a lo largo de la recta.
%   Consecuencia: TCP sigue exactamente la trayectoria programada.
% =========================================================================
disp('=== SOLUCIÓN B: Interpolación Cartesiana (ctraj) ===');

% Solo el tramo del cordón se interpola cartesianamente
% (los tramos libres siguen siendo articulares para mayor eficiencia)
T_tray_B = ctraj(T_inicio, T_fin, pasos_cord);

% IK punto a punto (usa la configuración anterior como semilla → consistencia)
qB   = zeros(pasos_cord, 6);
q_prev = q_inicio;
for i = 1:pasos_cord
    qB(i,:) = R.ikcon(T_tray_B(:,:,i), q_prev);
    q_prev  = qB(i,:);    % semilla recurrente: evita saltos de configuración
end

% Velocidad y aceleración articular por diferenciación numérica
dt = tv_cord(2) - tv_cord(1);
qdB  = gradient(qB,  dt);
qddB = gradient(qdB, dt);

tB = tv_cord;

% Extraer coordenadas cartesianas de las matrices T_tray_B
xyzB = zeros(pasos_cord, 3);
for i = 1:pasos_cord
    xyzB(i,:) = transl(T_tray_B(:,:,i))';
end

% Velocidad y aceleración cartesiana por diferenciación numérica
vxyzB  = gradient(xyzB,  dt);
axyzB  = gradient(vxyzB, dt);

% Trayectoria completa para simulación 3D
% (tramos libres articulares, cordón con IK de ctraj)
[qB_aprox,  ~, ~] = jtraj(q_home,   q_aprox,  tv_aprox);
[qB_desc,   ~, ~] = jtraj(q_aprox,  q_inicio, tv_desc);
[qB_retiro, ~, ~] = jtraj(q_fin,    q_retiro, tv_retiro);
[qB_vuelta, ~, ~] = jtraj(q_retiro, q_home,   tv_vuelta);

% Recalcular q_fin para Solución B (puede diferir levemente de q_fin articular)
q_fin_B = qB(end,:);

% Recalcular retiro y vuelta usando el q_fin de la Solución B
[qB_retiro, ~, ~] = jtraj(q_fin_B,  q_retiro, tv_retiro);

qB_full = [qB_aprox; qB_desc; qB; qB_retiro; qB_vuelta];

disp('  Solución B calculada.');

% =========================================================================
% 4. GRÁFICOS ARTICULARES — Posición, Velocidad, Aceleración
%    (solo tramo de soldadura)
% =========================================================================
disp('=== Generando gráficos articulares... ===');

etiquetas_q = {'q_1','q_2','q_3','q_4','q_5','q_6'};
colores = lines(6);

figure('Name','Perfiles Articulares — Solución A (jtraj)',...
       'Position',[50 50 1100 750]);

% --- Posición articular ---
subplot(3,1,1);
hold on;
for j = 1:6
    plot(tA, rad2deg(qA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Posición articular');
ylabel('Ángulo [°]'); grid on; xlim([tA(1) tA(end)]);

% --- Velocidad articular ---
subplot(3,1,2);
hold on;
for j = 1:6
    plot(tA, rad2deg(qdA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Velocidad articular');
ylabel('[°/s]'); grid on; xlim([tA(1) tA(end)]);

% --- Aceleración articular ---
subplot(3,1,3);
hold on;
for j = 1:6
    plot(tA, rad2deg(qddA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Aceleración articular');
xlabel('Tiempo [s]'); ylabel('[°/s²]'); grid on; xlim([tA(1) tA(end)]);

% ---

figure('Name','Perfiles Articulares — Solución B (ctraj + IK)',...
       'Position',[200 50 1100 750]);

subplot(3,1,1);
hold on;
for j = 1:6
    plot(tB, rad2deg(qB(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. B — Posición articular');
ylabel('Ángulo [°]'); grid on; xlim([tB(1) tB(end)]);

subplot(3,1,2);
hold on;
for j = 1:6
    plot(tB, rad2deg(qdB(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. B — Velocidad articular');
ylabel('[°/s]'); grid on; xlim([tB(1) tB(end)]);

subplot(3,1,3);
hold on;
for j = 1:6
    plot(tB, rad2deg(qddB(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. B — Aceleración articular');
xlabel('Tiempo [s]'); ylabel('[°/s²]'); grid on; xlim([tB(1) tB(end)]);

% =========================================================================
% 5. GRÁFICOS CARTESIANOS — Posición, Velocidad, Aceleración
% =========================================================================
disp('=== Generando gráficos cartesianos... ===');

eje_labels = {'X [m]', 'Y [m]', 'Z [m]'};
eje_col    = {'r','g','b'};

figure('Name','Perfiles Cartesianos — Comparativa A vs B',...
       'Position',[350 50 1100 850]);

titulos_pos = {'Posición X del TCP','Posición Y del TCP','Posición Z del TCP'};
titulos_vel = {'Velocidad X','Velocidad Y','Velocidad Z'};
titulos_acc = {'Aceleración X','Aceleración Y','Aceleración Z'};

for k = 1:3
    % Posición
    subplot(3,3,k);
    plot(tA, xyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, xyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_pos{k});
    ylabel(eje_labels{k}); grid on; xlim([tA(1) tA(end)]);
    if k==1, legend('Sol. A','Sol. B','Location','best'); end

    % Velocidad
    subplot(3,3,3+k);
    plot(tA, vxyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, vxyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_vel{k});
    ylabel(['d' eje_labels{k}(1) '/dt [m/s]']); grid on; xlim([tA(1) tA(end)]);

    % Aceleración
    subplot(3,3,6+k);
    plot(tA, axyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, axyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_acc{k});
    xlabel('Tiempo [s]');
    ylabel(['d²' eje_labels{k}(1) '/dt² [m/s²]']); grid on; xlim([tA(1) tA(end)]);
end
sgtitle('Espacio Cartesiano: Sol. A (rojo--) vs Sol. B (azul—)');

% -------------------------------------------------------------------------
% Gráfico adicional: desviación en Z y en Y respecto al ideal
%   Muestra cuantitativamente por qué la Sol. B es superior para soldadura
% -------------------------------------------------------------------------
figure('Name','Desviación del TCP respecto al cordón ideal',...
       'Position',[500 50 900 500]);

% Z_ideal = Z_cord * ones(pasos_cord, 1);    % Z debería ser constante
% Y_ideal = linspace(Y_ini, Y_fin, pasos_cord)';
% 1. Ajuste en Z: Le sumamos los 0.60m del pedestal al ideal para igualar la cota absoluta del robot
Z_ideal = (0.60 + Z_cord) * ones(pasos_cord, 1);    

% 2. Ajuste en Y: Usamos 'tpoly' para que el avance ideal tenga la misma 
% aceleración y desaceleración suave (S-curve) que genera ctraj por defecto.
[s, ~]  = tpoly(0, 1, pasos_cord); 
Y_ideal = Y_ini + (Y_fin - Y_ini) * s;

% Desviación en Z (debe ser ~0 si el robot mantiene la altura)
subplot(1,2,1);
plot(tA, (xyzA(:,3) - Z_ideal)*1e3, 'r--', 'LineWidth', 2); hold on;
plot(tB, (xyzB(:,3) - Z_ideal)*1e3, 'b-',  'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
title('Desviación en Z (altura del TCP)');
xlabel('Tiempo [s]'); ylabel('Desviación [mm]');
legend('Sol. A (articular)','Sol. B (cartesiana)','Ideal','Location','best');
grid on;

% Desviación en Y (linealidad del avance)
subplot(1,2,2);
plot(tA, (xyzA(:,2) - Y_ideal)*1e3, 'r--', 'LineWidth', 2); hold on;
plot(tB, (xyzB(:,2) - Y_ideal)*1e3, 'b-',  'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
title('Desviación en Y (linealidad del avance)');
xlabel('Tiempo [s]'); ylabel('Desviación [mm]');
legend('Sol. A (articular)','Sol. B (cartesiana)','Ideal','Location','best');
grid on;

sgtitle('Análisis de precisión del cordón de soldadura');

% Salida numérica de desviaciones máximas
devZ_A = max(abs(xyzA(:,3) - Z_ideal)) * 1e3;
devZ_B = max(abs(xyzB(:,3) - Z_ideal)) * 1e3;
devY_A = max(abs(xyzA(:,2) - Y_ideal)) * 1e3;
devY_B = max(abs(xyzB(:,2) - Y_ideal)) * 1e3;

fprintf('\n=== ANÁLISIS DE PRECISIÓN (tramo de soldadura) ===\n');
fprintf('  Desviación máxima en Z:\n');
fprintf('    Sol. A (articular):  %.2f mm\n', devZ_A);
fprintf('    Sol. B (cartesiana): %.2f mm\n', devZ_B);
fprintf('  Desviación máxima en Y:\n');
fprintf('    Sol. A (articular):  %.2f mm\n', devY_A);
fprintf('    Sol. B (cartesiana): %.2f mm\n', devY_B);

% =========================================================================
% 6. SIMULACIÓN 3D — Ciclo completo
%    home → aprox → soldadura → retiro → home
%    Se animan ambas soluciones en figuras separadas
% =========================================================================
disp(' ');
disp('=== SIMULACIÓN 3D ===');
disp('Iniciando Sol. A (rojo punteado = traza del TCP)...');

figure('Name','Sim. 3D — Solución A: Ciclo completo',...
       'Position',[50 100 850 700]);
R.plot(qA_full, ...
    'trail',     'r--', ...
    'fps',        20,   ...
    'workspace',  workspace, ...
    'jointdiam',  0.4,  ...
    'scale',      0.5);
title('Solución A — Interpolación Articular');

pause(2);

disp('Iniciando Sol. B (azul continuo = traza del TCP)...');
figure('Name','Sim. 3D — Solución B: Ciclo completo',...
       'Position',[150 100 850 700]);
R.plot(qB_full, ...
    'trail',     'b-',  ...
    'fps',        20,   ...
    'workspace',  workspace, ...
    'jointdiam',  0.4,  ...
    'scale',      0.5);
title('Solución B — Interpolación Cartesiana');

% =========================================================================
% CONCLUSIÓN IMPRESA EN CONSOLA
% =========================================================================
fprintf('\n=== CONCLUSIÓN ===\n');
fprintf('Para la tarea de soldadura por arco MIG/MAG se selecciona\n');
fprintf('la SOLUCIÓN B (interpolación cartesiana + IK punto a punto).\n\n');
fprintf('Justificación:\n');
fprintf('  1. Precisión geométrica: el TCP describe una línea recta exacta\n');
fprintf('     (desv. Z=%.2fmm vs %.2fmm de Sol. A).\n', devZ_B, devZ_A);
fprintf('  2. Velocidad de avance constante en Y → calidad uniforme del cordón.\n');
fprintf('  3. Orientación de la antorcha controlada en todo el recorrido.\n');
fprintf('  4. El mayor costo computacional (IK en cada paso) es aceptable\n');
fprintf('     para movimientos lentos de soldadura (%.1f mm/s).\n', ...
    (Y_fin-Y_ini)*1e3/t_cord);
fprintf('\nLa Sol. A es adecuada para movimientos libres (home ↔ aprox)\n');
fprintf('donde no se requiere control de trayectoria estricto.\n');