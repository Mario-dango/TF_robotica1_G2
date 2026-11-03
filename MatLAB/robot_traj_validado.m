% =========================================================================
% SCRIPT: GENERACIÓN Y ANÁLISIS DE TRAYECTORIAS DE SOLDADURA
%         FANUC M-10iD/12 — Robótica 1 — Trabajo Final
%
% ESTRUCTURA DEL SCRIPT:
%   0. Setup: carga robot, herramienta, base y workspace desde robot.m
%   1. Definición de puntos clave en el espacio cartesiano
%   2. Solución A — Interpolación ARTICULAR  (jtraj)
%   3. Solución B — Interpolación CARTESIANA (ctraj)
%   4. Gráficos articulares: posición, velocidad, aceleración
%   5. Gráficos cartesianos: posición, velocidad, aceleración
%   6. Simulación 3D completa: home → aprox → soldadura → retiro → home
% =========================================================================

clear; clc; close all;

% -------------------------------------------------------------------------
% 0. SETUP — Carga de robot.m (incluye R, R.base, R.tool y workspace)
% -------------------------------------------------------------------------
disp('=== SETUP ===');

if ~exist('R', 'var') || ~exist('workspace', 'var')
    disp('Cargando definición del robot desde robot.m...');
    robot;   % ejecuta robot.m -> crea L(1..6), R y workspace
end

if exist('IK_numeric', 'file') ~= 2
    error(['No se encontró IK_numeric.m. Coloca ese archivo en la misma ', ...
           'carpeta que este script o agrégalo al MATLAB path.']);
end

T_base = double(R.base);
T_tool = double(R.tool);
disp('Setup completado desde robot.m.');
fprintf('  Base Z = %.2f m | Tool dX = %.1f mm, dZ = %.1f mm\n\n', ...
    T_base(3,4), T_tool(1,4)*1e3, T_tool(3,4)*1e3);

% -------------------------------------------------------------------------
% 1. DEFINICIÓN DE PUNTOS CLAVE
%    Todas las cotas son relativas al marco BASE del robot.
%    Cota Z absoluta (piso): Z_base + Z_relativo = 0.60 + 0.65 = 1.25 m
% -------------------------------------------------------------------------
disp('=== PUNTOS CLAVE ===');

% --- q_home: postura de reposo coherente con R.offset = [0 pi/2 0 0 0 0]
%     Brazo erguido y antorcha orientada hacia abajo en la misma rama de
%     muñeca que la trayectoria de trabajo (sin giros bruscos de muñeca).
q_home = [0, deg2rad(15), deg2rad(-30), deg2rad(160), deg2rad(-100), 0];

% --- Coordenadas del cordón relativas a la base del robot ---
X_cord  =  0.80;           % [m] distancia al frente (eje X base)
Y_ini   = -0.20;           % [m] extremo inicial del cordón
Y_fin   =  0.20;           % [m] extremo final del cordón (400 mm)
Z_cord  =  0.65;           % [m] altura de trabajo (Z rel. base)
Z_aprox =  Z_cord + 0.05;  % [m] 50 mm de clearance sobre el cordón

% Orientación de la antorcha:
%   - troty(pi): gira 180° en Y -> TCP apunta hacia abajo (-Z)
%   - trotx(deg2rad(15)): 15° de ángulo de avance (MIG/MAG)
ori = troty(pi) * trotx(deg2rad(15));

T_aprox  = T_base * transl(X_cord, Y_ini, Z_aprox) * ori;
T_inicio = T_base * transl(X_cord, Y_ini, Z_cord)  * ori;
T_fin    = T_base * transl(X_cord, Y_fin, Z_cord)  * ori;
T_retiro = T_base * transl(X_cord, Y_fin, Z_aprox) * ori;

% Semilla inicial coherente con la convención DH y R.offset de robot.m
% (cercana a la solución de T_aprox con la antorcha apuntando hacia abajo)
q_seed = [0, deg2rad(45), deg2rad(-60), deg2rad(160), deg2rad(-125), 0];

% Comprobar que la postura home sea admisible según R.qlim
if any(q_home < R.qlim(:,1)' - 1e-9) || any(q_home > R.qlim(:,2)' + 1e-9)
    error('La configuración q_home está fuera de los límites articulares R.qlim.');
end

% Resolver los puntos críticos en orden, encadenando la semilla
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

fprintf('  Validación IK de puntos críticos: correcta.\n');
fprintf('  Cordón: X=%.2fm, Y=[%.2f -> %.2f]m, Z=%.2fm\n', ...
    X_cord, Y_ini, Y_fin, Z_cord);
fprintf('  Longitud del cordón: %.0f mm\n', (Y_fin-Y_ini)*1e3);
fprintf('  Clearance de aproximación: %.0f mm\n\n', (Z_aprox-Z_cord)*1e3);

% -------------------------------------------------------------------------
% 2. PARÁMETROS TEMPORALES
% -------------------------------------------------------------------------
t_aprox   = 3;    % [s] home -> aprox
t_desc    = 1;    % [s] aprox -> inicio
t_cord    = 5;    % [s] inicio -> fin (soldadura)
t_retiro  = 1;    % [s] fin -> retiro
t_vuelta  = 3;    % [s] retiro -> home

pasos_aprox  = 30;
pasos_desc   = 10;
pasos_cord   = 50;
pasos_retiro = 10;
pasos_vuelta = 30;

tv_aprox  = linspace(0, t_aprox,  pasos_aprox);
tv_desc   = linspace(0, t_desc,   pasos_desc);
tv_cord   = linspace(0, t_cord,   pasos_cord);
tv_retiro = linspace(0, t_retiro, pasos_retiro);
tv_vuelta = linspace(0, t_vuelta, pasos_vuelta);

disp('=== TIEMPOS DE CICLO ===');
fprintf('  Home -> Aprox:   %.1f s\n', t_aprox);
fprintf('  Aprox -> Inicio: %.1f s\n', t_desc);
fprintf('  Soldadura:       %.1f s  (%.1f mm/s)\n', ...
    t_cord, (Y_fin-Y_ini)*1e3/t_cord);
fprintf('  Retiro:          %.1f s\n', t_retiro);
fprintf('  Retiro -> Home:  %.1f s\n', t_vuelta);
fprintf('  TIEMPO TOTAL:    %.1f s\n\n', ...
    t_aprox+t_desc+t_cord+t_retiro+t_vuelta);

velocidad_nominal_deg_s = [260 240 260 430 450 720];
margen_velocidad = 0.80;
lim_velocidad_rad_s = deg2rad(velocidad_nominal_deg_s * margen_velocidad);

% =========================================================================
% SOLUCIÓN A — INTERPOLACIÓN ARTICULAR (jtraj)
% =========================================================================
disp('=== SOLUCIÓN A: Interpolación Articular (jtraj) ===');

[qA_aprox,  qdA_aprox,  qddA_aprox]  = jtraj(q_home,   q_aprox,  tv_aprox);
[qA_desc,   qdA_desc,   qddA_desc]   = jtraj(q_aprox,  q_inicio, tv_desc);
[qA_cord,   qdA_cord,   qddA_cord]   = jtraj(q_inicio, q_fin,    tv_cord);
[qA_retiro, qdA_retiro, qddA_retiro] = jtraj(q_fin,    q_retiro, tv_retiro);
[qA_vuelta, qdA_vuelta, qddA_vuelta] = jtraj(q_retiro, q_home,   tv_vuelta);

validarTrayectoriaArticular(R, qA_aprox,  'Solución A: home -> aproximación');
validarTrayectoriaArticular(R, qA_desc,   'Solución A: aproximación -> inicio');
validarTrayectoriaArticular(R, qA_cord,   'Solución A: cordón');
validarTrayectoriaArticular(R, qA_retiro, 'Solución A: retiro');
validarTrayectoriaArticular(R, qA_vuelta, 'Solución A: regreso a home');

validarVelocidadesArticulares(qdA_aprox,  lim_velocidad_rad_s, 'Solución A: home -> aproximación');
validarVelocidadesArticulares(qdA_desc,   lim_velocidad_rad_s, 'Solución A: aproximación -> inicio');
validarVelocidadesArticulares(qdA_cord,   lim_velocidad_rad_s, 'Solución A: cordón');
validarVelocidadesArticulares(qdA_retiro, lim_velocidad_rad_s, 'Solución A: retiro');
validarVelocidadesArticulares(qdA_vuelta, lim_velocidad_rad_s, 'Solución A: regreso a home');

qA   = qA_cord;
qdA  = qdA_cord;
qddA = qddA_cord;
tA   = tv_cord;

qA_full = [qA_aprox; qA_desc; qA_cord; qA_retiro; qA_vuelta];
validarTrayectoriaArticular(R, qA_full, 'Solución A: ciclo completo');

% Cinemática directa para obtener posición cartesiana del TCP
Tm_A = double(R.fkine(qA));
xyzA = squeeze(Tm_A(1:3, 4, :))';

% Velocidad y aceleración cartesiana por diferenciación numérica (a lo largo de filas = tiempo)
dt = tA(2) - tA(1);
[~, vxyzA] = gradient(xyzA,  dt);
[~, axyzA] = gradient(vxyzA, dt);

disp('  Solución A calculada.');

% =========================================================================
% SOLUCIÓN B — INTERPOLACIÓN CARTESIANA (ctraj)
% =========================================================================
disp('=== SOLUCIÓN B: Interpolación Cartesiana (ctraj) ===');

T_tray_B = ctraj(T_inicio, T_fin, pasos_cord);

[ok, qB, msg] = IK_numeric(R, T_tray_B, true, q_inicio);
if ~ok
    error('Trayectoria cartesiana rechazada por IK_numeric: %s', msg);
end

if size(qB,1) ~= pasos_cord || size(qB,2) ~= 6
    error('IK_numeric devolvió una matriz articular con dimensiones inesperadas.');
end
validarTrayectoriaArticular(R, qB, 'Solución B: cordón cartesiano');

% Velocidad y aceleración articular derivando respecto al tiempo (filas: 2da salida de gradient)
dt = tv_cord(2) - tv_cord(1);
[~, qdB]  = gradient(qB,  dt);
[~, qddB] = gradient(qdB, dt);
validarVelocidadesArticulares(qdB, lim_velocidad_rad_s, 'Solución B: cordón cartesiano');

tB = tv_cord;

% Comparar la trayectoria PEDIDA con la que realmente alcanza el robot
T_tray_B_num = double(T_tray_B);
Tm_B         = double(R.fkine(qB));
xyzB_ref     = zeros(pasos_cord, 3);
xyzB         = zeros(pasos_cord, 3);
error_pose_B = zeros(pasos_cord, 1);

for i = 1:pasos_cord
    T_objetivo      = T_tray_B_num(:,:,i);
    T_real          = Tm_B(:,:,i);
    xyzB_ref(i,:)   = T_objetivo(1:3, 4)';
    xyzB(i,:)       = T_real(1:3, 4)';
    error_pose_B(i) = norm(T_real - T_objetivo, 'fro');
end

error_TCP_B_max = max(error_pose_B);
if error_TCP_B_max > 1e-3
    error('La FK no reproduce la trayectoria cartesiana. Error máximo: %.6g', error_TCP_B_max);
end

% Velocidad y aceleración cartesiana derivando respecto al tiempo (filas)
[~, vxyzB] = gradient(xyzB,  dt);
[~, axyzB] = gradient(vxyzB, dt);

% Tramos libres articulares para el ciclo completo de la Solución B
[qB_aprox,  qdB_aprox,  ~] = jtraj(q_home,   q_aprox,  tv_aprox);
[qB_desc,   qdB_desc,   ~] = jtraj(q_aprox,  q_inicio, tv_desc);

q_fin_B = qB(end,:);
[ok, q_tmp, msg] = IK_numeric(R, T_retiro, true, q_fin_B);
if ~ok, error('CI de retiro para Solución B rechazada por IK_numeric: %s', msg); end
q_retiro_B = q_tmp(end,:);

[qB_retiro, qdB_retiro, ~] = jtraj(q_fin_B,    q_retiro_B, tv_retiro);
[qB_vuelta, qdB_vuelta, ~] = jtraj(q_retiro_B, q_home,     tv_vuelta);

validarTrayectoriaArticular(R, qB_aprox,  'Solución B: home -> aproximación');
validarTrayectoriaArticular(R, qB_desc,   'Solución B: aproximación -> inicio');
validarTrayectoriaArticular(R, qB_retiro, 'Solución B: retiro');
validarTrayectoriaArticular(R, qB_vuelta, 'Solución B: regreso a home');

validarVelocidadesArticulares(qdB_aprox,  lim_velocidad_rad_s, 'Solución B: home -> aproximación');
validarVelocidadesArticulares(qdB_desc,   lim_velocidad_rad_s, 'Solución B: aproximación -> inicio');
validarVelocidadesArticulares(qdB_retiro, lim_velocidad_rad_s, 'Solución B: retiro');
validarVelocidadesArticulares(qdB_vuelta, lim_velocidad_rad_s, 'Solución B: regreso a home');

qB_full = [qB_aprox; qB_desc; qB; qB_retiro; qB_vuelta];
validarTrayectoriaArticular(R, qB_full, 'Solución B: ciclo completo');

disp('  Solución B calculada.');

% =========================================================================
% 4. GRÁFICOS ARTICULARES — Posición, Velocidad, Aceleración
% =========================================================================
disp('=== Generando gráficos articulares... ===');

etiquetas_q = {'q_1','q_2','q_3','q_4','q_5','q_6'};
colores = lines(6);

figure('Name','Perfiles Articulares — Solución A (jtraj)',...
       'Position',[50 50 1100 750]);

subplot(3,1,1); hold on;
for j = 1:6
    plot(tA, rad2deg(qA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Posición articular');
ylabel('Ángulo [°]'); grid on; xlim([tA(1) tA(end)]);

subplot(3,1,2); hold on;
for j = 1:6
    plot(tA, rad2deg(qdA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Velocidad articular');
ylabel('[°/s]'); grid on; xlim([tA(1) tA(end)]);

subplot(3,1,3); hold on;
for j = 1:6
    plot(tA, rad2deg(qddA(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. A — Aceleración articular');
xlabel('Tiempo [s]'); ylabel('[°/s²]'); grid on; xlim([tA(1) tA(end)]);

figure('Name','Perfiles Articulares — Solución B (ctraj + IK)',...
       'Position',[200 50 1100 750]);

subplot(3,1,1); hold on;
for j = 1:6
    plot(tB, rad2deg(qB(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. B — Posición articular');
ylabel('Ángulo [°]'); grid on; xlim([tB(1) tB(end)]);

subplot(3,1,2); hold on;
for j = 1:6
    plot(tB, rad2deg(qdB(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5);
end
legend(etiquetas_q, 'Location','eastoutside');
title('Sol. B — Velocidad articular');
ylabel('[°/s]'); grid on; xlim([tB(1) tB(end)]);

subplot(3,1,3); hold on;
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

eje_labels  = {'X [m]', 'Y [m]', 'Z [m]'};
titulos_pos = {'Posición X del TCP','Posición Y del TCP','Posición Z del TCP'};
titulos_vel = {'Velocidad X','Velocidad Y','Velocidad Z'};
titulos_acc = {'Aceleración X','Aceleración Y','Aceleración Z'};

figure('Name','Perfiles Cartesianos — Comparativa A vs B',...
       'Position',[350 50 1100 850]);

for k = 1:3
    subplot(3,3,k);
    plot(tA, xyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, xyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_pos{k});
    ylabel(eje_labels{k}); grid on; xlim([tA(1) tA(end)]);
    if k==1, legend('Sol. A','Sol. B','Location','best'); end

    subplot(3,3,3+k);
    plot(tA, vxyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, vxyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_vel{k});
    ylabel(['d' eje_labels{k}(1) '/dt [m/s]']); grid on; xlim([tA(1) tA(end)]);

    subplot(3,3,6+k);
    plot(tA, axyzA(:,k), 'r--', 'LineWidth',1.5); hold on;
    plot(tB, axyzB(:,k), 'b-',  'LineWidth',1.5);
    title(titulos_acc{k});
    xlabel('Tiempo [s]');
    ylabel(['d²' eje_labels{k}(1) '/dt² [m/s²]']); grid on; xlim([tA(1) tA(end)]);
end
sgtitle('Espacio Cartesiano: Sol. A (rojo--) vs Sol. B (azul-)');

figure('Name','Desviación del TCP respecto al cordón ideal',...
       'Position',[500 50 900 500]);

Z_ideal = xyzB_ref(:,3);
Y_ideal = xyzB_ref(:,2);

subplot(1,2,1);
plot(tA, (xyzA(:,3) - Z_ideal)*1e3, 'r--', 'LineWidth', 2); hold on;
plot(tB, (xyzB(:,3) - Z_ideal)*1e3, 'b-',  'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
title('Desviación en Z (altura del TCP)');
xlabel('Tiempo [s]'); ylabel('Desviación [mm]');
legend('Sol. A (articular)','Sol. B (cartesiana)','Ideal','Location','best');
grid on;

subplot(1,2,2);
plot(tA, (xyzA(:,2) - Y_ideal)*1e3, 'r--', 'LineWidth', 2); hold on;
plot(tB, (xyzB(:,2) - Y_ideal)*1e3, 'b-',  'LineWidth', 2);
yline(0, 'k:', 'LineWidth', 1);
title('Desviación en Y (linealidad del avance)');
xlabel('Tiempo [s]'); ylabel('Desviación [mm]');
legend('Sol. A (articular)','Sol. B (cartesiana)','Ideal','Location','best');
grid on;

sgtitle('Análisis de precisión del cordón de soldadura');

devZ_A = max(abs(xyzA(:,3) - Z_ideal)) * 1e3;
devZ_B = max(abs(xyzB(:,3) - Z_ideal)) * 1e3;
devY_A = max(abs(xyzA(:,2) - Y_ideal)) * 1e3;
devY_B = max(abs(xyzB(:,2) - Y_ideal)) * 1e3;

p0 = xyzB_ref(1,:);
direccion = xyzB_ref(end,:) - p0;
longitud2 = dot(direccion, direccion);
if longitud2 < eps
    error('La trayectoria cartesiana de referencia tiene longitud nula.');
end
sA = ((xyzA - p0) * direccion') / longitud2;
sB = ((xyzB - p0) * direccion') / longitud2;
proyeccionA = p0 + sA .* direccion;
proyeccionB = p0 + sB .* direccion;
dev_linea_A = max(vecnorm(xyzA - proyeccionA, 2, 2)) * 1e3;
dev_linea_B = max(vecnorm(xyzB - proyeccionB, 2, 2)) * 1e3;

fprintf('\n=== ANÁLISIS DE PRECISIÓN (tramo de soldadura) ===\n');
fprintf('  Desviación máxima en Z:\n');
fprintf('    Sol. A (articular):  %.2f mm\n', devZ_A);
fprintf('    Sol. B (cartesiana): %.2f mm\n', devZ_B);
fprintf('  Desviación máxima en Y:\n');
fprintf('    Sol. A (articular):  %.2f mm\n', devY_A);
fprintf('    Sol. B (cartesiana): %.2f mm\n', devY_B);
fprintf('  Desviación geométrica máxima respecto a la recta:\n');
fprintf('    Sol. A (articular):  %.3f mm\n', dev_linea_A);
fprintf('    Sol. B (cartesiana): %.3f mm\n', dev_linea_B);
fprintf('  Error máximo FK vs. pose objetivo de Sol. B: %.6g\n', error_TCP_B_max);

% =========================================================================
% 6. SIMULACIÓN 3D — Ciclo completo
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
% FUNCIONES LOCALES DE VALIDACIÓN
% =========================================================================
function validarTrayectoriaArticular(R, q, nombre)
    if isempty(q) || size(q,2) ~= 6 || any(~isfinite(q), 'all')
        error('%s: trayectoria vacía, con dimensiones incorrectas o NaN/Inf.', nombre);
    end

    tolerancia = 1e-8;
    fuera = q < (R.qlim(:,1)' - tolerancia) | q > (R.qlim(:,2)' + tolerancia);
    if any(fuera, 'all')
        [fila, articulacion] = find(fuera, 1, 'first');
        error('%s: q%d fuera de límites en la muestra %d (%.4f rad).', ...
            nombre, articulacion, fila, q(fila, articulacion));
    end

    if size(q,1) > 1
        salto = max(abs(diff(q)), [], 'all');
        if salto > 0.5
            error('%s: salto articular de %.4f rad entre muestras (máximo permitido 0.5 rad).', ...
                nombre, salto);
        end
    end
end

function validarVelocidadesArticulares(qd, limites, nombre)
    if isempty(qd) || size(qd,2) ~= 6 || any(~isfinite(qd), 'all')
        error('%s: velocidades vacías, con dimensiones incorrectas o NaN/Inf.', nombre);
    end

    vmax = max(abs(qd), [], 1);
    excede = vmax > limites;
    if any(excede)
        articulaciones = find(excede);
        detalle = '';
        for k = 1:numel(articulaciones)
            j = articulaciones(k);
            detalle = [detalle, sprintf(' J%d: %.1f°/s > %.1f°/s;', ...
                j, rad2deg(vmax(j)), rad2deg(limites(j)))]; %#ok<AGROW>
        end
        error('%s: se exceden límites de velocidad articular.%s Aumenta la duración del tramo.', ...
            nombre, detalle);
    end
end