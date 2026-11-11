%% =====================================================================
%  TRAYECTORIA DE SOLDADURA EN "L" - FANUC M-10iD/12
%
%    Traslados en el aire  -> interpolación ARTICULAR  (jtraj)
%       q_home -> aprox -> A        y        B -> retiro -> q_home
%    Soldadura en la L     -> interpolación CARTESIANA (ctraj) + IK_numeric
%       1) tramo largo  A -> V : recta, orientación constante
%       2) vértice      V      : giro del soplete con el TCP quieto
%       3) tramo corto  V -> B : recta, orientación constante
%
%  ESTRUCTURA DEL SCRIPT:
%    0. Setup y parámetros         6. Validaciones y retorno a home
%    1. Geometría de la L          7. Cinemática directa del TCP
%    2. Leyes temporales           8. Métricas de la soldadura
%    3. Trayectoria cartesiana     9. Gráficos de perfiles y precisión
%    4. Cinemática inversa        10. Tabla resumen
%    5. Armado del ciclo          11. Simulación 3D (robot + traza del TCP)
%
%  Requiere: Robotics Toolbox, robot.m, IK_numeric.m
%  IK_numeric: [exito, q, msg] = IK_numeric(R, T, q_mejor, q0)
% =====================================================================
clear; clc; close all;
rng(1);                               % semillas reproducibles para IK_numeric

%% ---------------------------------------------------------------
%  0) SETUP Y PARÁMETROS
% ---------------------------------------------------------------
robot;                                % crea R (con base, tool y qlim) y workspace
if exist('IK_numeric', 'file') ~= 2
    error('No se encontró IK_numeric.m en el path de MATLAB.');
end

q_home = deg2rad([0, -20, -10, 0, 70, 0]);      % reposo (evita singularidad de muñeca)
if any(q_home < R.qlim(:,1)') || any(q_home > R.qlim(:,2)')
    error('q_home está fuera de los límites articulares R.qlim.');
end

% --- Muestreo y tiempos ---
dt          = 0.04;     % [s] período de muestreo (25 Hz)
v_soldadura = 0.010;    % [m/s] velocidad de avance (10 mm/s = 600 mm/min)
t_blend     = 1.0;      % [s] rampa de aceleración/frenado en cada extremo de cada tramo
t_giro      = 2.0;      % [s] duración del giro del soplete en el vértice
t_aprox     = 4.0;      % [s] home   -> aproximación          (jtraj)
t_desc      = 1.5;      % [s] aprox  -> A (descenso)          (jtraj)
t_retiro    = 1.5;      % [s] B      -> retiro (ascenso)      (jtraj)
t_vuelta    = 4.0;      % [s] retiro -> home                  (jtraj)

% --- Decisión de diseño en el vértice ---
% true : el soplete gira 90 deg en el vértice (con el TCP quieto) para mantener
%        el ángulo de avance respecto del sentido de marcha en AMBOS tramos.
% false: orientación constante en toda la L (sin giro; en el tramo corto el
%        ángulo de avance pasa a ser una inclinación lateral).
reorientar_vertice = true;
giro_vertice = deg2rad(90);     % [rad] giro del sentido de marcha (+90 deg: el tramo corto va hacia -X)

% --- Tolerancias de proceso ---
tol_lateral_mm = 0.5;   % [mm]  desviación máxima respecto de la unión
tol_orient_deg = 2.0;   % [deg] desviación máxima de orientación respecto de la referencia
tol_vel_pct    = 5.0;   % [%]   variación de velocidad admisible en las mesetas

% --- Límites de velocidad: datasheet FANUC M-10iD/8L [deg/s], con 20% de margen ---
% (si su robot es otra variante, ajuste estos valores)
vel_max_deg         = [210 210 220 430 450 720];
lim_velocidad_rad_s = deg2rad(vel_max_deg * 0.80);

%% ---------------------------------------------------------------
%  1) GEOMETRÍA DE LA L (coordenadas del MUNDO; la base está a z = 0.6 m)
%        A ----(tramo largo 400 mm, +Y)----> V
%                                            |
%                       B <--(tramo corto 200 mm)
%     Zona verificada: lejos de singularidades y de límites articulares.
% ---------------------------------------------------------------
L_largo   = 0.40;                       % [m] tramo largo
L_corto   = 0.20;                       % [m] tramo corto
Z_cord    = 0.70;                       % [m] altura de la unión
clearance = 0.05;                       % [m] altura de aproximación/retiro

if ~reorientar_vertice, giro_vertice = 0; t_giro = 0; end

p_V = [1.20, 0.20, Z_cord];             % vértice
p_A = p_V - [0, L_largo, 0];            % inicio (tramo largo paralelo a Y)
Rz  = @(a) [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
dir_corto = (Rz(giro_vertice) * [0; 1; 0])';      % sentido de marcha del tramo corto
if ~reorientar_vertice, dir_corto = [-1 0 0]; end % sin giro: tramo corto hacia -X igualmente
p_B = p_V + L_corto * dir_corto;        % final

% Orientación del soplete: alambre hacia abajo, 10 deg de ángulo de avance (push)
% hacia el sentido de marcha. En el tramo 2 la terna se gira 'giro_vertice' en torno a z.
angulo_avance = deg2rad(10);
z_t = [0; sin(angulo_avance); -cos(angulo_avance)];
x_t = [1; 0; 0];
R1  = [x_t, cross(z_t, x_t), z_t];      % orientación del tramo 1 (marcha +Y)
R2  = Rz(giro_vertice) * R1;            % orientación del tramo 2

pose = @(Rot, p) [Rot, p(:); 0 0 0 1];
T_aprox  = pose(R1, p_A + [0 0 clearance]);
T_A      = pose(R1, p_A);
T_V1     = pose(R1, p_V);               % vértice con la orientación del tramo 1
T_V2     = pose(R2, p_V);               % vértice con la orientación del tramo 2
T_B      = pose(R2, p_B);
T_retiro = pose(R2, p_B + [0 0 clearance]);

%% ---------------------------------------------------------------
%  2) LEYES TEMPORALES (cada etapa parte y termina en reposo)
%     tramos: trapecio LSPB (rampa + velocidad CONSTANTE + rampa)
%     giro  : polinomio quíntico (aceleración suave, sin golpes)
% ---------------------------------------------------------------
T_1 = round((L_largo/v_soldadura + t_blend) / dt) * dt;
T_2 = round((L_corto/v_soldadura + t_blend) / dt) * dt;
t_1 = vector_tiempo(T_1, dt);   N_1 = numel(t_1);
t_2 = vector_tiempo(T_2, dt);   N_2 = numel(t_2);
s_1 = lspb(0, 1, t_1, 1/(T_1 - t_blend));
s_2 = lspb(0, 1, t_2, 1/(T_2 - t_blend));
if reorientar_vertice
    t_g = vector_tiempo(t_giro, dt);   N_g = numel(t_g);
    s_g = tpoly(0, 1, t_g);
else
    N_g = 1;
end

disp('=== PLAN DEL CICLO ===');
fprintf('  %-26s %-8s %6.1f s\n', 'Home -> aproximación',    'jtraj', t_aprox);
fprintf('  %-26s %-8s %6.1f s\n', 'Aproximación -> A',       'jtraj', t_desc);
fprintf('  %-26s %-8s %6.1f s\n', 'Soldadura: tramo largo',  'ctraj', T_1);
if reorientar_vertice
    fprintf('  %-26s %-8s %6.1f s\n', 'Soldadura: giro en V',  'ctraj', t_giro);
end
fprintf('  %-26s %-8s %6.1f s\n', 'Soldadura: tramo corto',  'ctraj', T_2);
fprintf('  %-26s %-8s %6.1f s\n', 'B -> retiro',             'jtraj', t_retiro);
fprintf('  %-26s %-8s %6.1f s\n', 'Retiro -> home',          'jtraj', t_vuelta);
fprintf('  Meseta de soldadura a %.2f mm/s | ciclo total %.1f s\n\n', ...
        L_largo/(T_1 - t_blend)*1000, t_aprox + t_desc + T_1 + t_giro + T_2 + t_retiro + t_vuelta);

%% ---------------------------------------------------------------
%  3) TRAYECTORIA CARTESIANA DE LA SOLDADURA (ctraj en tres etapas)
%     El giro del vértice es una interpolación con la MISMA posición y
%     orientación distinta (rotación pura en torno al TCP).
% ---------------------------------------------------------------
T_leg1 = ctraj_seguro(T_A,  T_V1, s_1);          % 4x4xN_1
if reorientar_vertice
    T_giro = ctraj_seguro(T_V1, T_V2, s_g);      % 4x4xN_g
else
    T_giro = T_V1;                               % sin giro: una sola muestra
end
T_leg2 = ctraj_seguro(T_V2, T_B,  s_2);          % 4x4xN_2

T_L = cat(3, T_leg1, T_giro(:,:,2:end), T_leg2(:,:,2:end));   % secuencia completa
N_L = size(T_L, 3);
i_V1 = N_1;                    % muestra (dentro de la L) en que se llega al vértice
i_V2 = N_1 + N_g - 1;          % muestra en que se termina el giro
traj_L = SE3(T_L);

%% ---------------------------------------------------------------
%  4) CINEMÁTICA INVERSA CON IK_numeric
%     Se prueban las configuraciones del punto de aproximación (ordenadas
%     por cercanía a q_home) hasta que una recorre la L completa. IK_numeric
%     valida convergencia, límites articulares y saltos entre muestras.
% ---------------------------------------------------------------
cand = candidatos_IK(R, T_aprox);
[~, orden] = sort(sqrt(sum((cand - q_home).^2, 2)));
cand = cand(orden, :);
fprintf('Configuraciones IK en el punto de aproximación: %d\n', size(cand,1));
fprintf('Resolviendo IK de %d poses (puede tardar unos segundos)...\n', N_L);

configuracion_ok = false;
for k = 1:size(cand,1)
    q_aprox = cand(k,:);
    [ok1, q_inicio] = resolver_pose(R, T_A, q_aprox);                    % descenso a A
    if ~ok1, continue; end
    [ok2, q_L, msg] = IK_numeric(R, traj_L, true, q_inicio);             % la L completa
    if ~ok2
        fprintf('  Configuración %d descartada en la L: %s\n', k, msg);
        continue;
    end
    [ok3, q_retiro] = resolver_pose(R, T_retiro, q_L(end,:));            % ascenso desde B
    if ~ok3, continue; end
    configuracion_ok = true;
    break;
end
if ~configuracion_ok
    error('Ninguna configuración permite recorrer la L completa. Revise la geometría o q_home.');
end
fprintf('  q_aprox  = [%s] deg\n  q_inicio = [%s] deg\n  q_final  = [%s] deg\n\n', ...
        num2str(rad2deg(q_aprox),'%8.2f'), num2str(rad2deg(q_inicio),'%8.2f'), ...
        num2str(rad2deg(q_L(end,:)),'%8.2f'));

%% ---------------------------------------------------------------
%  5) ARMADO DEL CICLO COMPLETO
%     jtraj en los traslados (aire) + q_L (ctraj + IK) en la soldadura
% ---------------------------------------------------------------
tv_aprox = vector_tiempo(t_aprox, dt);
tv_desc  = vector_tiempo(t_desc,  dt);
tv_ret   = vector_tiempo(t_retiro, dt);
tv_vuel  = vector_tiempo(t_vuelta, dt);

q_seg_aprox = jtraj(q_home,     q_aprox,  tv_aprox);     % home    -> aproximación
q_seg_desc  = jtraj(q_aprox,    q_inicio, tv_desc);      % aprox   -> A
q_seg_ret   = jtraj(q_L(end,:), q_retiro, tv_ret);       % B       -> retiro
q_seg_vuel  = jtraj(q_retiro,   q_home,   tv_vuel);      % retiro  -> home

q_ciclo = unir(unir(unir(unir(q_seg_aprox, q_seg_desc), q_L), q_seg_ret), q_seg_vuel);

i0       = numel(tv_aprox) + numel(tv_desc) - 1;      % primera muestra de la L en el ciclo
idx_sold = i0 : (i0 + N_L - 1);
idx_vert = [i0 + i_V1 - 1, i0 + i_V2 - 1];            % [llegada al vértice, fin del giro]
t_total  = (0:size(q_ciclo,1)-1)' * dt;

%% ---------------------------------------------------------------
%  6) VALIDACIONES (límites, continuidad, velocidad articular y retorno a home)
% ---------------------------------------------------------------
[qd, qdd] = derivadas(q_ciclo, dt);
validarTrayectoriaArticular(R, q_ciclo, 'Ciclo completo');
validarVelocidadesArticulares(qd, lim_velocidad_rad_s, 'Ciclo completo');
disp('Validaciones articulares (límites, saltos y velocidades): correctas.');
fprintf('Retorno a q_home (jtraj): error final = %.1e rad\n', max(abs(q_ciclo(end,:) - q_home)));

%% ---------------------------------------------------------------
%  7) CINEMÁTICA DIRECTA: MOVIMIENTO REAL DEL TCP
% ---------------------------------------------------------------
[p_tcp, Rot_tcp] = cinematica_directa(R, q_ciclo);
[pd_tcp, pdd_tcp] = derivadas(p_tcp, dt);

% La trayectoria articular debe reproducir la referencia cartesiana (error de pose por FK)
Tm = double(R.fkine(q_L));
err_FK = 0;
for i = 1:N_L
    err_FK = max(err_FK, norm(Tm(:,:,i) - T_L(:,:,i), 'fro'));
end
fprintf('Error máximo FK vs referencia cartesiana (ctraj): %.3g\n', err_FK);

%% ---------------------------------------------------------------
%  8) MÉTRICAS DE LA SOLDADURA
% ---------------------------------------------------------------
[desv, v_tcp, ori_err, epos] = metricas_L(p_tcp(idx_sold,:), pd_tcp(idx_sold,:), ...
                                           Rot_tcp(:,:,idx_sold), T_L, p_A, p_V, p_B);
t_c = t_total(idx_sold) - t_total(idx_sold(1));

% Mesetas de velocidad constante de cada tramo (se excluyen las rampas)
mes = false(N_L, 1);
mes(1:N_1)    = t_1 > t_blend & t_1 < T_1 - t_blend;
mes(i_V2:N_L) = t_2 > t_blend & t_2 < T_2 - t_blend;
% Ventana de ±1 s alrededor del vértice
n1s = round(1/dt);
ven = max(1, i_V1 - n1s) : min(N_L, i_V2 + n1s);

var_vel = 100*(max(v_tcp(mes)) - min(v_tcp(mes))) / v_soldadura;
w_sold  = manipulabilidad(R, q_ciclo(idx_sold,:));

%% ---------------------------------------------------------------
%  9) GRÁFICOS
%     Zona celeste: soldadura (ctraj). Resto: traslados (jtraj).
%     Líneas magenta: llegada al vértice y fin del giro del soplete.
% ---------------------------------------------------------------
disp('=== Generando gráficos... ===');
graficar_articular(t_total, q_ciclo, qd, qdd, idx_sold, idx_vert);
graficar_cartesiano(t_total, p_tcp, pd_tcp, pdd_tcp, idx_sold, idx_vert);

figure('Name','Análisis de precisión de la soldadura en L', 'Position',[500 50 1000 700]);

subplot(2,2,1); hold on; grid on; axis equal;
plot([p_A(2) p_V(2) p_B(2)], [p_A(1) p_V(1) p_B(1)], 'k:', 'LineWidth', 1.5);
plot(p_tcp(idx_sold,2), p_tcp(idx_sold,1), 'b-', 'LineWidth', 1.5);
plot(p_V(2), p_V(1), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);
xlabel('Y [m]'); ylabel('X [m]'); title('Vista superior del TCP en la L');
legend('Unión ideal','Trayectoria del TCP','Vértice','Location','best');

subplot(2,2,2); hold on; grid on;
plot(t_c, desv, 'b-', 'LineWidth', 1.5);
yline(tol_lateral_mm, 'k:', 'LineWidth', 1);
xline(t_c(i_V1), 'm:', 'HandleVisibility', 'off'); xline(t_c(i_V2), 'm:', 'HandleVisibility', 'off');
xlabel('Tiempo [s]'); ylabel('Desviación [mm]');
title('Distancia del TCP a la unión'); legend('TCP','Tolerancia','Location','best');

subplot(2,2,3); hold on; grid on;
plot(t_c, v_tcp*1000, 'b-', 'LineWidth', 1.5);
yline(v_soldadura*1000, 'k:', 'LineWidth', 1);
xline(t_c(i_V1), 'm:', 'HandleVisibility', 'off'); xline(t_c(i_V2), 'm:', 'HandleVisibility', 'off');
xlabel('Tiempo [s]'); ylabel('|v_{TCP}| [mm/s]');
title('Rapidez del TCP'); legend('TCP','Consigna','Location','best');

subplot(2,2,4); hold on; grid on;
plot(t_c, ori_err, 'b-', 'LineWidth', 1.5);
yline(tol_orient_deg, 'k:', 'LineWidth', 1);
xline(t_c(i_V1), 'm:', 'HandleVisibility', 'off'); xline(t_c(i_V2), 'm:', 'HandleVisibility', 'off');
xlabel('Tiempo [s]'); ylabel('Error [°]');
title('Orientación del soplete vs referencia'); legend('TCP','Tolerancia','Location','best');

sgtitle('Análisis de precisión del cordón de soldadura en L');

%% ---------------------------------------------------------------
%  10) TABLA RESUMEN
% ---------------------------------------------------------------
cumple_lat = max(desv)    <= tol_lateral_mm;
cumple_ori = max(ori_err) <= tol_orient_deg;
cumple_vel = var_vel      <= tol_vel_pct;
etq = {'NO CUMPLE', 'CUMPLE'};
txt = @(b) etq{double(b) + 1};      % texto del veredicto

fprintf('\n=============== RESUMEN DE LA SOLDADURA EN L ===============\n');
fprintf('%-44s %10.3f mm   (tol. %.1f) %s\n', 'Desviación a la unión, toda la L', max(desv), tol_lateral_mm, txt(cumple_lat));
fprintf('%-44s %10.3f mm\n', 'Desviación a la unión, vértice ±1 s', max(desv(ven)));
fprintf('%-44s %10.2f deg  (tol. %.1f) %s\n', 'Error de orientación máximo', max(ori_err), tol_orient_deg, txt(cumple_ori));
fprintf('%-44s %10.2f %%    (tol. %.1f) %s\n', 'Variación de velocidad en las mesetas', var_vel, tol_vel_pct, txt(cumple_vel));
fprintf('%-44s %10.2f mm/s\n', 'Rapidez en mesetas (mín.)', min(v_tcp(mes))*1000);
fprintf('%-44s %10.2f mm/s\n', 'Rapidez en mesetas (máx.)', max(v_tcp(mes))*1000);
fprintf('%-44s %10.3f\n',    'Manipulabilidad mínima en la soldadura', min(w_sold));
fprintf('%-44s %10.3f\n',    'Vel. articular máx. / límite datasheet (ciclo)', max(abs(rad2deg(qd)) ./ vel_max_deg, [], 'all'));

% Tiempo con la antorcha lenta (< 50% de la consigna) en torno al vértice: indicador de
% aporte térmico localizado (el aporte por unidad de longitud crece como 1/v).
t_lento = sum(v_tcp(ven) < 0.5*v_soldadura) * dt;
fprintf('\nTiempo con v < 50%% de la consigna en torno al vértice: %.1f s\n', t_lento);
fprintf('(se reduce con menor t_giro / t_blend, o con reorientar_vertice = false)\n');

%% ---------------------------------------------------------------
%  11) SIMULACIÓN 3D — Ciclo completo: home -> aprox -> L -> retiro -> home
%      Se muestra sólo el robot y la traza del TCP (azul continuo).
% ---------------------------------------------------------------
simular  = true;                          % false para omitir la simulación 3D
paso_sim = 3;                             % se dibuja 1 de cada 'paso_sim' muestras
ws_sim   = [-0.5 2.0 -1.0 1.0 0 2.0];     % volumen de visualización (use 'workspace' para la vista amplia)

if simular
    disp(' ');
    disp('=== SIMULACIÓN 3D ===');
    disp('Iniciando simulación (azul continuo = traza del TCP)...');
    n_ciclo = size(q_ciclo, 1);
    k_sim   = unique([1:paso_sim:n_ciclo, n_ciclo]);   % siempre incluye la muestra final (q_home)

    figure('Name','Sim. 3D — Ciclo completo', 'Position',[50 100 850 700]);
    R.plot(q_ciclo(k_sim,:), ...
        'trail',     'b-',  ...
        'fps',        20,   ...
        'workspace',  ws_sim, ...
        'jointdiam',  0.4,  ...
        'scale',      0.5);
    title('Soldadura en L — jtraj (traslados) + ctraj (cordón)');
end


%% =====================================================================
%  FUNCIONES LOCALES
% =====================================================================

function tv = vector_tiempo(T, dt)
% Vector de tiempos columna de 0 a T con paso dt (T debe ser múltiplo de dt)
    tv = linspace(0, T, round(T/dt) + 1)';
end

function q = unir(q1, q2)
% Concatena dos tramos articulares eliminando la muestra repetida del empalme
    q = [q1; q2(2:end,:)];
end

function T = ctraj_seguro(T0, T1, s)
% Interpolación cartesiana T0->T1 con escalar s(t) en [0,1]. Usa ctraj; si el toolbox
% falla (clase Quaternion), usa una interpolación equivalente: posición lineal y
% rotación por eje-ángulo (equivalente a SLERP). Devuelve un arreglo 4x4xN numérico.
    persistent avisado
    s = s(:);
    try
        T = double(ctraj(SE3(T0), SE3(T1), s));
        if size(T,3) ~= numel(s), error('dimensión inesperada de ctraj'); end
    catch ME
        if isempty(avisado)
            warning('ctraj no disponible (%s). Se usa interpolación equivalente.', ...
                    strtok(ME.message, newline));
            avisado = true;
        end
        R0 = T0(1:3,1:3);
        W  = real(logm(R0' * T1(1:3,1:3)));          % rotación relativa (matriz antisimétrica)
        p0 = T0(1:3,4);  p1 = T1(1:3,4);
        T  = repmat(eye(4), [1, 1, numel(s)]);
        for i = 1:numel(s)
            T(1:3,1:3,i) = R0 * expm(s(i) * W);
            T(1:3,4,i)   = p0 + s(i) * (p1 - p0);
        end
    end
end

function cand = candidatos_IK(R, T_pose)
% Todas las configuraciones distintas que alcanzan T_pose (IK_numeric, q_mejor = false)
    [exito, soluciones, msg] = IK_numeric(R, SE3(T_pose), false, zeros(1,6));
    if ~exito || isempty(soluciones)
        error('IK_numeric no encontró soluciones: %s', msg);
    end
    cand = zeros(numel(soluciones), 6);
    for k = 1:numel(soluciones)
        cand(k,:) = soluciones{k}(1,:);
    end
end

function [ok, q] = resolver_pose(R, T_pose, q_semilla)
% Una sola pose con IK_numeric (q_mejor = true), partiendo de q_semilla
    [ok, q_tmp, ~] = IK_numeric(R, SE3(T_pose), true, q_semilla);
    if ok, q = q_tmp(end,:); else, q = []; end
end

function [p, Rot] = cinematica_directa(R, q)
% Posición p (Nx3) y rotación Rot (3x3xN) del TCP para q (Nx6)
    Tm  = double(R.fkine(q));
    p   = squeeze(Tm(1:3,4,:))';
    Rot = Tm(1:3,1:3,:);
end

function [d1, d2] = derivadas(x, dt)
% Primera y segunda derivada numérica por columnas
    d1 = gradient(x', dt)';
    d2 = gradient(d1', dt)';
end

function w = manipulabilidad(R, q)
% Índice de manipulabilidad de Yoshikawa en cada configuración de q (Nx6)
    w = zeros(size(q,1), 1);
    for i = 1:size(q,1)
        w(i) = R.maniplty(q(i,:));
    end
end

function validarTrayectoriaArticular(R, q, nombre)
% Verifica dimensiones, NaN/Inf, límites articulares y saltos entre muestras
    if isempty(q) || size(q,2) ~= 6 || any(~isfinite(q), 'all')
        error('%s: trayectoria vacía, con dimensiones incorrectas o NaN/Inf.', nombre);
    end
    tol = 1e-8;
    fuera = q < (R.qlim(:,1)' - tol) | q > (R.qlim(:,2)' + tol);
    if any(fuera, 'all')
        [fila, art] = find(fuera, 1, 'first');
        error('%s: q%d fuera de límites en la muestra %d (%.4f rad).', nombre, art, fila, q(fila,art));
    end
    salto = max(abs(diff(q)), [], 'all');
    if salto > 0.5
        error('%s: salto articular de %.4f rad entre muestras (máx. 0.5 rad).', nombre, salto);
    end
end

function validarVelocidadesArticulares(qd, limites, nombre)
% Verifica que |qd| no supere los límites de velocidad articular (rad/s)
    vmax = max(abs(qd), [], 1);
    excede = find(vmax > limites);
    if ~isempty(excede)
        detalle = '';
        for j = excede
            detalle = [detalle, sprintf(' J%d: %.1f°/s > %.1f°/s;', j, rad2deg(vmax(j)), rad2deg(limites(j)))]; %#ok<AGROW>
        end
        error('%s: se exceden límites de velocidad.%s Aumente la duración del tramo.', nombre, detalle);
    end
end

function d = dist_segmento(p, a, b)
% Distancia de cada punto de p (Nx3) al segmento a-b (1x3)
    ab = b - a;
    s  = ((p - a) * ab') / (ab * ab');
    s  = min(max(s, 0), 1);
    d  = sqrt(sum((p - (a + s .* ab)).^2, 2));
end

function [desv_mm, v_tcp, ori_deg, epos_mm] = metricas_L(p, pd, Rot, T_ref, p_A, p_V, p_B)
% Métricas de calidad sobre la L:
%  desv_mm : distancia del TCP a la poligonal A-V-B (error geométrico, independiente del tiempo)
%  v_tcp   : rapidez lineal del TCP
%  ori_deg : ángulo entre la orientación real y la de referencia (misma muestra)
%  epos_mm : error de posición respecto de la referencia sincronizada en el tiempo
    N = size(p,1);
    desv_mm = min(dist_segmento(p, p_A, p_V), dist_segmento(p, p_V, p_B)) * 1000;
    v_tcp   = sqrt(sum(pd.^2, 2));
    p_ref   = squeeze(T_ref(1:3,4,:))';
    epos_mm = sqrt(sum((p - p_ref).^2, 2)) * 1000;
    ori_deg = zeros(N,1);
    for i = 1:N
        c = (trace(T_ref(1:3,1:3,i)' * Rot(:,:,i)) - 1) / 2;
        ori_deg(i) = acosd(max(min(c,1),-1));
    end
end

function marcar_tramos(t, idx_sold, idx_vert)
% Líneas magenta = vértice (llegada y fin del giro). Fondo celeste eliminado.
    xline(t(idx_vert(1)), 'm:', 'HandleVisibility', 'off');
    xline(t(idx_vert(2)), 'm:', 'HandleVisibility', 'off');
end

function graficar_articular(t, q, qd, qdd, idx_sold, idx_vert)
% Perfiles articulares de posición, velocidad y aceleración (ciclo completo)
    etiquetas_q = {'q_1','q_2','q_3','q_4','q_5','q_6'};
    colores = lines(6);
    figure('Name','Perfiles Articulares — Ciclo completo', 'Position',[50 50 1100 750]);

    subplot(3,1,1); hold on;
    for j = 1:6, plot(t, rad2deg(q(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5); end
    legend(etiquetas_q, 'Location','eastoutside');
    title('Posición articular');
    ylabel('Ángulo [°]'); grid on; xlim([t(1) t(end)]); marcar_tramos(t, idx_sold, idx_vert);

    subplot(3,1,2); hold on;
    for j = 1:6, plot(t, rad2deg(qd(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5); end
    legend(etiquetas_q, 'Location','eastoutside');
    title('Velocidad articular');
    ylabel('[°/s]'); grid on; xlim([t(1) t(end)]); marcar_tramos(t, idx_sold, idx_vert);

    subplot(3,1,3); hold on;
    for j = 1:6, plot(t, rad2deg(qdd(:,j)), 'Color', colores(j,:), 'LineWidth', 1.5); end
    legend(etiquetas_q, 'Location','eastoutside');
    title('Aceleración articular');
    xlabel('Tiempo [s]'); ylabel('[°/s²]'); grid on; xlim([t(1) t(end)]); marcar_tramos(t, idx_sold, idx_vert);

    sgtitle('Espacio articular');
end

function graficar_cartesiano(t, p, pd, pdd, idx_sold, idx_vert)
% Perfiles cartesianos del TCP: posición, velocidad (con rapidez) y aceleración
    colores = lines(3);
    figure('Name','Perfiles Cartesianos — Ciclo completo', 'Position',[350 50 1100 750]);

    subplot(3,1,1); hold on;
    for k = 1:3, plot(t, p(:,k), 'Color', colores(k,:), 'LineWidth', 1.5); end
    legend('X','Y','Z','Location','eastoutside');
    title('Posición del TCP'); ylabel('[m]'); grid on; xlim([t(1) t(end)]);
    marcar_tramos(t, idx_sold, idx_vert);

    subplot(3,1,2); hold on;
    for k = 1:3, plot(t, pd(:,k), 'Color', colores(k,:), 'LineWidth', 1.5); end
    plot(t, sqrt(sum(pd.^2, 2)), 'k', 'LineWidth', 1.8);
    legend('v_x','v_y','v_z','|v|','Location','eastoutside');
    title('Velocidad del TCP'); ylabel('[m/s]'); grid on; xlim([t(1) t(end)]);
    marcar_tramos(t, idx_sold, idx_vert);

    subplot(3,1,3); hold on;
    for k = 1:3, plot(t, pdd(:,k), 'Color', colores(k,:), 'LineWidth', 1.5); end
    legend('a_x','a_y','a_z','Location','eastoutside');
    title('Aceleración del TCP'); ylabel('[m/s²]'); xlabel('Tiempo [s]');
    grid on; xlim([t(1) t(end)]); marcar_tramos(t, idx_sold, idx_vert);

    sgtitle('Espacio cartesiano');
end