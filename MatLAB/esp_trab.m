function esp_trab(R,workspace)

%% --- Segmentos para determinar el espacio de trabajo del robot - vista XZ --- 
traj_0 = jtraj(deg2rad([0, 56, -58, 0, 0, 0])  , deg2rad([0, -45, -58, 0, 0, 0]) , 100); % bocado - parte abajo derecha
traj_1 = jtraj(deg2rad([0, -45, -58, 0, 0, 0]) , deg2rad([0, -65, -45, 0, 0, 0]) , 100); % bocado
traj_2 = jtraj(deg2rad([0, -65, -45, 0, 0, 0]) , deg2rad([0, -145, -34, 0, 0, 0]), 100); % bocado - parte inferior 
traj_3 = jtraj(deg2rad([0, -145, -34, 0, 0, 0]), deg2rad([0, -145, 90, 0, 0, 0]) , 100); % barrido articulación 3 - parte inferior externa del workspace
traj_4 = jtraj(deg2rad([0, -145, 90, 0, 0, 0]) , deg2rad([0, 65, 90, 0, 0, 0])   , 100); % barrido articulación 2 - parte derecha externa del workspace
traj_5 = jtraj(deg2rad([0, 65, 90, 0, 0, 0])   , deg2rad([0, 65, 182, 0, 0, 0])  , 100); % barrido articulación 3 - parte izquierda externa del workspace
traj_6 = jtraj(deg2rad([0, 65, 182, 0, 0, 0])  , deg2rad([0, 15, 235, 0, 0, 0])  , 100); % 
traj_7 = jtraj(deg2rad([0, 15, 235, 0, 0, 0])  , deg2rad([0, -120, 215, 0, 0, 0]), 100); % bocado - parte superior
traj_8 = jtraj(deg2rad([0, -120, 215, 0, 0, 0]), deg2rad([0, -165, 227, 0, 0, 0]), 100); % bocado - parte derecha

% Concatenación de segmentos
EDTXZ = vertcat(traj_0, traj_1, traj_2, traj_3, traj_4, traj_5, traj_6, traj_7, traj_8);

% como tenemos 100 puntos por cada segmento, vamos a tener una matriz de
% 900x3
Puntos_Lat = zeros(length(EDTXZ), 3);

% calculamos cinemática directa para cada uno de los puntos del espacio de
% trabajo para obtener las coordenadas de traslación del espacio cartesiano

for i = 1:length(Puntos_Lat)
    Puntos_Lat(i, :) = transl(R.fkine(EDTXZ(i,:)));
end

figure('Name', 'Espacio de trabajo plano XZ');
plot(Puntos_Lat(:,1), Puntos_Lat(:,3), 'r', 'LineWidth', 2);
grid on; xlabel('Eje X (m)'); ylabel('Eje Z (m)'); axis equal;

%% --- Vista plano XY ---

traj_planta = jtraj(deg2rad([-170, -90, 90, 0, 0, 0]), deg2rad([170, -90, 90, 0, 0, 0]), 100);

Puntos_Sup = zeros(length(traj_planta), 3);

for i = 1:length(Puntos_Sup)
    Puntos_Sup(i,:) = transl(R.fkine(traj_planta(i,:)));
end

figure('Name', 'Espacio de trabajo plano XY');
plot(Puntos_Sup(:,1), Puntos_Sup(:,2), 'g', 'LineWidth', 2);
grid on; xlabel('Eje X (m)'); ylabel('Eje Y (m)'); axis equal;

%% --- Gráfico 3D combinado ---
figure('Name', 'Espacio de trabajo 3D combinado');
hold on; grid on;

% Superposición de trayectorias
plot3(Puntos_Lat(:,1), Puntos_Lat(:,2), Puntos_Lat(:,3), 'r', 'LineWidth', 2);
plot3(Puntos_Sup(:,1), Puntos_Sup(:,2), Puntos_Sup(:,3), 'g', 'LineWidth', 2);

% Ploteo del modelo cinemático
R.teach([0 0 0 0 0 0], 'workspace', workspace, 'scale', 0.5, 'notiles');
view(3); hold off;

end

