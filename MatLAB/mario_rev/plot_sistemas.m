% =========================================================================
% Ejercicio TF 2: Análisis de Sistemas de Referencia DH
% =========================================================================
clear; clc; close all;

% a. Llamado del archivo "robot.m"
robot; 

figure('Name', 'Analisis de Sistemas de Referencia DH');

% b. Vector de posiciones articulares a analizar
% Evaluamos la posición cero (ahora corregida por el offset, debe verse en "L")
q_test = [0, 0, 0, 0, 0, 0]; 

% c. Vector de booleanos para indicar qué sistemas visualizar[cite: 1]
% 1 = graficar, 0 = ocultar (Sistemas: {0}, {1}, {2}, {3}, {4}, {5}, {6})
sistemas = [1, 1, 1, 1, 1, 1, 1]; 

% d. Ploteo del robot[cite: 1]
% Usamos 'notiles' para quitar el suelo y ver los ejes limpios
R.plot(q_test, 'workspace', workspace, 'scale', 0.5, 'jointdiam', 0.5, 'notiles');

% e. Bucle para recorrer y graficar los sistemas
hold on; 
T = R.base; 

for i = 1:(R.n + 1)
    
    if sistemas(i) == 1
        num_sistema = i - 1; 
        % Graficamos el sistema actual usando trplot
        trplot(T, 'frame', num2str(num_sistema), 'length', 0.25, 'color', 'b', 'thick', 1.5);
    end
    
    % Calculamos la transformación hacia el siguiente eslabón
    if i <= R.n
        T = T * R.links(i).A(q_test(i));
    end
end

% Si tuvieras herramienta definida en R.tool, la graficamos al final:
% T_tool = T * R.tool;
% trplot(T_tool, 'frame', 'Tool', 'length', 0.3, 'color', 'r', 'thick', 2);

hold off;