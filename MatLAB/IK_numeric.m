function [exito, q_resultado, mensaje] = IK_numeric(R,T,q_mejor,q0)

% Inicialización de variables de salida al comienzo (buena practica)
exito = false;
q_resultado = [];
mensaje = 'Error desconocido';

% Mascara - consideramos las 6 articulaciones activas
M = [1,1,1,1,1,1];
% cantidad de semillas aleatorias que se prueban para q_mejor = false
seeds = 100;
% Tolerancia para decidir si dos soluciones son "la misma" [rad].
tol = 1e-2;
% salto máximo admitido entre dos muestras consecutivas [rad]
umbral_salto = 0.5;
% Error máximo admitido entre la pose pedida y la pose que
% realmente alcanza el robot con la q calculada (norma de la diferencia de
% las matrices 4x4). Si es mayor, ikine NO convergió.
tol_pose = 1e-3;

% Apagamos los warnings de ikine (los guardamos en estado_warn
% para poder restaurarlos al final). No perdemos información porque ahora
% detectamos la no convergencia nosotros, comparando con la pose pedida.
estado_warn = warning('off', 'all');

% Versión numérica (4x4xN) de las poses pedidas, para poder
% compararlas con lo que calcula fkine
T_obj = double(T);

% Primera pose sola (para resolverla por separado).
% Si T es un arreglo numérico 4x4xN tomamos la primera matriz; si es un
% objeto SE3 tomamos su primer elemento.
if isnumeric(T)
    T_primera = T(:,:,1);
else
    T_primera = T(1);
end

%% --- q_mejor = true -> unica solucion ---
if q_mejor == true
    try
        q_temp = R.ikine(T, 'q0', q0, 'mask', M);

        % Verificación con cinemática directa 
        Tc = double(R.fkine(q_temp));      % poses alcanzadas con q_temp (4x4xN)
        error_pose = 0;
        for k = 1:size(T_obj,3)
            error_pose = max(error_pose, norm(Tc(:,:,k) - T_obj(:,:,k)));
        end

        if error_pose > tol_pose
            mensaje = ['ikine no convergió: la pose alcanzada difiere de la pedida en ', num2str(error_pose)];
        else

            % corrección de saltos de 2*pi ---------------------------------
            for k = 2:size(q_temp,1)
                n = round((q_temp(k,:) - q_temp(k-1,:)) / (2*pi));
                q_temp(k,:) = q_temp(k,:) - 2*pi*n;
            end
            % ---------------------------------------------------------------

            % verificamos límites articulares
            fuera_de_limites = false;
            for i = 1:6
                if min(q_temp(:,i)) < R.qlim(i,1) || max(q_temp(:,i)) > R.qlim(i,2)
                    fuera_de_limites = true;
                    mensaje = ['La articulación q', num2str(i), ' queda fuera de sus límites'];
                    break;
                end
            end

            % verificamos continuidad (salto máximo entre muestras)
            salto_max = 0;
            if size(q_temp,1) > 1
                salto_max = max(max(abs(diff(q_temp))));
            end

            if fuera_de_limites == false
                if salto_max > umbral_salto
                    mensaje = ['Hay un salto de configuración de ', num2str(salto_max), ' rad entre muestras'];
                else
                    % si pudo calcular la cinematica inversa, actualizamos las salidas
                    exito = true;
                    q_resultado = q_temp;
                    mensaje = 'Solución encontrada con éxito';
                end
            end
        end

    catch ME
        mensaje = ['Solución no encontrada: ', ME.message];
    end
%% --- q_mejor = false -> multiples soluciones ---
else
    tic
    soluciones_encontradas = {};   
    q_semilla_nuevo = zeros(1,6);

    % Lista de configuraciones de la PRIMERA pose que ya
    % probamos (hayan dado una solución válida o no). Cada fila es un q.
    primeras_probadas = zeros(0,6);

    for j=1:seeds
        % genero semilla aleatoria a partir de los límites articulares
        for i = 1:6
        q_semilla_nuevo(i) = R.qlim(i,1) + (R.qlim(i,2)-R.qlim(i,1))*rand(1,1); % formato: a + (b-a)*rand()
        end
        try
            % PASO 1: resolvemos SOLO la primer pose 
            q_primera = R.ikine(T_primera, 'q0', q_semilla_nuevo, 'mask', M);
            if isempty(q_primera)
                continue
            end

            % si no convergió, descartamos la semilla
            Tc1 = double(R.fkine(q_primera));
            if norm(Tc1 - T_obj(:,:,1)) > tol_pose
                continue;
            end

            % ¿esta configuración de la primera pose ya la probamos?
            % (se compara módulo 2*pi, igual que en el bloque de repetidas)
            ya_probada = false;
            for k = 1:size(primeras_probadas,1)
                d = q_primera - primeras_probadas(k,:);
                d = atan2(sin(d), cos(d));
                if max(abs(d)) < tol
                    ya_probada = true;
                    break;
                end
            end
            if ya_probada
                continue;
            end
            primeras_probadas(end+1,:) = q_primera;

            % PASO 2: trayectoria completa, usando como semilla
            % la solución de la primera pose
            q_temp = R.ikine(T,'q0', q_primera, 'mask', M);
            if isempty(q_temp)
                continue
            end

            % si ikine no convergió en alguna pose, descartamos la solución
            Tc = double(R.fkine(q_temp));
            error_pose = 0;
            for k = 1:size(T_obj,3)
                error_pose = max(error_pose, norm(Tc(:,:,k) - T_obj(:,:,k)));
            end
            if error_pose > tol_pose
                continue;
            end

            % corrección de saltos de 2*pi ---------------------------------
            for k = 2:size(q_temp,1)
                n = round((q_temp(k,:) - q_temp(k-1,:)) / (2*pi));
                q_temp(k,:) = q_temp(k,:) - 2*pi*n;
            end
            % ---------------------------------------------------------------

            es_valida = true;
            for i = 1:6
                % Comparamos el mínimo y máximo de toda la columna (articulación i) contra sus límites
                if min(q_temp(:, i)) < R.qlim(i, 1) || max(q_temp(:, i)) > R.qlim(i, 2)
                    es_valida = false;
                    break;
                end
            end
            if es_valida == false
                continue;
            end

            % descartamos la solución si todavía tiene saltos grandes
            if size(q_temp,1) > 1
                if max(max(abs(diff(q_temp)))) > umbral_salto
                    continue;
                end
            end

            % bloque de verificacion de soluciones repetidas
            % (con el PASO 1 casi nunca se activa, pero lo dejamos como
            % seguridad: compara la trayectoria completa, módulo 2*pi)
            es_repetida = false;
            for k = 1:length(soluciones_encontradas)
                sol_guardada = soluciones_encontradas{k};
                diferencia = q_temp - sol_guardada;                    % matriz Nx6
                diferencia = atan2(sin(diferencia), cos(diferencia));  % pasa cada ángulo a (-pi, pi]
                distancia = max(max(abs(diferencia)));                 % mayor diferencia de todas
                if distancia < tol
                    es_repetida = true;
                    break;
                end
            end
            if es_repetida == false
                soluciones_encontradas{end+1} = q_temp;
                exito = true;
                mensaje = 'Se encontraron múltiples soluciones';
            end

        catch ME
            mensaje = ['Solución no encontrada: ', ME.message];
        end
    end
    q_resultado = soluciones_encontradas;

    % si ninguna semilla dio una solución válida, lo decimos
    if exito == false
        mensaje = 'Ninguna semilla produjo una solución que converja, sea continua y esté dentro de los límites articulares';
    end
    toc
end

% restauramos el estado original de los warnings
warning(estado_warn);

end