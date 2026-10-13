
function [exito, resultado, mensaje_error] = resolver_CI_trayectoria(robot, poses, q_mejor, q0)

    exito = false;
    resultado = [];
    mensaje_error = '';

 
    n_gdl = robot.n;
    
    if length(q0) ~= n_gdl
        mensaje_error = sprintf('El vector q0 debe tener %d elementos (el robot tiene %d GDL).', n_gdl, n_gdl);
        return;
    end
    q0 = q0(:)'; 

    [T_mat, N, err_p] = normalizar_poses(poses);
    if ~isempty(err_p)
        mensaje_error = err_p;
        return;
    end

    if n_gdl == 6
        mask = ones(1, 6);
    else
        mask = ones(1, 6);
        mask(3) = 0; mask(4) = 0; mask(5) = 0; 
    end

    if q_mejor
        [ok_traj, q_traj, err_msg] = resolver_rama_continua(robot, T_mat, q0, mask);
        if ok_traj
            exito = true;
            resultado = q_traj;
            mensaje_error = '';
        else
            exito = false;
            resultado = [];
            mensaje_error = sprintf('Fallo al resolver la trayectoria desde q0: %s', err_msg);
        end
        return;
    end

    semillas = generar_semillas_articulares(robot, q0);
    
    T1 = T_mat(:, :, 1);
    soluciones_q1 = {};
    
    for i = 1:size(semillas, 1)
        q_seed = semillas(i, :);
        [q_sol, ok_sol] = resolver_pose_ikine(robot, T1, q_seed, mask);
        
        if ok_sol && dentro_de_limites(robot, q_sol)

            es_duplicada = false;
            for k = 1:length(soluciones_q1)
                if norm(wrapToPi(q_sol - soluciones_q1{k})) < 0.01
                    es_duplicada = true;
                    break;
                end
            end
            if ~es_duplicada
                soluciones_q1{end+1} = q_sol;
            end
        end
    end

    if isempty(soluciones_q1)
        exito = false;
        mensaje_error = 'No se encontro ninguna solucion articular valida para la pose inicial.';
        return;
    end

    trayectorias_validas = {};
    for m = 1:length(soluciones_q1)
        q1_m = soluciones_q1{m};
        [ok_m, q_traj_m, ~] = resolver_rama_continua(robot, T_mat, q1_m, mask);
        
        if ok_m

            es_duplicada_traj = false;
            for t = 1:length(trayectorias_validas)
                diff_max = max(max(abs(wrapToPi(q_traj_m - trayectorias_validas{t}))));
                if diff_max < 0.01
                    es_duplicada_traj = true;
                    break;
                end
            end
            if ~es_duplicada_traj
                trayectorias_validas{end+1} = q_traj_m;
            end
        end
    end

    if isempty(trayectorias_validas)
        exito = false;
        mensaje_error = 'Se encontraron posiciones iniciales pero ninguna pudo completarse dentro de qlim.';
        return;
    end

    exito = true;
    resultado = trayectorias_validas;
    mensaje_error = '';

end

% FUNCIONES AUXILIARES INTERNAS

function [ok, q_traj, err_msg] = resolver_rama_continua(robot, T_mat, q_inicial, mask)
    N = size(T_mat, 3);
    n_gdl = robot.n;
    q_traj = zeros(N, n_gdl);
    q_semilla = q_inicial;
    ok = true;
    err_msg = '';

    for k = 1:N
        Tk = T_mat(:, :, k);
        [qk, ok_k] = resolver_pose_ikine(robot, Tk, q_semilla, mask);
        
        if ~ok_k
            ok = false;
            err_msg = sprintf('El solver no convergio en la pose N°%d.', k);
            q_traj = [];
            return;
        end
        
        if ~dentro_de_limites(robot, qk)
            ok = false;
            err_msg = sprintf('La solucion en la pose N°%d excede los limites articulares (qlim).', k);
            q_traj = [];
            return;
        end
        
        q_traj(k, :) = qk;
        q_semilla = qk; 
    end
end

function [q_sol, ok] = resolver_pose_ikine(robot, T, q_seed, mask)
    ok = false;
    q_sol = zeros(1, robot.n);
    tol_pos = 0.001;

    try
        q_sol = robot.ikine(T, 'q0', q_seed, 'mask', mask, 'verbose', false);
    catch
        try
            q_sol = robot.ikine(T, q_seed, mask);
        catch
            return;
        end
    end

    
    if isempty(q_sol) || any(isnan(q_sol)) || length(q_sol) ~= robot.n
        q_sol = zeros(1, robot.n);
        return;
    end

    q_sol = wrapToPi(q_sol(:)');

    T_calc = robot.fkine(q_sol);
    if isa(T_calc, 'SE3'), T_calc = T_calc.T; end
    
    err_pos = norm(T(1:3,4) - T_calc(1:3,4));
    if err_pos < tol_pos
        ok = true;
    end
end

function dentro = dentro_de_limites(robot, q)
    q = wrapToPi(q(:)');
    tol = 0.0001;
    dentro = true;
    if isprop(robot, 'qlim') && ~isempty(robot.qlim)
        qmin = robot.qlim(:,1)';
        qmax = robot.qlim(:,2)';
        dentro = all(q >= (qmin - tol) & q <= (qmax + tol));
    end
end

function semillas = generar_semillas_articulares(robot, q0)
    n = robot.n;
    semillas = q0(:)';
    grid_vals = [-pi/2, 0, pi/2, pi-0.1, -pi+0.1];
    
    if n == 3
        [g1, g2, g3] = ndgrid(grid_vals, grid_vals, grid_vals);
        semillas = [semillas; [g1(:), g2(:), g3(:)]];
    else
        base_seeds = [
            q0;
            -q0;
            q0 + pi/2;
            q0 - pi/2;
            zeros(1, n);
            ones(1, n)*pi/4;
            ones(1, n)*(-pi/4)
        ];
        semillas = [semillas; base_seeds];
    end
    semillas = wrapToPi(semillas);
end

function [T_mat, N, err_msg] = normalizar_poses(poses)
    err_msg = '';
    if isa(poses, 'SE3')
        N = length(poses);
        T_mat = zeros(4,4,N);
        for k = 1:N
            T_mat(:,:,k) = poses(k).T;
        end
    elseif iscell(poses)
        N = length(poses);
        T_mat = zeros(4,4,N);
        for k = 1:N
            T_mat(:,:,k) = poses{k};
        end
    elseif isnumeric(poses) && ndims(poses) == 3 && size(poses,1) == 4 && size(poses,2) == 4
        T_mat = poses;
        N = size(poses, 3);
    elseif isnumeric(poses) && size(poses,1) == 4 && size(poses,2) == 4
        T_mat = poses;
        N = 1;
    else
        T_mat = [];
        N = 0;
        err_msg = 'El formato de las poses no es valido.';
    end
end
