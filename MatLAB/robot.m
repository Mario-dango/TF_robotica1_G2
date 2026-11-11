% b. declaramos dh, en cada fila tenemos cada uno de los parametros de cada
% articulacion

dh = [
    0,   0.450,   0.075,   pi/2;
    0,       0,   0.640,      0;
    0,   0.119,   0.195,   pi/2; % el parametro d no sale bien en el datasheet, considero la mitad de la medida 238mm   
    0,   0.700,       0,   pi/2; % 600 porque en el datasheet no se especifica una medida y use 100mm (700-100)mm =s 600mm
    0,       0,       0,  -pi/2;
    0,   0.075,       0,      0
    ];


% preasignacion de memoria
L = Link.empty(0, 6);

for i = 1:6
    L(i) = Link(dh(i, 1:4));
end

% c. creamos objeto SerialLink

R = SerialLink(L);

R.name = 'FANUC M-10iD/12';

% limites articulares del robot
R.qlim = [deg2rad(-170), deg2rad(170);
          deg2rad(-145), deg2rad(90);
          deg2rad(-170), deg2rad(285);
          deg2rad(-190), deg2rad(190);
          deg2rad(-140), deg2rad(140);
          deg2rad(-270), deg2rad(270);
];

% ponemos un offset de 90° grados en q2 para tener una posición como la del

% datasheet del robot FANUC M10iD/12L

R.offset = [0 pi/2 0 0 0 0];

% Matriz para base y herramienta
R.base = [1, 0, 0,   0;
          0, 1, 0,   0;
          0, 0, 1, 0.6;
          0, 0,  0,  1];

R.tool = [ 0.9063,  0,  0.4226, -0.0634;
                0,  1,       0,       0; 
          -0.4226,  0,  0.9063,  0.2963;
                0,  0,       0,      1];

% d. definimos el workspace con el formato [-limX, +limX, -limY, +limY, -limZ, +limZ]
workspace = [-3 3 -3 3 -1 4];
