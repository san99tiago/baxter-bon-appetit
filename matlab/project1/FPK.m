% METADATA OF THE FILE
%{ 
    File: TPK.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-16-2020, 16:49
%}

% Definimos la funcion para calcular la TPK
function [Two_gt] = FPK(dof,ds,arm,ndof)
    %% Definimos cantidades importantes
    % Tabla de parametros DH brazo baxter 7dof
    DHTable0_7 = [0 0 0 dof(1);
                 -pi/2 ds(2) 0 (dof(2)+pi/2);
                 pi/2 0 ds(3) dof(3);
                 -pi/2 ds(4) 0 dof(4);
                 pi/2 0 ds(5) dof(5);
                 -pi/2 ds(6) 0 dof(6);
                 pi/2 0 0 dof(7)];
    % Tabla de parametros DH brazo baxter 6dof
    DHTable0_6 = [0 0 0 dof(1);
                 -pi/2 ds(2) 0 dof(2);
                 0 sqrt(ds(3)^2 + ds(4)^2) 0 (dof(4)+pi/2);
                 pi/2 0 ds(5) dof(5);
                 -pi/2 ds(6) 0 dof(6);
                 pi/2 0 0 dof(7)];
    % Matriz de transformacion wo->bl
    Two_bl = TransformationMatrix(RotationMatrix('z',-45,true),...
    [ds(8);-ds(9);ds(10)]);
    % Matriz de transformacion wo->br
    Two_br = TransformationMatrix(RotationMatrix('z',-135,true),...
    [-ds(8);-ds(9);ds(10)]);
    % Matriz de transformacion bl/br->0
    Tb_0 = TransformationMatrix(eye(3),[0;0;ds(1)]);
    % Matriz de transformacion 7->gt
    T7_gt = TransformationMatrix(eye(3),[0;0;ds(7)]);
    
    %% Calculamos la TPK dependiendo del brazo 
    switch(ndof)
        case 6
            switch(arm)
                case 'r'
                    Two_gt = Two_br*Tb_0*DirectCinematic(DHTable0_6,false)*T7_gt;
                case 'l'
                    Two_gt = Two_bl*Tb_0*DirectCinematic(DHTable0_6,false)*T7_gt;
            end
        case 7
            switch(arm)
                case 'r'
                    Two_gt = Two_br*Tb_0*DirectCinematic(DHTable0_7,false)*T7_gt;
                case 'l'
                    Two_gt = Two_bl*Tb_0*DirectCinematic(DHTable0_7,false)*T7_gt;
            end
    end
end
