% METADATA OF THE FILE
%{ 
    File: Puntos_Simulacion.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-17-2020, 18:06
%}

% Definimos la funcion asociada a encontrar los puntos 
function [P_list] = Puntos_Simulacion(dof,ds,arm)
    %% Definimos cantidades importantes
    % Definimos angulo importante
    alpha = atan(ds(4)/ds(3));
    lh = sqrt(ds(3)^2+ds(4)^2);
    % Matriz de transformacion wo->bl
    Two_bl = TransformationMatrix(RotationMatrix('z',-45,true),...
    [ds(8);-ds(9);ds(10)]);
    % Matriz de transformacion wo->br
    Two_br = TransformationMatrix(RotationMatrix('z',-135,true),...
    [-ds(8);-ds(9);ds(10)]);
    % Tabla de parametros DH brazo baxter 6dof
    DHTable0_7 = [0 0 0 dof(1);
                 -pi/2 ds(2) 0 dof(2);
                 0 0 0 0;
                 0 lh 0 (dof(3)+pi/2);
                 pi/2 0 ds(5) dof(4);
                 -pi/2 ds(6) 0 dof(5);
                 pi/2 0 0 dof(6)];
   
    % Inicializamos la matriz de puntos
    P_list = [];
    % Definimos la respuesta dependiendo del brazo
    switch(arm)
        case 'r'
            P_list(:,1) = Two_br(1:3,4);
            M_list = Two_br*TransformationMatrix(eye(3),[0;0;ds(1)]);
        case 'l'
            P_list(:,1) = Two_bl(1:3,4);
            M_list = Two_bl*TransformationMatrix(eye(3),[0;0;ds(1)]);
    end
    %M_list = TransformationMatrix(eye(3),[0;0;ds(1)]);
    
    % Realizamos los calculos necesarios
    for j = 1:7
        % Calculamos la rotacion de alpha de i-1
        TMalpha = TransformationMatrix(RotationMatrix('x',DHTable0_7(j,1),false),[0;0;0]);
        % Calculamos la traslacion de a de i-1
        TMa = TransformationMatrix(eye(3),[DHTable0_7(j,2);0;0]);
        % Calculamos la traslacion de b de i
        TMb = TransformationMatrix(eye(3),[0;0;DHTable0_7(j,3)]);
        % Calculamos la rotacion de theta de i
        TMtheta = TransformationMatrix(RotationMatrix('z',DHTable0_7(j,4),false),[0;0;0]);
        
        % Calculamos la matriz de transformacion total
        TMtotal = TMalpha*TMa*TMb*TMtheta;
        
        % Almacenamos el punto 
        M_list = M_list*TMtotal;
        P_list(:,j+1) = M_list(1:3,4);
        if j == 3
            P_curve = M_list*[ds(3)*cos(alpha);-ds(3)*sin(alpha);0;1];
            P_list(:,j+1) = P_curve(1:3,1);
        end
       
    end
    M_list = M_list*TransformationMatrix(eye(3),[0;0;ds(7)]);
    P_list(:,9) = M_list(1:3,4);
    
    % Cuadramos los numeros que son muy pequeños 
    for j = 1:9
        for k = 1:3
            if abs(P_list(k,j)) < 10^-8
                P_list(k,j) = 0;
            end
        end
    end
end
