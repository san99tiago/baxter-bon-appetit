% METADATA OF THE FILE
%{ 
    File: IPK_7dof.m
    Class: Industrial Robotics
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%}

% Definimos la funcion encargada de calcular la IPK de Baxter on 7 dof
function [dof] = IPK_7dof(FPK,ds,arm,dof_0,iter,delta,tol)
    %% Definimos caracteristicas importantes del programa
    format long 
    i = 1;
    del = 1000;
    error = 1000;
    %% Definimos cantidades importantes
    % Matriz de transformacion wo->0
    Two_0_r = TransformationMatrix(RotationMatrix('z',-135,true),...
    [-ds(7);-ds(8);ds(9)])*TransformationMatrix(eye(3),[0;0;ds(1)]);
    Two_0_l = TransformationMatrix(RotationMatrix('z',-45,true),...
    [ds(7);-ds(8);ds(9)])*TransformationMatrix(eye(3),[0;0;ds(1)]);

    % Matriz de transformacion 7->gt
    T7_gt = TransformationMatrix(eye(3),[0;0;ds(6)]);
    
    % Matriz de transformacion 0->6
    switch(arm)
        case 'r'
            T0_7 = (Two_0_r^-1)*FPK*(T7_gt^-1);
        case 'l'
            T0_7 = (Two_0_l^-1)*FPK*(T7_gt^-1);
    end
    
    
    %% Iniciamos el ciclo encargado del proceso iterativo de la solucion
    dof = dof_0;
    del_dof = [1;1;1;1;1;1;1];
    while i<=iter & del>delta & norm(del_dof)>tol
        J = BaxterJacobian(dof)
        J_ast = transpose(J)*(J*transpose(J))^-1
        F = BaxterFunctions(dof,T0_7)
        del_dof = -J_ast*F
        dof = dof + del_dof
        del = norm(F)
        i = i + 1
    end
end