% METADATA OF THE FILE
%{ 
    File: BaxterFunctions.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 30-16-2020, 15:19
%}

% Definimos la funcion asociada a evaluar las funciones (ipk 7dof)
function [F] = BaxterFunctions(dof_0,FPK)
    %% Definimos las variables importantes
    % Distancias del robot 
    syms L1 L2 L3 L4 L5
    % Grados de libertad del robot
    syms t1 t2 t3 t4 t5 t6 t7
    
    %% Definimos las ecuaciones del robot 
    % Ecuaciones relacionadas a la translacion 
    f1 = L1*cos(t1) + L2*cos(t1)*cos(t2) - L3*(sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3)) - ...
         L4*((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*sin(t4) - cos(t1)*cos(t2)*cos(t4)) - ...
         L5*((sin(t1)*cos(t3) - cos(t1)*sin(t2)*sin(t3))*sin(t5) + ((sin(t1)*sin(t3) + ...
         cos(t1)*sin(t2)*cos(t3))*cos(t4) + cos(t1)*cos(t2)*sin(t4))*cos(t5)) - FPK(1,4);
     
    f2 = L1*sin(t1) + L2*sin(t1)*cos(t2) + L3*(cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3)) + ...
         L4*((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*sin(t4) + sin(t1)*cos(t2)*cos(t4)) + ...
         L5*((cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))*sin(t5) + ((cos(t1)*sin(t3) - ...
         sin(t1)*sin(t2)*cos(t3))*cos(t4) - sin(t1)*cos(t2)*sin(t4))*cos(t5)) - FPK(2,4);
     
    f3 = -L2*sin(t2) - L3*cos(t2)*cos(t3) - L4*(sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4)) + ...
         L5*((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + cos(t2)*sin(t3)*sin(t5)) - FPK(3,4);
    
    % Ecuaciones relacionadas con la rotacion 
    f4 = -((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*sin(t4) - cos(t1)*cos(t2)*cos(t4))*cos(t6) - ...
         ((sin(t1)*cos(t3) - cos(t1)*sin(t2)*sin(t3))*sin(t5) + ((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*cos(t4) + ...
         cos(t1)*cos(t2)*sin(t4))*cos(t5))*sin(t6) - FPK(1,3);
     
    f5 = ((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*sin(t4) + sin(t1)*cos(t2)*cos(t4))*cos(t6) + ...
         ((cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))*sin(t5) + ((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*cos(t4) - ...
         sin(t1)*cos(t2)*sin(t4))*cos(t5))*sin(t6) - FPK(2,3);
    
    f6 = -((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*sin(t5) - cos(t2)*sin(t3)*cos(t5))*cos(t7) - ...
         ((sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4))*sin(t6) + ((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + ...
         cos(t2)*sin(t3)*sin(t5))*cos(t6))*sin(t7) - FPK(3,2);
     
    %% Definimos los valores de las variables
    L1 = 0.06900;
    L2 = 0.36435;
    L3 = 0.06900;
    L4 = 0.37429;
    L5 = 0.01000;
    t1 = dof_0(1);
    t2 = dof_0(2);
    t3 = dof_0(3);
    t4 = dof_0(4);
    t5 = dof_0(5);
    t6 = dof_0(6);
    t7 = dof_0(7);
    
    % Definimos el Jacobiano de baxter evaluado con los valores dados
    F = [f1;f2;f3;f4;f5;f6];
    F = double(subs(F));    
end