% METADATA OF THE FILE
%{ 
    File: Baxter_Jacobian_7d0f.m
    Class: Industrial Robotics 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%} 

%Definimos la funcion encargada de calcular el jacobiano de Baxter
function [J] = Baxter_Jacobian_7dof()
    %% Definimos variables importantes 
    % Distancias del robot 
    syms L1 L2 L3 L4 L5
    % Grados de libertad del robot
    syms t1 t2 t3 t4 t5 t6 t7
    % Terminos de la transformada del robot 
    syms x0_7 y0_7 z0_7 r13 r23 r32
    
    %% Definimos las ecuaciones del robot 
    % Ecuaciones relacionadas a la translacion 
    f1 = L1*cos(t1) + L2*cos(t1)*cos(t2) - L3*(sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3)) - ...
         L4*((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*sin(t4) - cos(t1)*cos(t2)*cos(t4)) - ...
         L5*((sin(t1)*cos(t3) - cos(t1)*sin(t2)*sin(t3))*sin(t5) + ((sin(t1)*sin(t3) + ...
         cos(t1)*sin(t2)*cos(t3))*cos(t4) + cos(t1)*cos(t2)*sin(t4))*cos(t5)) - x0_7;
     
    f2 = L1*sin(t1) + L2*sin(t1)*cos(t2) + L3*(cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3)) + ...
         L4*((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*sin(t4) + sin(t1)*cos(t2)*cos(t4)) + ...
         L5*((cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))*sin(t5) + ((cos(t1)*sin(t3) - ...
         sin(t1)*sin(t2)*cos(t3))*cos(t4) - sin(t1)*cos(t2)*sin(t4))*cos(t5)) - y0_7;
     
    f3 = -L2*sin(t2) - L3*cos(t2)*cos(t3) - L4*(sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4)) + ...
         L5*((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + cos(t2)*sin(t3)*sin(t5)) - z0_7;
    
    % Ecuaciones relacionadas con la rotacion 
    f4 = -((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*sin(t4) - cos(t1)*cos(t2)*cos(t4))*cos(t6) - ...
         ((sin(t1)*cos(t3) - cos(t1)*sin(t2)*sin(t3))*sin(t5) + ((sin(t1)*sin(t3) + cos(t1)*sin(t2)*cos(t3))*cos(t4) + ...
         cos(t1)*cos(t2)*sin(t4))*cos(t5))*sin(t6) - r13;
     
    f5 = ((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*sin(t4) + sin(t1)*cos(t2)*cos(t4))*cos(t6) + ...
         ((cos(t1)*cos(t3) + sin(t1)*sin(t2)*sin(t3))*sin(t5) + ((cos(t1)*sin(t3) - sin(t1)*sin(t2)*cos(t3))*cos(t4) - ...
         sin(t1)*cos(t2)*sin(t4))*cos(t5))*sin(t6) - r23;
    
    f6 = -((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*sin(t5) - cos(t2)*sin(t3)*cos(t5))*cos(t7) - ...
         ((sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4))*sin(t6) + ((sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*cos(t5) + ...
         cos(t2)*sin(t3)*sin(t5))*cos(t6))*sin(t7) - r32;
     
    %% Calculamos el Jacobiano de Baxter
    J = jacobian([f1;f2;f3;f4;f5;f6],[t1 t2 t3 t4 t5 t6 t7]);
        
end 
