% METADATA OF THE FILE
%{ 
    File: IPK.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-16-2020, 20:02
%}

% Definimos la funcion para calcular la IPK
function [vdof] = IPK(FPK,ds,arm,type)
    %% Definimos cantidades importantes
    % Matriz de transformacion wo->0
    Two_0_r = TransformationMatrix(RotationMatrix('z',-135,true),...
    [-ds(7);-ds(8);ds(9)])*TransformationMatrix(eye(3),[0;0;ds(1)]);
    Two_0_l = TransformationMatrix(RotationMatrix('z',-45,true),...
    [ds(7);-ds(8);ds(9)])*TransformationMatrix(eye(3),[0;0;ds(1)]);

    % Matriz de transformacion 6->gt
    T6_gt = TransformationMatrix(eye(3),[0;0;ds(6)]);
    
    % Matriz de transformacion 0->6
    switch(arm)
        case 'r'
            T0_6 = (Two_0_r^-1)*FPK*(T6_gt^-1);
        case 'l'
            T0_6 = (Two_0_l^-1)*FPK*(T6_gt^-1);
    end
    
    %% Comenzamos calculando los dof referentes a la traslacion 
    % Calculamos theta 1
    t1 = atan2(T0_6(2,4),T0_6(1,4));
    
    % Calculamos theta 2
    % Definimos cantidades importantes
    lh = sqrt(ds(3)^2+ds(4)^2);
    E = 2*lh*(ds(2) - T0_6(1,4)/cos(t1));
    F = 2*lh*T0_6(3,4);
    G = (T0_6(1,4)/cos(t1))^2 + ds(2)^2 + lh^2 - ds(5)^2 + T0_6(3,4)^2 - 2*ds(2)*(T0_6(1,4)/cos(t1));
    syms t
    tt2 = double(solve((G-E)*t^2 + 2*F*t + (G+E) == 0,t));
    t2 = 2*atan(tt2);
    for i = 1:2
        if abs(imag(t2(i))) < 2
            t2(i) = real(t2(i));
        end
    end
    
    % Calculamos theta 3
    t4 = atan2(-T0_6(3,4) - lh*sin(t2),T0_6(1,4)/cos(t1) - ds(2) - lh*cos(t2)) - t2;
    
    %% Calculamos los dof referentes a la rotacion 
    % Definimos cantidades importantes
    s1 = sin(t1);
    c1 = cos(t1);
    s24 = sin(t2+t4);
    c24 = cos(t2+t4);
    R0_3 = struct('eu',[-c1*s24(1) -c1*c24(1) -s1;
                       -s1*s24(1) -s1*c24(1) c1;
                       -c24(1) s24(1) 0],...
                  'ed',[-c1*s24(2) -c1*c24(2) -s1;
                       -s1*s24(2) -s1*c24(2) c1;
                       -c24(2) s24(2) 0]);
    R3_6 = struct('eu',transpose(R0_3.eu)*T0_6(1:3,1:3),...
                  'ed',transpose(R0_3.ed)*T0_6(1:3,1:3));
    
   switch(type)
       case 'u'
           % Calculamos theta 5
           t5 = atan2(R3_6.eu(3,3),R3_6.eu(1,3));
           
           % Calculamos theta 7
           t7 = atan2(-R3_6.eu(2,2),R3_6.eu(2,1));
           
           % Calculamos theta 6
           t6 = atan2(R3_6.eu(2,1)/cos(t7),-R3_6.eu(2,3));
       case 'd'
           % Calculamos theta 5
           t5 = atan2(R3_6.ed(3,3),R3_6.ed(1,3));
           
           % Calculamos theta 7
           t7 = atan2(-R3_6.ed(2,2),R3_6.ed(2,1));
           
           % Calculamos theta 6
           t6 = atan2(R3_6.ed(2,1)/cos(t7),-R3_6.ed(2,3));
   end
   
   %% Definimos el vector de los dof
   switch(type)
       case 'u'
           vdof = [t1;t2(1);0;t4(1);t5;t6;t7];
       case 'd'
           vdof = [t1;t2(2);0;t4(2);t5;t6;t7];
   end
    
end