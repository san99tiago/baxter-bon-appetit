% METADATA OF THE FILE
%{ 
    File: Trayectoria_5_orden.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-15-2020, 20:59
%}

function [p,v,a,j] = Trayectoria_5_orden(t,tf,tp0,tpf,tv0,tvf,ta0,taf)
    % Definimos los parametros de la trayectoria
    a0 = tp0;
    a1 = tv0;
    a2 = ta0/2;
    a3 = (20*tpf - 20*tp0 - (8*tvf + 12*tv0)*tf - (3*ta0 - taf)*tf^2)/(2*tf^3);
    a4 = (30*tp0 - 30*tpf + (14*tvf + 16*tv0)*tf + (3*ta0 - 2*taf)*tf^2)/(2*tf^4);
    a5 = (12*tpf - 12*tp0 - (6*tvf + 6*tv0)*tf - (ta0 - taf)*tf^2)/(2*tf^5);
    
    % Definimos las cantidades cinematicas
    p = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5;
    v = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4;
    a = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3;
    j = 6*a3 + 24*a4*t + 60*a5*t^2;

end