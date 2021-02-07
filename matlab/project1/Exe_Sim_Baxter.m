% METADATA OF THE FILE
%{ 
    File: Exe_Sim_Baxter.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-19-2020, 10:51
%}

% Definimos la funcion encargada de los parametros para la simulacion 
function [] = Exe_Sim_Baxter()
    % Creamos un objeto BAXTER
    br = BaxterClass;
    % Definimos los valores asociados al tiempo de la simulacion 
    tp = [5;0.1];
    % Cantidades asociadas a la traslacion 
    linear = struct('r',struct('p0',[-1.1141 -0.9001 1.3644],... % P home
                               'pf',[-0.8 0 2],... % P goal
                               'v0',[0 0.2 0],... % V home
                               'vf',[0 0 0],... % V goal
                               'a0',[0 0 0],... % A home
                               'af',[0 0 0.1]),... % A goal
                    'l',struct('p0',[1.1141 -0.9001 1.3644],... % P home
                               'pf',[1 -0.5 2],... % P goal
                               'v0',[0 0 0],... % V home
                               'vf',[0 0 0.1],... % V goal
                               'a0',[0 0 0],... % A home
                               'af',[0.3 0 0])); % A goal
    % Cantidades asociadas a la orientacion
    or = struct('r',struct('p0',[0.7854 1.5708 -1.5708],... % P home
                           'pf',[0.7854 1.5708 -1.5708],... % P goal
                           'v0',[0 0 0],... % V home
                           'vf',[0 0 0],... % V goal
                           'a0',[0 0 0],... % A home
                           'af',[0 0 0]),... % A goal
                'l',struct('p0',[0.7854 1.5708 0],... % P home
                           'pf',[0.7854 1.5708 0],... % P goal
                           'v0',[0 0 0],... % V home
                           'vf',[0 0 0],... % V goal
                           'a0',[0 0 0],... % A home
                           'af',[0 0 0])); % A goal
                       
    % Ejecutamos la simulacion
    Baxter_Simulation(br,tp,linear,or);
end