% METADATA OF THE FILE
%{ 
    File: DirectCinematic.m
    Class: Industrial Robotics 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%}

function [DC_TMatrix_Out] = DirectCinematic(TableOfParams, type)
    %{
       En esta funcion se calcula la cinematica directa de cualquier robot,
       tiene como entrada la tabla de parametros de DH del robot.
       Es posible escoger trabajar con grados y con radianes 
       Si type == true se trabaja en grados y si type == false en radianes
    %}
    
    % Primero obtenemos el numero de filas de la matriz de parametros 
    n = size(TableOfParams);
    
    % Definimos si trabajamos en grados o en radianes 
    if type
        for i = 1:n(1)
            TableOfParams(i,1) = deg2rad(TableOfParams(i,1));
            TableOfParams(i,4) = deg2rad(TableOfParams(i,4));
        end
    end 
    
    % Definimos una matriz auxiliar donde se acumularan los productos 
    AUX = eye(4);
    
    % Realizamos los calculos necesarios
    for j = 1:n(1)
        % Calculamos la rotacion de alpha de i-1
        TMalpha = TransformationMatrix(RotationMatrix('x',TableOfParams(j,1),type),[0;0;0]);
        % Calculamos la traslacion de a de i-1
        TMa = TransformationMatrix(eye(3),[TableOfParams(j,2);0;0]);
        % Calculamos la traslacion de b de i
        TMb = TransformationMatrix(eye(3),[0;0;TableOfParams(j,3)]);
        % Calculamos la rotacion de theta de i
        TMtheta = TransformationMatrix(RotationMatrix('z',TableOfParams(j,4),type),[0;0;0]);
        
        % Calculamos la matriz de transformacion total
        TMtotal = TMalpha*TMa*TMb*TMtheta;
        
        % Calculamos el acumulado de las matrices de transformacion 
        AUX = AUX*TMtotal;
    end
    
    % Definimos la matriz de transformacion de la cinematica directa
    DC_TMatrix_Out = AUX;
end
