% METADATA OF THE FILE
%{ 
    File: BaxterClass.m
    Class: Industrial Robotics 
    Author: Elkin Javier Guerra Galeano
    Created on 09-16-2020, 16:14
%}

% Definimos la clase BAXTER 
classdef BaxterClass
    %% Definimos los atributos de baxter
    properties
        % Medidas de los brazos de baxter
        l0 = 0.27035;
        l1 = 0.06900;
        l2 = 0.36435;
        l3 = 0.06900;
        l4 = 0.37429;
        l5 = 0.01000;
        %l5 = 0;
        l6 = 0.36830;
        % Medidas adicionales 
        L = 0.27800;
        h = 0.06400;
        H = 1.10400;
    end
    
    %% Definimos los metodos de baxter
    methods
        % Metodo para calcular la FPK 
        function [Two_t] = FPK(obj,dof,arm,ndof)
            Two_t = FPK(dof,[obj.l0;obj.l1;...
            obj.l2;obj.l3;obj.l4;obj.l5;obj.l6;obj.L;obj.h;obj.H],arm,ndof);
        end 
        % Metodo para calcular la IPK (6 dof)
        function [vdof] = IPK(obj,FPK,arm,type)
            vdof = IPK(FPK,[obj.l0;obj.l1;obj.l2;...
            obj.l3;obj.l4;obj.l6;obj.L;obj.h;obj.H],arm,type);
        end
        % Metodo para calcular la IPK (7 dof)
        function [vdof] = IPK_7dof(obj,FPK,arm,dof_0,iter,delta,tol)
            vdof = IPK_7dof(FPK,[obj.l0;obj.l1;obj.l2;...
            obj.l3;obj.l4;obj.l6;obj.L;obj.h;obj.H],arm,dof_0,iter,delta,tol);
        end
        % Metodo para calcular los puntos del cuerpo de baxter
        function [P_list] = Puntos_Baxter(obj,dof,arm)
            P_list = Puntos_Simulacion(dof,[obj.l0;obj.l1;...
            obj.l2;obj.l3;obj.l4;obj.l5;obj.l6;obj.L;obj.h;obj.H],arm);
        end
    end
end