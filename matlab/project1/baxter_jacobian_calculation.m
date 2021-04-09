% METADATA OF THE FILE
%{ 
    File: baxter_jacobian_calculation.m
    Class: Industrial Robotics 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%} 

function [J_0] = baxter_jacobian_calculation(limb)

    %{
        This script calculates the baxter end effector's jacobian for future 
        work with the baxter robot of the EIA University.
    %}

    %% Define the main variables
    syms L1 L2 L3 L4 L5 t1 t2 t3 t4 t5 t6

    %% Define the end effector's jacobian in 4 coordinates

    j_1_1 = ((L1 + L2*cos(t2))*cos(t4) + (L3*sin(t4) + L4)*cos(t2))*sin(t3) + ...
            + L5*(sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4))*sin(t5);

    j_2_1 = -((L1 + L2*cos(t2))*sin(t4) - L3*cos(t2)*cos(t4))*sin(t3) - L5*(...
            sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4))*sin(t5) + L5*cos(t2)*...
            sin(t3)*cos(t5);

    j_3_1 = (L1 + L2*cos(t2))*cos(t3) - L3*sin(t2) - L4*(sin(t2)*sin(t4) - cos(t2)*cos(t3)*...
            cos(t4)) - L5*(sin(t2)*cos(t4) + cos(t2)*cos(t3)*sin(t4))*cos(t5);

    J_4 = [j_1_1, (L2*cos(t4) + L3*sin(t4) + L4)*cos(t3) - L5*sin(t3)*sin(t4)*sin(t5), -L5*cos(t4)*sin(t5), L4, -L5*sin(t5),0,0;
           j_2_1, (-L2*sin(t4) + L3*cos(t4) + L5*cos(t5))*cos(t3) - L5*sin(t3)*cos(t4)*sin(t5), L5*sin(t4)*sin(t5), L5*cos(t5),0,0,0;
           j_3_1, -(L2 + L4*cos(t4) - L5*sin(t4)*cos(t5))*sin(t3), L3 + L4*sin(t4) + L5*cos(t4)*cos(t5),0, L5*cos(t5),0,0;
           sin(t2)*sin(t4) - cos(t2)*cos(t3)*cos(t4), sin(t3)*cos(t4), -sin(t4),0,0, -sin(t5), cos(t5)*sin(t6);
           sin(t2)*sin(t4) + cos(t2)*cos(t3)*cos(t4), -sin(t3)*sin(t4), -cos(t4),0,-1,0, -cos(t6);
           cos(t2)*sin(t3), cos(t3),0,1,0, cos(t5), sin(t5)*sin(t6)];

    %% Define the rotation matrix between W0 and 4
    
    % Depends of the limb the rotation matrix between wo and 0 en calulated
    if (limb == 'right')
        R_W0_0 = [-sqrt(2)/2, sqrt(2)/2,0;
                  -sqrt(2)/2, -sqrt(2)/2,0;
                  0, 0, 1];
    elseif (limb == 'left')
          R_W0_0 = [sqrt(2)/2, sqrt(2)/2,0;
                    -sqrt(2)/2, sqrt(2)/2,0;
                    0, 0, 1];   
    end
    
    % Calculate the rotation matrix between 0 and 1
    R_0_1 = [cos(t1), -sin(t1),0;
             sin(t1), cos(t1),0;
             0,0,1];
         
    % Calculate the rotation matrix between 1 and 2
    R_1_2 = [-sin(t2), -cos(t2),0;
             0, 0, 1;
             -cos(t2), sin(t2),0];
         
    % Calculate the rotation matrix between 2 and 3     
    R_2_3 = [cos(t3), -sin(t3),0;
             0, 0, -1;
             sin(t3), cos(t3),0];
         
    % Calculate the rotation matrix between 3 and 4
    R_3_4 = [cos(t4), -sin(t4),0;
             0, 0, 1;
             -sin(t4), -cos(t4),0];
    
    R_W0_4 = R_W0_0*R_0_1*R_1_2*R_2_3*R_3_4;

    %% Calculate the end effector's jabobian with respect to 0

    J_0 = simplify([R_W0_4,zeros(3);
           zeros(3), R_W0_4]*J_4);
   
end