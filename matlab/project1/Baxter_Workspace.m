% METADATA OF THE FILE
%{ 
    File: Baxter_Workspace.m
    Class: Grade Work 1 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%}

function [right_index,left_index] = Baxter_Workspace(right_limb_data,left_limb_data)
    %% Calculate the maneuverability index for each point
    right_m_index = [];
    left_m_index = [];
    
    % Create a Baxter's instance
    bx = BaxterClass();
    
    % Calculate right limb's index vector
    for i = 1:length(right_limb_data)
        right_joint_values = bx.IPK([
            -0.10901883,  0.86815814, -0.48416562, right_limb_data(i,1);
            -0.07848417, -0.4930657,  -0.86644472, right_limb_data(i,2);
            -0.99093649, -0.05645945,  0.12189011, right_limb_data(i,3);
            0,          0,          0,          1],'r','u');
        
        right_limb_jacobian = BaxterJacobian(right_joint_values);
        
        right_m_index(i) = sqrt(det(right_limb_jacobian*transpose(right_limb_jacobian)));
        disp(right_m_index(i))  
    end
    
     % We plot the left arms's points
    for j = 1:length(left_limb_data)
        left_joint_values = bx.IPK([
            -0.10901883,  0.86815814, -0.48416562, left_limb_data(j,1);
            -0.07848417, -0.4930657,  -0.86644472, left_limb_data(j,2);
            -0.99093649, -0.05645945,  0.12189011, left_limb_data(j,3);
            0,          0,          0,          1],'l','u');
        
        left_limb_jacobian = BaxterJacobian(left_joint_values);
        
        left_m_index(j) = sqrt(det(left_limb_jacobian*transpose(left_limb_jacobian)));
        disp(left_m_index(j))
    end
    
    right_index = right_m_index;
    left_index = left_m_index;
    
    %% Here the Baxter's workspace is configured
    
    % Define de Delaunay Triangulation for each limb
    rdt = delaunayTriangulation(right_limb_data);
    ldt = delaunayTriangulation(left_limb_data);
    
    % Create a convexHull intance for each limb
    [K_r,v_r] = convexHull(rdt);
    [K_l,v_l] = convexHull(ldt);
    
    % Define the color's contrains
    P_wo_0_r = [-0.2780,-0.0640,1.3744];
    P_wo_0_l = [0.2780,-0.0640,1.3744];
    
    C_r = sqrt((right_limb_data(:,1)-P_wo_0_r(1)).^2+...
               (right_limb_data(:,2)-P_wo_0_r(2)).^2+...
               (right_limb_data(:,3)-P_wo_0_r(3)).^2);
    C_l = sqrt((left_limb_data(:,1)-P_wo_0_l(1)).^2+...
               (left_limb_data(:,2)-P_wo_0_l(2)).^2+...
               (left_limb_data(:,3)-P_wo_0_l(3)).^2);
    
    % Plot the Baxter's workspace
    plot3(P_wo_0_r(1),P_wo_0_r(2),P_wo_0_r(3),'.','MarkerSize',20,'Color','black')
    plot3(P_wo_0_l(1),P_wo_0_l(2),P_wo_0_l(3),'.','MarkerSize',20,'Color','black')
    
    trisurf(K_r,rdt.Points(:,1),rdt.Points(:,2),rdt.Points(:,3),C_r,...
            'FaceAlpha',0.6,'LineStyle','none')
    hold on 
    
    trisurf(K_l,ldt.Points(:,1),ldt.Points(:,2),ldt.Points(:,3),C_l,...
            'FaceAlpha',0.6,'LineStyle','none')
    
    title('BAXTER WORKSPACE')
    grid on 
    xlabel('x axis (m)')
    ylabel('y axis (m)')
    zlabel('z axis (m)')
    
end
