% METADATA OF THE FILE
%{ 
    File: Baxter_Workspace.m
    Class: Grade Work 1 
    Author: Elkin Javier Guerra Galeano and Santiago Garcia Arango
    Created on 02-28-2021, 11:00
%}

function [] = Baxter_Workspace(rightdata27feb,leftdata27feb)
    %% Here the Baxter's workspace is configured
    % Workspace params 
    axis(gca,'equal');
    
    % We plot the right arms's points
    for i = 1:length(rightdata27feb)
        plot3(rightdata27feb(i,1),rightdata27feb(i,2),rightdata27feb(i,3),'.','color','r')
        hold on
    end
    
     % We plot the left arms's points
    for j = 1:length(leftdata27feb)
        plot3(leftdata27feb(j,1),leftdata27feb(j,2),leftdata27feb(j,3),'.','color','b')
        hold on
    end
    
    title('BAXTER WORKSPACE')
    
end
