% METADATA OF THE FILE
%{ 
    File: baxter_calibration_measurements.m
    Class: Industrial Robotics 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%} 

function [] = baxter_calibration_measurements(TM_input,TM_output)
    %{ 
       This function calculate the absolute, relative and percentage error
       for all the important variables of the baxter robot (Position and 
       orientation.
    %}
    
    % Define the names of the main rows of the table       
    variables = {'X-Position(m)';'Y-Position(m)';'Z-Position(m)';
                 'X-Orientation(rad)';'Y-Orientation(rad)';'Z-Orientation(rad)'};
    
    % Calculate the orientation angles for input and output matrices
    input_ori = OrientationAngles(TM_input(1:3,1:3),false);
    output_ori = OrientationAngles(TM_output(1:3,1:3),false);
    
    % Define the main arrays for the table
    Input_values = [TM_input(1,4);TM_input(2,4);TM_input(3,4);
                    input_ori(1);input_ori(2);input_ori(2)];
    Output_values = [TM_output(1,4);TM_output(2,4);TM_output(3,4);
                     output_ori(1);output_ori(2);output_ori(2)];
    
    % Calculate the errors
    Absolute_error = abs(Input_values - Output_values);
    Relative_error = Absolute_error./abs(Input_values);
    Percentage_error = Relative_error*100;
    
    % Made the table
    mltable = table(Input_values,Output_values,Absolute_error,Relative_error, ...
                    Percentage_error,'RowNames',variables);
    disp(mltable)
end