function [R_Matrix_Out] = RotationMatrix(direction, angle, type)
    % Generamos la matriz de rotación deseada dependiendo de los parametros

    % Escogemos el tipo de rotación 
    % Dependiendo del valor del parametro 'type', se hacen las operaciones
    % type == true -> grados, type == false -> radianes 
    switch(direction)
        case 'x'
            if type == true
                R_Matrix_Out = [1 0 0;0 cosd(angle) -sind(angle);0 sind(angle) cosd(angle)];
            else 
                R_Matrix_Out = [1 0 0;0 cos(angle) -sin(angle);0 sin(angle) cos(angle)];
            end 
        case 'y'
            if type == true
                R_Matrix_Out = [cosd(angle) 0 sind(angle);0 1 0;-sind(angle) 0 cosd(angle)];
            else 
                R_Matrix_Out = [cos(angle) 0 sin(angle);0 1 0;-sin(angle) 0 cos(angle)];
            end 
        case 'z'
            if type == true
                R_Matrix_Out = [cosd(angle) -sind(angle) 0;sind(angle) cosd(angle) 0;0 0 1];
            else 
                R_Matrix_Out = [cos(angle) -sin(angle) 0;sin(angle) cos(angle) 0;0 0 1];
            end 
        otherwise 
            fprintf('ERROR')
    end   

end