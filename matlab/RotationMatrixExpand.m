function [R_Matrix_Out] = RotationMatrixExpand(angles, type)
    % Definimos primero si los angulos son dados en radianes o en grados
    % Definimos parametros importantes 
    gamma = angles(1);
    beta = angles(2);
    alpha = angles(3);
    
    % Definimos la matris de rotacion extendida
    if type
        R_Matrix_Out = RotationMatrix('z', alpha, true)*RotationMatrix('y', beta, true)*RotationMatrix('x', gamma, true);
    else
        R_Matrix_Out = RotationMatrix('z', alpha, false)*RotationMatrix('y', beta, false)*RotationMatrix('x', gamma, false);
    end
   
end 