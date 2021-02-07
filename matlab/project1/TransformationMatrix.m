function [T_Matrix_Out] = TransformationMatrix(RotationMatrix,PositionOriginOS)
    % Generamos la matrix de transformación deseada dada por los parametros
    T_Matrix_Out = [RotationMatrix PositionOriginOS;0 0 0 1];
    
end

