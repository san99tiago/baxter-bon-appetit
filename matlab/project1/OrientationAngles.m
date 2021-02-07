function [O_Angles] = OrientationAngles(RotationMatrixExpand_p, type)
    % Primero definimos si los angulos se dan en grados o en radianes 
    % Definimos parametros importantes 
    r11 = RotationMatrixExpand_p(1,1);
    r21 = RotationMatrixExpand_p(2,1);
    r31 = RotationMatrixExpand_p(3,1);
    r32 = RotationMatrixExpand_p(3,2);
    r33 = RotationMatrixExpand_p(3,3);
    
    % Definimos los angulos de la orientación 
    if type
        beta = atan2d(-r31, sqrt(r11^2 + r21^2));
        
        O_Angles = [atan2d(r32/cosd(beta), r33/cosd(beta));
                    beta;
                    atan2d(r21/cosd(beta), r11/cosd(beta))];
    else
        beta = atan2(-r31, sqrt(r11^2 + r21^2));
        
        O_Angles = [atan2(r32/cos(beta), r33/cos(beta));
                    beta;
                    atan2(r21/cos(beta), r11/cos(beta))];
    end
end 