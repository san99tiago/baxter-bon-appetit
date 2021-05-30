% METADATA OF THE FILE
%{ 
    File: Baxter_Simulation.m
    Class: Industrial Robotics 
    Authors:
     Elkin Javier Guerra Galeano
     Santiago Garcia Arango
%}

% Simulacion trayectorias suaves robot Baxter

function [] = Baxter_Simulation(br,tp,linear,or)
    %% CONFIGURAMOS EL ESPACIO DE TRABAJO DEL BAXTER
    % Definimos parametros iniciales 
    time = []; % Vector de tiempo
    p = struct('r',[],'l',[]); % Matriz de posicion lineal 
    v = struct('r',[],'l',[]); % Matriz de velocidad lineal 
    a = struct('r',[],'l',[]); % Matriz de aceleracion lineal 
    je = struct('r',[],'l',[]); % Matriz de jerk lineal
    angles = struct('r',[],'l',[]); % Matriz de orientacion
    i = 0; % Indice general
    tta = struct('r',[],'l',[]); % Matriz de los dof
    lines_r = []; % Vector de lineas del brazo derecho
    lines_l = []; % Vector de lineas del brazo izquierdo
    
    % Parametros del espacio de trabajo 
    axis(gca,'equal'); 
  
    for t = 0:tp(2):tp(1)
        %% Construimos los vectores asociados a las cantidades cinematicas
        i = i + 1;
        time(i) = t;
        for j =1:3
            % BRAZO DERECHO
            % Cantidades traslacionales
            [p.r(i,j),v.r(i,j),a.r(i,j),je.r(i,j)] = Trayectoria_5_orden(t,tp(1),linear.r.p0(j),...
            linear.r.pf(j),linear.r.v0(j),linear.r.vf(j),linear.r.a0(j),linear.r.af(j));
            % Cantidades de la orientacion
            angles.r(i,j) = Trayectoria_5_orden(t,tp(1),or.r.p0(j),...
            or.r.pf(j),or.r.v0(j),or.r.vf(j),or.r.a0(j),or.r.af(j));
            % BRAZO IZQUIERDO
            % Cantidades traslacionales
            [p.l(i,j),v.l(i,j),a.l(i,j),je.l(i,j)] = Trayectoria_5_orden(t,tp(1),linear.l.p0(j),...
            linear.l.pf(j),linear.l.v0(j),linear.l.vf(j),linear.l.a0(j),linear.l.af(j));
            % Cantidades de la orientacion
            angles.l(i,j) = Trayectoria_5_orden(t,tp(1),or.l.p0(j),...
            or.l.pf(j),or.l.v0(j),or.l.vf(j),or.l.a0(j),or.l.af(j));
        end
        % Definimos las matrices de transformacion 
        FPK = struct('r',TransformationMatrix(RotationMatrixExpand(angles.r(i,:),false),transpose(p.r(i,:))),...
                     'l',TransformationMatrix(RotationMatrixExpand(angles.l(i,:),false),transpose(p.l(i,:))));
        %Cuadramos los numeros que son muy pequeños 
        for jr = 1:4
            for kr = 1:4
                if abs(FPK.r(kr,jr)) < 10^-8
                    FPK.r(kr,jr) = 0;
                end
            end
        end
        for jl = 1:4
            for kl = 1:4
                if abs(FPK.l(kl,jl)) < 10^-8
                    FPK.l(kl,jl) = 0;
                end
            end
        end
        
        %% REALIZAMOS LA CINEMATICA INVERZA PARA EL BAXTER EN CODO ARRIBA
        dof_r = br.IPK(FPK.r,'r','u');
        dof_l = br.IPK(FPK.l,'l','u');
        tta.r(:,i) = dof_r;
        tta.l(:,i) = dof_l;
        
        %% GRAFICAMOS LOS BRAZOS DE BAXTER
        % Calculamos los puntos de los brazos
        ptos_r = br.Puntos_Baxter(dof_r,'r');
        ptos_l = br.Puntos_Baxter(dof_l,'l');
        
        % Graficamos el soporte y los hombres de baxter
        sopor = plot3([0 0],[0 0],[0 br.H],'LineWidth',8,'Color','black');
        sho_r = plot3([0 -br.L],[0 -br.h],[br.H br.H],'LineWidth',8,'Color','black');
        sho_l = plot3([0 br.L],[0 -br.h],[br.H br.H],'LineWidth',8,'Color','black');
        
        for w = 1:8
            title('BAXTER SIMULATION ELKIN GUERRA')
            % Graficamos brazo derecho
            lines_r(w) = plot3([ptos_r(1,w) ptos_r(1,w+1)],[ptos_r(2,w) ptos_r(2,w+1)],...
            [ptos_r(3,w) ptos_r(3,w+1)],'Linewidth',4,'Color','red');
            hold on
            % Graficamos brazo izquierdo
            lines_l(w) = plot3([ptos_l(1,w) ptos_l(1,w+1)],[ptos_l(2,w) ptos_l(2,w+1)],...
            [ptos_l(3,w) ptos_l(3,w+1)],'Linewidth',4,'Color','red');
            hold on
        end
        
        % Graficamos la trayectoria de los brazos de baxter
        Trayectory_r = plot3(p.r(:,1),p.r(:,2),p.r(:,3),'Color','#0072BD','Linewidth',1.5);
        hold on
        Trayectory_l = plot3(p.l(:,1),p.l(:,2),p.l(:,3),'Color','#0072BD','Linewidth',1.5);
        hold on
        % Cuadramos los limites del espacio de trabajo
        xlim([-3 3])
        ylim([-3 1])
        zlim([0 3])
    
        %% RECETEAMOS LOS PARAMETROS DE LA SIMULACION 
        % Damos un tiempo de espera
        pause(0.001);
        % Eliminamos las lineas anteriores 
        delete(lines_r);
        delete(lines_l);
        delete(sopor);
        delete(sho_r);
        delete(sho_l);
        delete(Trayectory_r);
        delete(Trayectory_l);
        
        %% MOSTRAMOS EL VALORES DE LOS DOH EN TODO MOMENTO 
        fprintf('Theta1->7 Right =\n')
        disp(dof_r)
        fprintf('\nTheta1->7 Left =\n')
        disp(dof_l)
        
    end
    
    %% GRAFICAMOS LAS CANTIDADES CINEMATICAS
    % Graficamos las cantidades del brazo derecho
    figure()
    subplot(3,2,1)
    plot(time,p.r(:,1),time,p.r(:,2),time,p.r(:,3))
    title('CURVAS DE POSICION (RIGHT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion (m)')
    grid on
    subplot(3,2,2)
    plot(time,v.r(:,1),time,v.r(:,2),time,v.r(:,3))
    title('CURVAS DE VELOCIDAD (RIGHT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Velocidad (m/s)')
    grid on
    subplot(3,2,3)
    plot(time,a.r(:,1),time,a.r(:,2),time,a.r(:,3))
    title('CURVAS DE ACELERACION (RIGHT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Aceleracion (m/s^2)')
    grid on
    subplot(3,2,4)
    plot(time,je.r(:,1),time,je.r(:,2),time,je.r(:,3))
    title('CURVAS DE JERK (RIGHT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Jerk (m/s^3)')
    grid on
    subplot(3,2,5)
    plot(time,tta.r(1,:),time,tta.r(2,:),time,tta.r(3,:))
    title('CURVA DE THETA1->4 (RIGHT)')
    legend('t1','t2','t4')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion angular (rad)')
    grid on
    subplot(3,2,6)
    plot(time,tta.r(4,:),time,tta.r(5,:),time,tta.r(6,:))
    title('CURVA DE THETA5->7 (RIGHT)')
    legend('t5','t6','t7')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion angular (rad)')
    grid on
    
    % Graficamos las cantidades del brazo izquierdo
    figure()
    subplot(3,2,1)
    plot(time,p.l(:,1),time,p.l(:,2),time,p.l(:,3))
    title('CURVAS DE POSICION (LEFT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion (m)')
    grid on
    subplot(3,2,2)
    plot(time,v.l(:,1),time,v.l(:,2),time,v.l(:,3))
    title('CURVAS DE VELOCIDAD (LEFT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Velocidad (m/s)')
    grid on
    subplot(3,2,3)
    plot(time,a.l(:,1),time,a.l(:,2),time,a.l(:,3))
    title('CURVAS DE ACELERACION (LEFT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Aceleracion (m/s^2)')
    grid on
    subplot(3,2,4)
    plot(time,je.l(:,1),time,je.l(:,2),time,je.l(:,3))
    title('CURVAS DE JERK (LEFT)')
    legend('x','y','z')
    xlabel('Tiempo (segundos)')
    ylabel('Jerk (m/s^3)')
    grid on
    subplot(3,2,5)
    plot(time,tta.l(1,:),time,tta.l(2,:),time,tta.l(3,:))
    title('CURVA DE THETA1->4 (LEFT)')
    legend('t1','t2','t4')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion angular (rad)')
    grid on
    subplot(3,2,6)
    plot(time,tta.l(4,:),time,tta.l(5,:),time,tta.l(6,:))
    title('CURVA DE THETA5->7 (LEFT)')
    legend('t5','t6','t7')
    xlabel('Tiempo (segundos)')
    ylabel('Posicion angular (rad)')
    grid on
end