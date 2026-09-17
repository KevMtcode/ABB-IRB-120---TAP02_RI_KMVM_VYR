%% Direct Kinematic Model
% DH Matrix - DH=[a_ij alpha_ij s_j theta_j]
function T_H = FKplotframes(DH, only_return, show_corr)
    T_corr = [0 0 1 0; 0 -1 0 0; 1 0 0 0; 0 0 0 1];   % marco DH_6 -> flange URDF
    frame=size(DH,1);
    
    %Reconstruye la matriz homogenea 4*4
    for i=1:frame
        T_ij(:,:,i)=T_DH(DH(i,1),DH(i,2),DH(i,3),DH(i,4)); %MARCO LOCAL
    end
    T(:,:,1)=T_ij(:,:,1); %Dejar fijo el primer frame
    for i=1:frame-1
        T(:,:,i+1)=T(:,:,i)*T_ij(:,:,i+1); %Postmultiplicación para mover los siguientes
    end
    if only_return == 1 && show_corr == 1 % solo para comparar transformadas homogéneas con Moveit
        T(:,:,frame) = T(:,:,frame) * T_corr;
        T_H =  T;
    elseif only_return == 1 && show_corr == 0 % para los cálculos con Jacobiano
        T_H =  T;
    else
        % Graph:
        % Define the Fixed Frame
        hf=figure(1);
        set(hf,'position',[445   275   563   628])
        % Unit vectors scale factor(visual)
        L=50;
        h_ejexF=plot3([0 L],[0 0],[0 0],'r','linewidth',2.5);
        hold on
        h_ejeyF=plot3([0 0],[0 L],[0 0],'g','linewidth',2.5);
        hold on
        h_ejezF=plot3([0 0],[0 0],[0 L],'b','linewidth',2.5);
        hold on
        axis equal
        axis([-50 50 -50 50 -50 60])
        xlabel('X [mm]')
        ylabel('Y [mm]')
        zlabel('Z [mm]')
        xlim([-200, 700])
        ylim([-500, 400])
        zlim([0, 700])
        grid on
        view(27,37)
        % Mobile Coordinate System Based on DoF
        for i=1:frame
            h_ejexM(i,1)=plot3([0 0],[0 0],[0 0],'r','linewidth',2);
            hold on
            h_ejeyM(i,1)=plot3([0 0],[0 0],[0 0],'g','linewidth',2);
            hold on
            h_ejezM(i,1)=plot3([0 0],[0 0],[0 0],'b','linewidth',2);
        end
        % Points Matrix on {j}
        Mp=[0,0,0,1;
            L,0,0,1;
            0,L,0,1;
            0,0,L,1];
        % Points Matrix on {0}
        for k=1:frame
            for i=1:4
                Mp_T(i,:,k)=(T(:,:,k)*Mp(i,:)')'; % MARCO GLOBAL, %Premultiplicación
            end
        end
        % Plot each Coordinate System in the Global Frame
        for k=1:frame
            set(h_ejexM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(2,1,k)],...
                'ydata',[Mp_T(1,2,k) Mp_T(2,2,k)],...
                'zdata',[Mp_T(1,3,k) Mp_T(2,3,k)])
            set(h_ejeyM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(3,1,k)],...
                'ydata',[Mp_T(1,2,k) Mp_T(3,2,k)],...
                'zdata',[Mp_T(1,3,k) Mp_T(3,3,k)])
            set(h_ejezM(k,1),'xdata',[Mp_T(1,1,k) Mp_T(4,1,k)],...
                'ydata',[Mp_T(1,2,k) Mp_T(4,2,k)],...
                'zdata',[Mp_T(1,3,k) Mp_T(4,3,k)])
        end
    end
end
% Transform
function T_ij = T_DH(a_ij, alpha_ij, d_i, theta_i)
T_ij = [ cos(theta_i),                -sin(theta_i),                0,             a_ij;
         cos(alpha_ij)*sin(theta_i),   cos(alpha_ij)*cos(theta_i), -sin(alpha_ij), -d_i*sin(alpha_ij);
         sin(alpha_ij)*sin(theta_i),   sin(alpha_ij)*cos(theta_i),  cos(alpha_ij),  d_i*cos(alpha_ij);
         0, 0, 0, 1 ];
end