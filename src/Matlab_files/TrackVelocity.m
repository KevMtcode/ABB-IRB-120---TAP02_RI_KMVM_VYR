% Cálculo de la velocidad de la herramienta en cada punto del tramo:
function v_t = TrackVelocity(data_q, DH_H)
    DH_H(:,1) = DH_H(:,1) / 1000;
    DH_H(:,3) = DH_H(:,3) / 1000;
    DH = zeros(6,4,size(data_q,1));
    T_H = zeros(4,4,6,size(data_q,1)); % Cuatro dimensiones: fila, columna, frames y puntos
    Jb = zeros(6,6,size(data_q,1));
    qv_i = zeros(6, 1, size(data_q,1));
    vtool_i = zeros(6,1,size(data_q,1));
    vtool_linear_i = zeros(size(data_q,1),1);
    for i=1:size(data_q,1) % Inicializar los DH con el de Home, cada punto será una capa de la matriz
        DH(:, :, i) = DH_H;
    end
    for i=1:size(data_q,1)
        for j=8:13
            qv_i(j-7,1,i) = data_q(i,j);
        end
    end
    for i=1:size(data_q,1) % cada punto será una capa de la matriz
        for j=2:7 % empezando en 2 porque t es la primera columna
            DH(j-1,4,i) = DH(j-1,4,i) + data_q(i,j);
        end
        T_H(:,:,:, i) = FKplotframes(DH(:,:,i), 1, 0); % Transformada homogénea para cada punto
        Jb(:,:, i) = JacobianVelocities(T_H(:,:,:,i)); % Jacobiano para cada punto
        vtool_i(:,:,i) = Jb(:,:,i)*qv_i(:,:,i); % Vector de velocidad de la herramienta
        vtool_linear_i(i, 1) = sqrt(vtool_i(1,1,i)^2 + vtool_i(2,1,i)^2 + vtool_i(3,1,i)^2); %Magnitud de la velocidad de la herramienta
    end
    figure;
    plot(data_q(:,1), vtool_linear_i(:,1))
    grid on
    hold off
    xlabel('Tiempo(s)')
    ylabel('Velocidad(m/s)')
end