function J = JacobianVelocities(T_H)
    % Jacobiano de velocidades angulares:
    p_i = ones(1, 6); %revolute
    Jw = zeros(3, 6);
    for i=1:6
        Jw(:, i) = p_i(i)*T_H(1:3, 3, i);
    end
    % Jacobiano de velocidades lineales:
    Jv = zeros(3, 6);
    for i=1:6
        Jv(:, i) = cross(T_H(1:3, 3, i),(T_H(1:3, 4, 6) - T_H(1:3, 4, i)));
    end
    % Jacobiano completo de velocidades:
    J = [Jv;Jw];
end