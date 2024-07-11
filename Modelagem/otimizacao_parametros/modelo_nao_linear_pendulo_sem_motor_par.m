function xdot = modelo_nao_linear_pendulo_sem_motor_par(x, u, lr, Ir, Br, mk, lk, Ik, Bk)
    global g

    % Parâmetros do sistema
    mr = 0.06; % Massa do braço (Kg)

    % Leitura das variáveis de estado e comando (malha aberta)
    theta_r = x(1);
    theta_k = x(2);
    dtheta_r = x(3);
    dtheta_k = x(4);

    % Modelo da planta em malha aberta
    xdot = zeros(4, 1);

    xdot(1) = dtheta_r;
    xdot(2) = dtheta_k;
    xdot(3) = (Bk * mk * cos(theta_k) * lk * lr * dtheta_k - Br * mk * (lk^2) * dtheta_r + ((mk^2) * (lk^4) * dtheta_r + mk * (lk^2) * Ik * dtheta_r) * sin(2 * theta_k) * dtheta_k ...
        + (((mk^2) * (lk^3) * lr + mk * lk * lr * Ik) * (dtheta_k^2) - (mk^2) * (lk^3) * lr * (cos(theta_k)^2) * (dtheta_k^2) - (mk^2) * g * (lk^2) * lr * cos(theta_k)) * sin(theta_k) ...
        - Br * Ik * dtheta_r + (mk * (lk^2) + Ik) * u) / (((lr^2) * Ik + (lk^2) * Ir) * mk + ((mk^2) * (lk^4) + mk * (lk^2) * Ik) * (sin(theta_k)^2) ...
        - (mk^2) * (lk^2) * (lr^2) * (cos(theta_k)^2) + (mk^2) * (lk^2) * (lr^2) + Ir * Ik);
    xdot(4) = -(-(mk^2) * sin(2 * theta_k) * cos(theta_k) * lr * (lk^3) * dtheta_r * dtheta_k - ((mk^2) * cos(theta_k) * (lk^4) * (dtheta_r^2) ...
        + (mk^2) * g * (lk^3)) * (sin(theta_k)^3) - Br * mk * cos(theta_k) * lk * lr * dtheta_r + (Bk * mk * (lr^2) + Bk * Ir) * dtheta_k + Bk * mk * (sin(theta_k^2)) * (lk^2) * dtheta_k ...
        + mk * cos(theta_k) * lk * lr * u + (((mk^2) * (lr^2) * (lk^2) * (dtheta_k^2) - (mk^2) * (lr^2) * (lk^2) * (dtheta_r^2)) * cos(theta_k) - (mk^2) * g * lk * (lr^2) ...
        - mk * g * lk * Ir) * sin(theta_k)) / (((mk^2) * (lk^4) + mk * (lk^2) * Ik) * (sin(theta_k)^2) + ((lr^2) * Ik + (lk^2) * Ir) * mk + Ik * Ir ...
        - (mk^2) * (cos(theta_k)^2) * (lk^2) * (lr^2) + (mk^2) * (lk^2) * (lr^2));
end