function xdot = modelo_nao_linear_pendulo_com_motor_par(t, x, t_data, u_data, ng, nm, kg, kt, km, R)
    global g

    % Parâmetros otimizados do sistema
    mr = 0.06; % Massa do braço (Kg)
    lr = 0.153331; % Tamanho do braço (m)
    Ir = 0.004284; % Momento de inércia do braço (kg*m^2)
    Br = 0.323912; % Coeficiente de atrito do braço (kg*m^2s^(−1))
    mk = 0.003205; % Massa do pêndulo (Kg)
    lk = 0.085275; % Tamanho do pêndulo (m)
    Ik = 0.000013; % Momento de inércia do pêndulo (kg*m^2)
    Bk = 0.000042; % Coeficiente de atrito do pêndulo (kg*m^2s^(−1))
    g = 9.81; % Aceleração devido à gravidade (m/s^2)

    % Leitura das variáveis de estado e comando (malha aberta)
    theta_r = x(1);
    theta_k = x(2);
    dtheta_r = x(3);
    dtheta_k = x(4);

    % Modelo da planta em malha aberta
    xdot = zeros(4, 1);

    % Interpolando os valores de tensão ao longo do tempo. 
    %   Isso é crucial porque os dados de tensão podem não estar disponíveis exatamente nos 
    % mesmos pontos de tempo em que as soluções do modelo dinâmico são calculadas. Ao interpolar 
    % a tensão, garantimos que a função de entrada (tensão aplicada ao motor) esteja devidamente 
    % alinhada com os pontos de tempo do modelo dinâmico, permitindo uma comparação precisa entre 
    % os dados simulados e os dados reais.
    tensao = interp1(t_data, u_data, t);

    % Equação do motor
    motor = ((ng*nm*kg*kt)/R)*tensao - ((ng*nm*km*kt*(kg^2))/R)*x(3);

    xdot(1) = dtheta_r;
    xdot(2) = dtheta_k;
    xdot(3) = (Bk * mk * cos(theta_k) * lk * lr * dtheta_k - Br * mk * (lk^2) * dtheta_r + ((mk^2) * (lk^4) * dtheta_r + mk * (lk^2) * Ik * dtheta_r) * sin(2 * theta_k) * dtheta_k ...
        + (((mk^2) * (lk^3) * lr + mk * lk * lr * Ik) * (dtheta_k^2) - (mk^2) * (lk^3) * lr * (cos(theta_k)^2) * (dtheta_k^2) - (mk^2) * g * (lk^2) * lr * cos(theta_k)) * sin(theta_k) ...
        - Br * Ik * dtheta_r + (mk * (lk^2) + Ik) * motor) / (((lr^2) * Ik + (lk^2) * Ir) * mk + ((mk^2) * (lk^4) + mk * (lk^2) * Ik) * (sin(theta_k)^2) ...
        - (mk^2) * (lk^2) * (lr^2) * (cos(theta_k)^2) + (mk^2) * (lk^2) * (lr^2) + Ir * Ik);
    xdot(4) = -(-(mk^2) * sin(2 * theta_k) * cos(theta_k) * lr * (lk^3) * dtheta_r * dtheta_k - ((mk^2) * cos(theta_k) * (lk^4) * (dtheta_r^2) ...
        + (mk^2) * g * (lk^3)) * (sin(theta_k)^3) - Br * mk * cos(theta_k) * lk * lr * dtheta_r + (Bk * mk * (lr^2) + Bk * Ir) * dtheta_k + Bk * mk * (sin(theta_k^2)) * (lk^2) * dtheta_k ...
        + mk * cos(theta_k) * lk * lr * motor + (((mk^2) * (lr^2) * (lk^2) * (dtheta_k^2) - (mk^2) * (lr^2) * (lk^2) * (dtheta_r^2)) * cos(theta_k) - (mk^2) * g * lk * (lr^2) ...
        - mk * g * lk * Ir) * sin(theta_k)) / (((mk^2) * (lk^4) + mk * (lk^2) * Ik) * (sin(theta_k)^2) + ((lr^2) * Ik + (lk^2) * Ir) * mk + Ik * Ir ...
        - (mk^2) * (cos(theta_k)^2) * (lk^2) * (lr^2) + (mk^2) * (lk^2) * (lr^2));
end