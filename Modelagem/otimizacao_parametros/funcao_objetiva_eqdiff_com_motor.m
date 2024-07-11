function f = funcao_objetiva_eqdiff_com_motor(x0, dados_reais, time, tensao)
    global g
    
    ng = x0(1); % Rendimento das engrenagens
    nm = x0(2); % Rendimento do motor
    kg = x0(3); % Constante das engrenagens
    kt = x0(4); % Constante de acoplamento do motor (N*m/A)
    km = x0(5); % Constante de torque eletromotivo do servomotor (N*m/A)
    R = x0(6); % Resistência (ohms)
       
    % Definido as condições iniciais
    y0 = [0; pi; 0; 0]; % [θr θk dθr/dt dθk/dt]
    
    % Definindo o intervalo de tempo para a simulação
    tspan = [0, time(end)];

    % Simular o sistema com o coeficiente de atrito Br fornecido
    [t, x] = ode45(@(t, x) modelo_nao_linear_pendulo_com_motor_par(t, x, time, tensao, ng, nm, kg, kt, km, R), tspan, y0);

    % Definição da função de custo (objetivo)
    % Minimizar a soma dos quadrados das diferenças entre as posições simuladas e os dados reais
    % Interpolação para garantir que os dados simulados estejam no mesmo intervalo de tempo que os dados reais
    theta_k_simulado = interp1(t, x(:, 2), time);
    f = sum(sqrt(abs(theta_k_simulado-dados_reais).^2));
end