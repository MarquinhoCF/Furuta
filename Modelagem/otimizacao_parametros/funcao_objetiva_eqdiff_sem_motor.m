function f = funcao_objetiva_eqdiff_sem_motor(x0, dados_reais, t)
    global g
       
    % Leitura dos parâmetros do sistema
    lr = x0(1);
    Ir = x0(2);
    Br = x0(3);
    mk = x0(4);
    lk = x0(5);
    Ik = x0(6);
    Bk = x0(7);
    
    y0 = [0; pi/2; 0; 0]; % [θr θk dθr/dt dθk/dt]
    u = 0; % A entrada sempre sera 0

    % Simular o sistema com o coeficiente de atrito Br fornecido
    [~, x] = ode45(@(t, x) modelo_nao_linear_pendulo_sem_motor_par(x, u, lr, Ir, Br, mk, lk, Ik, Bk), t, y0);

    % Definição da função de custo (objetivo)
    % Minimizar a soma dos quadrados das diferenças entre as posições simuladas e os dados reais
    theta_k_simulado = x(:, 2); % Extraindo a posição angular do pêndulo
    f = sum(sqrt(abs(theta_k_simulado-dados_reais).^2));
end