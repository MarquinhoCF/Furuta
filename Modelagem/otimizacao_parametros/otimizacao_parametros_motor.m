clc; close all; clear all;

load("dados_com_motor.mat")

global g
g = 9.81; % Aceleração da gravidade (m/s^2)

% Parâmetros do sistema
ng = 0.9; % Rendimento das engrenagens
nm = 0.75; % Rendimento do motor
kg = 80; % Constante das engrenagens
kt = 0.03; % Constante de acoplamento do motor (N*m/A)
km = 0.15; % Constante de torque eletromotivo do servomotor (N*m/A)
R = 40; % Resistência (ohms)

% Condições Iniciais
y0 = [0; pi; 0; 0]; % [θr θk dθr/dt dθk/dt]

% Plota o vetor da entrada (controle)
figure;
plot(time, tensao);
xlabel('Tempo (s)');
ylabel('Amplitude (V)');
title('Tensão PWM');
grid on;

% Definindo o intervalo de tempo para a simulação
tspan = [0, time(end)];

% Simulação do modelo com parâmetros do motor não otimizados
[t, x] = ode45(@(t, x) modelo_nao_linear_pendulo_com_motor_par(t, x, time, tensao, ng, nm, kg, kt, km, R), tspan, y0);

% Converter ângulos de radianos para graus
x(:, 1) = rad2deg(x(:, 1));
x(:, 2) = rad2deg(x(:, 2));

% Plotar Respostas em Malha Aberta
figure;
subplot(2, 1, 1);
plot(t, x(:, 1), t, x(:, 2));
title('Posições Angulares');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θr', 'θk');

subplot(2, 1, 2);
plot(t, x(:, 3), t, x(:, 4));
title('Velocidades Angulares');
xlabel('Tempo (s)');
ylabel('Velocidade Angular (rad/s)');
legend('dθr/dt', 'dθk/dt');

% Dados Reais Coletados
dados_reais = dadoscommotor.PosioAngularPnduloEmgraus + 180;
dados_reais_rad = deg2rad(dados_reais);

% Plotagem dos Dados Reais
figure;
plot(time, dados_reais, t, x(:, 2));
title('Posições Angulares');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θk Referência', 'θk Simulado');

% Plotagem da entrada e os dados reais
figure;
subplot(2, 1, 1);
plot(time, tensao);
title('PWM de entrada');
xlabel('Tempo (s)');
ylabel('Tensão (V)');

subplot(2, 1, 2);
plot(time, dados_reais);
title('Posição Angular');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');

% Parâmetros iniciais para o otimizador
x0 = [ng, nm, kg, kt, km, R];

% Definição dos máximos e mínimos dos parâmetros
max = [0.95, 0.95, 150, 0.04, 0.20, 50];
min = [0.7, 0.7, 50, 0.005, 0.05, 2];

% Definição do Problema de Otimização
options = optimset('Display', 'iter', 'MaxFunEvals', 500);
[par_otimizado, fval] = fmincon(@funcao_objetiva_eqdiff_com_motor, x0, [], [], [], [], min, max, [], options, dados_reais_rad, time, tensao);

% Printando os parâmetros otimizados obtidos
fprintf("ng = %6f", par_otimizado(1));
disp(" ");
fprintf("nm = %6f", par_otimizado(2));
disp(" ");
fprintf("kg = %6f", par_otimizado(3));
disp(" ");
fprintf("kt = %6f", par_otimizado(4));
disp(" ");
fprintf("km = %6f", par_otimizado(5));
disp(" ");
fprintf("R = %6f", par_otimizado(6));
disp(" ");

% Simulação com parâmetros otimizados
[t, x_otimizado] = ode45(@(t, x) modelo_nao_linear_pendulo_com_motor_par(t, x, time, tensao, par_otimizado(1), par_otimizado(2), par_otimizado(3), par_otimizado(4), par_otimizado(5), par_otimizado(6)), tspan, y0);

% Converter ângulos de radianos para graus
x_otimizado(:, 1) = rad2deg(x_otimizado(:, 1));
x_otimizado(:, 2) = rad2deg(x_otimizado(:, 2));

% Plotar Respostas em Malha Aberta
figure;
subplot(2, 1, 1);
plot(t, x_otimizado(:, 1), t, x_otimizado(:, 2));
title('Posições Angulares');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θr', 'θk');

subplot(2, 1, 2);
plot(t, x_otimizado(:, 3), t, x_otimizado(:, 4));
title('Velocidades Angulares');
xlabel('Tempo (s)');
ylabel('Velocidade Angular (rad/s)');
legend('dθr/dt', 'dθk/dt');

% Plotar Resposta Otimizada
figure;
plot(time, dados_reais, t, x_otimizado(:, 2));
title('Posições Angulares');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θk Referência', 'θk Otimizado');