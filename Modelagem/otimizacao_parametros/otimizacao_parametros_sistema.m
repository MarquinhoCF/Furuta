clc; close all; clear all;

load('dados_do_pendulo_formatado.mat');

global g
g = 9.81; % Aceleração da gravidade (m/s^2)

% Parâmetros do sistema
mr = 0.06; % Massa do braço (Kg)
lr = 0.2; % Tamanho do braço (m)
Ir = 0.00067; % Momento de inércia do braço (kg*m^2)
Br = 0.08; % Coeficiente de atrito do braço (kg*m^2s^(-1))
mk = 0.002; % Massa do pêndulo (Kg)
lk = 0.1; % Tamanho do pêndulo (m)
Ik = 0.000067; % Momento de inércia do pêndulo (kg*m^2)
Bk = 0.00015; % Coeficiente de atrito do pêndulo (kg*m^2s^(-1))

% Intervalo de Tempo para Integração
t = [0:0.01:15];

% Condições Iniciais
y0 = [0; pi/2; 0; 0]; % [θr θk dθr/dt dθk/dt]
u = 0; % Entrada (controle)

% Simulação em Malha Aberta
[t, x] = ode45(@(t, x) modelo_nao_linear_pendulo_sem_motor_par(x, u, lr, Ir, Br, mk, lk, Ik, Bk), t, y0);

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
dados_reais = Integradorpendulodadosentregue.PosioAngularPndulorad - 7.07; % Dados de posição angular
dados_reais_rad = deg2rad(dados_reais);

% Plotagem dos dados reais
figure;
plot(time, dados_reais);
title('Posição Angular');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θk');

% Parâmetros iniciais para o otimizador
x0 = [lr, Ir, Br, mk, lk, Ik, Bk];

% Definição dos máximos e mínimos dos parâmetros
max = [0.25, 0.5, 1.0, 0.004, 0.15, 0.25, 0.5];
min = [0.15, 0.00001, 0.00001, 0.0015, 0.05, 0.00001, 0.00001];

% Definição do Problema de Otimização
options = optimset('Display', 'iter', 'MaxFunEvals', 500);
[par_otimizado, fval] = fmincon(@funcao_objetiva_eqdiff_sem_motor, x0, [], [], [], [], min, max, [], options, dados_reais_rad, time);

% Printando os parâmetros otimizados obtidos
fprintf("lr = %6f", par_otimizado(1));
disp(" ");
fprintf("Ir = %6f", par_otimizado(2));
disp(" ");
fprintf("Br = %6f", par_otimizado(3));
disp(" ");
fprintf("mk = %6f", par_otimizado(4));
disp(" ");
fprintf("lk = %6f", par_otimizado(5));
disp(" ");
fprintf("Ik = %6f", par_otimizado(6));
disp(" ");
fprintf("Bk = %6f", par_otimizado(7));
disp(" ");

% Simulação com os parâmetros otimizados
[t, x_otimizado] = ode45(@(t, x) modelo_nao_linear_pendulo_sem_motor_par(x, u, par_otimizado(1), par_otimizado(2), par_otimizado(3), par_otimizado(4), par_otimizado(5), par_otimizado(6), par_otimizado(7)), t, y0);

% Converter ângulos de radianos para graus
x_otimizado(:, 1) = rad2deg(x_otimizado(:, 1));
x_otimizado(:, 2) = rad2deg(x_otimizado(:, 2));

% Plotar Comparação das Respostas Otimizadas e a de Referência
figure;
plot(time, dados_reais, t, x_otimizado(:, 2));
title('Posições Angulares');
xlabel('Tempo (s)');
ylabel('Ângulo (graus)');
legend('θk Referência', 'θk Otimizado');

% Plotar Respostas Otimizadas em Malha Aberta
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