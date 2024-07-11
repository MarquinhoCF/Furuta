clear all; close all; clc

% Declaração dos parâmtros otimizados
Jp = 0.000013; % Momento de inércia do pêndulo
mp = 0.003205; % Massa do pêndulo
Lp = 0.085275; % Comprimento do pêndulo
Lr = 0.153331; % Comprimento do braço do pêndulo
eta_p = 0.856519; % Eficiência da polia
kg = 50.024570; % Ganho da engrenagem
eta_m = 0.824368; % Eficiência do motor
kt = 0.021524; % Constante de torque do motor
Rm = 49.999750; % Resistência do motor
Km = 0.0157308; % Constante de back-EMF do motor
Br = 0.323912; % Coeficiente de amortecimento do rotor
g = 9.81; % Aceleração devido à gravidade
Jr = 0.004284; % Momento de inércia do rotor
Bp = 0.000042; % Coeficiente de amortecimento do pêndulo

% Calculando J_T (momento de inercia total)
JT = (Jp * mp * Lr^2 + Jr * Jp + 0.25 * Jr * mp * Lp^2);

% Matrizes de estado
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (1/JT)*(1/4*mp^2*Lp^2*Lr*g), -(1/JT)*(((Jp + 1/4*mp*Lp^2)*(eta_p*kg*eta_m*kt*kg*Km))/Rm + (Jp + 1/4*mp*Lp^2)*Br), -(1/JT)*(1/2*mp*Lp*Lr*Bp);
     0, (1/JT)*(1/2*mp*Lp*g*(Jr + mp*Lr^2)), -(1/JT)*(1/2*mp*Lp*Lr*(eta_p*kg*eta_m*kt*kg*Km)/Rm), -(1/JT)*(Jr + mp*Lr^2)*Bp];

B = [0;
     0;
     (1/JT)*(Jp + 1/4*mp*Lp^2)*eta_p*kg*eta_m*kt/Rm;
     (1/JT)*(1/2*mp*Lp*Lr)*eta_p*kg*eta_m*kt/Rm];

C = eye(4);

D = [0;0;0;0];

sys_ss = ss(A,B,C,D);

Ts = 0.06;

sys_d = c2d(sys_ss,Ts,'zoh');

co = ctrb(sys_d);
ob = obsv(sys_d);

controllability = rank(co); 
observability = rank(ob);

% Definição dos pesos da função de custo do LQR
Q = [1 0 0 0;
    0 100 0 0;
    0 0 1 0;
    0 0 0 1];
R = 10;

[Kd, S, E] = dlqr(sys_d.A,sys_d.B,Q,R);

disp(Kd)
