%% Inicijalizacija sistema - nominalne vrednosti parametara modela
clc
clear all
close all

set(groot, ...
    'defaultAxesFontSize', 20, ...
    'defaultTextFontSize', 20, ...
    'defaultLineLineWidth', 1.3, ...
    'defaultAxesLineWidth', 0.8, ...
    'defaultTextInterpreter', 'latex', ...
    'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex');


syms x1 x2 u;
syms R L C E positive
assumeAlso(u >= 0 & u < 1)

x1dot = ((1-u)*x2 + u*E)/L;
x2dot = (-(1-u)*x1 - x2/R)/C;

equations = [x1dot == 0, x2dot == 0];
sol = solve(equations, [x1 x2], 'Real', true,'ReturnConditions',true);

x1_eq = sol.x1;
x2_eq = sol.x2;
u_eq = solve(x2_eq == x2, u);

% Definisanje nominalnih vrednosti parametara
L = 15.91e-3; % Induktivnost [H]
C = 470e-6; % Kapacitet [F]
R = 52;  % Otpornost [Ohm]
E = 12;    % Napon [V]

x1_eq = eval(x1_eq);
x2_eq = eval(x2_eq);

%% Trazenje nominalnog rezima i upravljanja

x2_eq = -22.5;
x2 = -22.5;
u_eq = eval(u_eq);
x1_eq = subs(x1_eq,u,u_eq);

%% Simulacija bez suma

noise_var = 0;
t = 2;
sim('buck_boost',t);
figure(1), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(2), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(3), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(4), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(5), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% Simulacija sa sumom

noise_var = (0.03*x2_eq)^2;
t = 2;
sim('buck_boost',t);
figure(6), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(7), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(8), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(9), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(10), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;


%% Simulacija robusnost na promene L
t = 2;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sim('buck_boost',t);
    figure(11),  plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(12),  plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(13),  plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(14),  plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(15), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
    T_const = t_out(find(y_out <= -0.63*22.5, 1,"first")) - 1
end

figure(11);  legend();
figure(12);  legend();
figure(13);  legend();
figure(14);  legend();
figure(15); legend();

%% Linearizacija modela u nominalnom rezimu

syms x1 x2 u L C R E;

x1dot = ((1-u)*x2 + u*E)/L;
x2dot = (-(1-u)*x1 - x2/R)/C;

A = [diff(x1dot,x1) diff(x1dot,x2); diff(x2dot,x1) diff(x2dot,x2)];
B = [diff(x1dot,u); diff(x2dot,u)];
c = [0 1];

syms s
f = det(s*eye(2)-A);

x1 = double(x1_eq); x2 = double(x2_eq); u = double(u_eq);
% Definisanje nominalnih vrednosti parametara
L = 15.91e-3; % Induktivnost u henrima
C = 470e-6; % Kapacitans u faradima
R = 52;  % Otpornost u omima
E = 12;    % Napon u voltima

A = eval(A);
B = eval(B);
f = eval(f);
sys = ss(A, B, c, 0);
G = tf(sys);

% polovi i nule
% figure; pzmap(G);


%% Projektovanje kontrolera, PI
s = tf('s');
%figure(16); rlocus(G)
% K1 = 0.0003;
K1 = 0.00941;
T = K1*G/(1+K1*G);
T = minreal(T);
%figure; pzmap(T);
%figure; step(T)

% K2 = K1 * (s + 250)/s*3*(-1);
K2 = K1 * (s + 250)/s*(-1)*0.1;
%figure; rlocus(K2*G)
T = minreal(K2*G/(1+K2*G));
%figure; margin(K2*G)
%figure; step(T);
[num_PI,den_PI]=tfdata(K2,'v');
num = num_PI;
den = den_PI;


num_NF_PI = 1;
den_NF_PI = 1;
% num_NF_PI = [1];
% den_NF_PI = [1/50 1];

%% Kontroler na bazi inverzije dinamike
pole(G)
zero(G)

Gz = zpk(G);
z = zero(Gz);
z(real(z) > 0) = -z(real(z) > 0);
G_approx = zpk(z, pole(Gz), -Gz.K);



w0 = 80;
K = w0/s * G_approx^-1;

W = minreal(K * G);

% figure; pzmap(W);

[num_INV,den_INV]=tfdata(K,'v');
num = num_INV;
den = den_INV;

num_NF_INV = [1];
den_NF_INV = [1];
% num_NF_INV = [1];
% den_NF_INV = [1/300 1];


%% Podesavanja PID kao ZN, relejni eksperiment

t = 3;
noise_var = 0;
L = 15.91e-3;
sim('buck_boost_ZN',t);
figure(20), plot(t_out,e_out); xlabel('t [s]'); ylabel('e [V]'); hold on; grid on;

%%  Odredjivanje parametara kontrolera
Tu = 0.036;
Ampl = 8.184;
Ku = 4*0.1*u_eq/(pi*Ampl);


K = 0.6*Ku;
Ti = Tu/2;
Td = Tu/8;

s = tf('s');
K1 = -K*(1 + 1/(Ti*s) + Td*s/((Td/10)*s + 1));
% figure; rlocus(K1*G)


%% Izmestanje diferencijalnog dejstva u povratnu granu
K_ff =  -K*(1 + 1/(Ti*s));
K_fb = -K*Td*s/((Td/10)*s + 1);

[num1,den1]=tfdata(K_ff,'v');
[num2,den2]=tfdata(K_fb,'v');

num_NF_ZN = [1];
den_NF_ZN = [1];
% num_NF_ZN = [1];
% den_NF_ZN = [1/400 1];

%% no noise PI
t = 6;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_regulacija_fin',t);
figure(31), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(32), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(33), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(34), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(35), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% noise PI
t = 6;
noise_var = (0.03*x2_eq)^2;
L = 15.91e-3;
sel = 1;
sim('buck_boost_regulacija_fin',t);
figure(36), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(37), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(38), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(39), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(40), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% L robustness PI
t = 6;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_regulacija_fin',t);
    figure(41), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(42), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(43), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(44), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(45), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');               hold on; grid on;
end

figure(41); legend();
figure(42); legend();
figure(43); legend();
figure(44); legend();
figure(45); legend();

%% no noise INV
t = 6;
noise_var = 0;
L = 15.91e-3;
sel = 2;
sim('buck_boost_regulacija_fin',t);
figure(51), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(52), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(53), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(54), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(55), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% noise INV
t = 6;
noise_var = (0.03*x2_eq)^2;
L = 15.91e-3;
sel = 2;
sim('buck_boost_regulacija_fin',t);
figure(56), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(57), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(58), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(59), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(60), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;


%% L robustness INV

t = 6;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 2;
    sim('buck_boost_regulacija_fin',t);
    figure(61), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(62), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(63), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$'); hold on; grid on;
    figure(64), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(65), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(61); legend();
figure(62); legend();
figure(63); legend();
figure(64); legend();
figure(65); legend();

%% no noise ZN
t = 6;
noise_var = 0;
L = 15.91e-3;
sel = 3;
sim('buck_boost_regulacija_fin',t);
figure(71), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(72), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(73), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(74), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(75), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% noise ZN
t = 6;
noise_var = (0.03*x2_eq)^2;
L = 15.91e-3;
sel = 3;
sim('buck_boost_regulacija_fin',t);
figure(76), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(77), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(78), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(79), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(80), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% L robustness ZN
t = 6;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 3;
    sim('buck_boost_regulacija_fin',t);
    figure(81), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(82), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(83), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(84), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(85), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');               hold on; grid on;
end

figure(81); legend();
figure(82); legend();
figure(83); legend();
figure(84); legend();
figure(85); legend();


%% Komparativna analaiza kontrolera

num_NF_PI = [1];
den_NF_PI = [1/50 1];

num_NF_INV = [1];
den_NF_INV = [1/300 1];

num_NF_ZN = [1];
den_NF_ZN = [1/400 1];

%% PI sa sumom, bez filtra merenja vs. sa filtrom merenja
L = 15.91e-3;
sel = 1;
noise_var = (0.03*x2_eq)^2;
t = 6;
for i = 1:2
    if i == 1
        num_NF_PI = [1];
        den_NF_PI = [1];
    else
        num_NF_PI = [1];
        den_NF_PI = [1/50 1];
    end
    sim('buck_boost_regulacija_fin.slx',t);
    figure(86), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(87), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(88), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(89), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(90), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(86); legend('PI', 'PI + filtar merenja')
figure(87); legend('PI', 'PI + filtar merenja')
figure(88); legend('PI', 'PI + filtar merenja')
figure(89); legend('PI', 'PI + filtar merenja')
figure(90); legend('PI', 'PI + filtar merenja')


%% INV sa sumom, bez filtra merenja vs. sa filtrom merenja
L = 15.91e-3;
sel = 2;
noise_var = (0.03*x2_eq)^2;
t = 6;
for i = 1:2
    if i == 1
        num_NF_INV = [1];
        den_NF_INV = [1];
    else
        num_NF_INV = [1];
        den_NF_INV = [1/300 1];
    end
    sim('buck_boost_regulacija_fin.slx',t);
    figure(91), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(92), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(93), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(94), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(95), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(91); legend('INV', 'INV + filtar merenja')
figure(92); legend('INV', 'INV + filtar merenja')
figure(93); legend('INV', 'INV + filtar merenja')
figure(94); legend('INV', 'INV + filtar merenja')
figure(95); legend('INV', 'INV + filtar merenja')


%% ZN sa sumom, bez filtra merenja vs. sa filtrom merenja
L = 15.91e-3;
sel = 3;
noise_var = (0.03*x2_eq)^2;
t = 6;
for i = 1:2
    if i == 1
        num_NF_ZN = [1];
        den_NF_ZN = [1];
    else
        num_NF_ZN = [1];
        den_NF_ZN = [1/400 1];
    end
    sim('buck_boost_regulacija_fin.slx',t);
    figure(96), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(97), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(98), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(99), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(100), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(96); legend('ZN 2DOF', 'ZN 2DOF + filtar merenja')
figure(97); legend('ZN 2DOF', 'ZN 2DOF + filtar merenja')
figure(98); legend('ZN 2DOF', 'ZN 2DOF + filtar merenja')
figure(99); legend('ZN 2DOF', 'ZN 2DOF + filtar merenja')
figure(100); legend('ZN 2DOF', 'ZN 2DOF + filtar merenja')


%% Poredjenje sva 3 kontrolera sa sumom + filtar merenja
num_NF_PI = [1];
den_NF_PI = [1/50 1];

num_NF_INV = [1];
den_NF_INV = [1/300 1];

num_NF_ZN = [1];
den_NF_ZN = [1/400 1];

L = 15.91e-3;
sels = [3 2 1];
noise_var = (0.03*x2_eq)^2;
t = 6;
for i = 1:length(sels)
    sel = sels(i);
    sim('buck_boost_regulacija_fin.slx',t);
    figure(101), plot(t_out,x1_out, 'LineWidth',1.5); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(102), plot(t_out,x2_out, 'LineWidth',1.5); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(103), plot(t_out,u_out, 'LineWidth',1.5); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(104), plot(x1_out,x2_out, 'LineWidth',1.5); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(105), plot(t_out,y_out, 'LineWidth',1.5); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(101); legend('ZN 2DOF', 'Inverzija dinamike', 'PI')
figure(102); legend('ZN 2DOF', 'Inverzija dinamike', 'PI')
figure(103); legend('ZN 2DOF', 'Inverzija dinamike', 'PI')
figure(104); legend('ZN 2DOF', 'Inverzija dinamike', 'PI')
figure(105); legend('ZN 2DOF', 'Inverzija dinamike', 'PI')

%%  export grafika

exportFolder = 'figures';
if ~exist(exportFolder, 'dir')
    mkdir(exportFolder);
end

figHandles = findall(0,'Type','figure');

for k = 1:length(figHandles)
    fig = figHandles(k);

    leg = findobj(fig,'Type','Legend');
    if ~isempty(leg)
        leg.Interpreter = 'latex';
        leg.AutoUpdate = 'off';
        
        leg.Position(3) = leg.Position(3) * 1.2;
        
        ax = findobj(fig,'Type','Axes');
        axRight = ax(1).Position(1) + ax(1).Position(3);  
        legRight = leg.Position(1) + leg.Position(3); 
        if legRight > axRight
            leg.Position(1) = leg.Position(1) - (legRight - axRight + 0.01);
        end
    end

    drawnow; 

    figName = ['figure_' num2str(fig.Number)];
    exportgraphics(fig, fullfile(exportFolder, [figName '.pdf']), 'ContentType','vector');
    close(fig);
end