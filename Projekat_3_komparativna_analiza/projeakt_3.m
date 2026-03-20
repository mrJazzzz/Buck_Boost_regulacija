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

x1_eq = sol.x1
x2_eq = sol.x2
u_eq = solve(x2_eq == x2, u);

% Definisanje nominalnih vrednosti parametara
L = 15.91e-3; % Induktivnost u henrima
C = 470e-6; % Kapacitans u faradima
R = 52;  % Otpornost u omima
E = 12;    % Napon u voltima


x1_eq = eval(x1_eq)
x2_eq = eval(x2_eq)

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


%% Simulacija sa sumom

noise_var = (0.03*x2_eq)^2;
t = 2;
sim('buck_boost',t);
figure(4), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(5), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(6), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% Simulacija robusnost na promene L
t = 2;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sim('buck_boost',t);
    figure(7),  plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(8),  plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(9),  plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    T_const = t_out(find(y_out <= -0.63*22.5, 1,"first")) - 1
end

figure(7);  legend();
figure(8);  legend();
figure(9);  legend();


%% Linearizacija modela u nominalnom rezimu

syms x1 x2 u L C R E;

x1dot = ((1-u)*x2 + u*E)/L;
x2dot = (-(1-u)*x1 - x2/R)/C;

A = [diff(x1dot,x1) diff(x1dot,x2); diff(x2dot,x1) diff(x2dot,x2)]
B = [diff(x1dot,u); diff(x2dot,u)]
c = [0 1]

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
G = tf(sys)

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


% num_NF_PI = [1];
% den_NF_PI = [1];
num_NF_PI = [1];
den_NF_PI = [1/50 1];

%% no noise PI

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_lin_regulacija',t);
figure(10), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(11), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(12), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% noise PI
t = 2.5;
noise_var = (0.03*x2_eq)^2;
L = 15.91e-3;
sel = 1;
sim('buck_boost_lin_regulacija',t);
figure(13), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(14), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(15), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% L robustness PI
t = 2.5;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_lin_regulacija',t);
    figure(16), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(17), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(18), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
end

figure(16); legend();
figure(17); legend();
figure(18); legend();



%% eFL

L=15.91*10^-3;   %Induktivnost [H]
C=470*10^-6;     %Kapacitivnost [F]
R=52;            %Otpornost potrosaca [Ohm]
E=12;            %Napon [V]

Vc=-22.5; %Nominalna vrednost izlazne varijable
Ye=Vc;

x10=0;
x20=0;

x2e=Vc; % vC
x1e=1.2441; % iL
ue=0.65217; % vCnom/(E-Vcnom)

gama = C*x2e^2 - 2*E*C*x2e + L*x1e^2;


%% eFL + I no noise

w0 = 38;
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1/800 1];

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_eFL',t);
figure(19), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(20), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(21), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% eFL + I noise
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;

t = 2.5;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 1;
num_NF = [1];
den_NF = [1/800 1];
sim('buck_boost_eFL',t);
figure(22), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(23), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(24), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% L robustness eFL + I
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1/800 1];

t = 2.5;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_eFL',t);
    figure(25), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(26), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(27), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
end

figure(25); legend();
figure(26); legend();
figure(27); legend();



%% SMC

%% Inicijalizacija SMC parametara
k0 = 1;
k1 = 1;
ki = 1;
fi = 1;

%% BLSMC+I no noise

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;
num_NF = [1];
den_NF = [1/800 1];

sim('buck_boost_SMC',t);
figure(28), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(29), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(30), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


 

%% BLSMC+I noise

t = 2.5;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;


num_NF = [1];
den_NF = [1/800 1]; 

sim('buck_boost_SMC',t);
figure(31), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(32), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(33), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


%% L robustness BLSMC+I
t = 2.5;
Ls = [0.8 0.9 1 1.1 1.2];
num_NF = [1];
den_NF = [1/800 1];

for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 3;
    beta = 2e3;
    k0 = 4*30;
    ki = 4*30^2;
    k1 = 1;
    fi = 5;
    sim('buck_boost_SMC',t);
    figure(34), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(35), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(36), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
end

figure(34); legend();
figure(35); legend();
figure(36); legend();



%% Komparativna analiza

%% no noise

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_lin_regulacija',t);
figure(37), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(38), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(39), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


w0 = 38;
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1/800 1];

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_eFL',t);
figure(37), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(38), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(39), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;

t = 2.5;
noise_var = 0;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;
num_NF = [1];
den_NF = [1/800 1];

sim('buck_boost_SMC',t);
figure(37), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(38), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(39), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


figure(37); legend('PI','eFL+I','BLSMC+I');
figure(38); legend('PI','eFL+I','BLSMC+I');
figure(39); legend('PI','eFL+I','BLSMC+I');


%% noise

t = 2.5;
noise_var = (0.03*Vc)^2;;
L = 15.91e-3;
sel = 1;
sim('buck_boost_lin_regulacija',t);
figure(40), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(41), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(42), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


w0 = 38;
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1/800 1];

t = 2.5;
noise_var = (0.03*Vc)^2;;
L = 15.91e-3;
sel = 1;
sim('buck_boost_eFL',t);
figure(40), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(41), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(42), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;

t = 2.5;
noise_var = (0.03*Vc)^2;;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;
num_NF = [1];
den_NF = [1/800 1];

sim('buck_boost_SMC',t);
figure(40), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(41), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(42), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;


figure(40); legend('PI','eFL+I','BLSMC+I');
figure(41); legend('PI','eFL+I','BLSMC+I');
figure(42); legend('PI','eFL+I','BLSMC+I');

%% robustness

Ls = [0.8 0.9 1 1.1 1.2];
colors = get(groot,'defaultAxesColorOrder');

for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;

    t = 2.5;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_lin_regulacija',t);
    figure(43), plot(t_out,x1_out,'Color',colors(1,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(44), plot(t_out,x2_out,'Color',colors(1,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(45), plot(t_out,u_out,'Color',colors(1,:));  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    
    
    w0 = 38;
    k1 = 9*w0;
    k0 = 27*w0^2;
    ki = 27*w0^3;
    num_NF = [1];
    den_NF = [1/800 1];
    
    t = 2.5;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_eFL',t);
    figure(43), plot(t_out,x1_out,'Color',colors(2,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(44), plot(t_out,x2_out,'Color',colors(2,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(45), plot(t_out,u_out,'Color',colors(2,:));  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    
    t = 2.5;
    noise_var = 0;
    sel = 3;
    beta = 2e3;
    k0 = 4*30;
    ki = 4*30^2;
    k1 = 1;
    fi = 5;
    num_NF = [1];
    den_NF = [1/800 1];
    
    sim('buck_boost_SMC',t);
    figure(43), plot(t_out,x1_out,'Color',colors(3,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(44), plot(t_out,x2_out,'Color',colors(3,:)); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(45), plot(t_out,u_out,'Color',colors(3,:));  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;    
end

figure(43); legend('PI','eFL+I','BLSMC+I');
figure(44); legend('PI','eFL+I','BLSMC+I');
figure(45); legend('PI','eFL+I','BLSMC+I');


%%  zoom detalja

%% zoom pozitivnog stepa no noise

fig_old = figure(38);
ax_old = fig_old.CurrentAxes;

fig_new = figure(46);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[0.6 1.1])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')


%% zoom poremecaja no noise

fig_old = figure(38);
ax_old = fig_old.CurrentAxes;

fig_new = figure(47);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[1.85 2.3])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')


%% zoom pozitivnog stepa noise

fig_old = figure(41);
ax_old = fig_old.CurrentAxes;

fig_new = figure(48);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[0.6 1.1])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')



%% zoom poremecaja noise

fig_old = figure(41);
ax_old = fig_old.CurrentAxes;

fig_new = figure(49);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[1.85 2.3])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')

%% zoom upravljanje enoise

fig_old = figure(42);
ax_old = fig_old.CurrentAxes;

fig_new = figure(50);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[0.6 0.9])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')

%% zoom pozitivnog stepa robusnost

fig_old = figure(44);
ax_old = fig_old.CurrentAxes;

fig_new = figure(51);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[0.6 1.1])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')



%% zoom poremecaja robusnost

fig_old = figure(44);
ax_old = fig_old.CurrentAxes;

fig_new = figure(52);
ax_new = copyobj(ax_old, fig_new);

set(ax_new,'Position',get(0,'DefaultAxesPosition'))

xlim(ax_new,[1.85 2.3])

lgd = legend(ax_old);
legend(ax_new, lgd.String, 'Location','best')


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