clear all;
close all;
clc

set(groot, ...
    'defaultAxesFontSize', 20, ...
    'defaultTextFontSize', 20, ...
    'defaultLineLineWidth', 1.3, ...
    'defaultAxesLineWidth', 0.8, ...
    'defaultTextInterpreter', 'latex', ...
    'defaultAxesTickLabelInterpreter', 'latex', ...
    'defaultLegendInterpreter', 'latex');

% Inicijalizacija parametara

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

noise_var=0;


%% eFL no noise
w0 = 105; 
k1 = 4*w0;
k0 = 4*w0^2;
ki = 0;
num_NF = [1];
den_NF = [1];

t = 1;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_eFL',t);
figure(1), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(2), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(3), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(4), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(5), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% eFL noise, no filter vs filter
k1 = 4*w0;
k0 = 4*w0^2;
ki = 0;

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 1;
for i = 1:2
    if i == 1
        num_NF = [1];
        den_NF = [1];
    else
        num_NF = [1];
        den_NF = [1/400 1];
    end
    sim('buck_boost_eFL',t);
    figure(6), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(7), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(8), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(9), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(10), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(6); legend('eFL', 'eFL + filtar merenja','Location','Best')
figure(7); legend('eFL', 'eFL + filtar merenja','Location','Best')
figure(8); legend('eFL', 'eFL + filtar merenja','Location','Best')
figure(9); legend('eFL', 'eFL + filtar merenja','Location','Best')
figure(10); legend('eFL', 'eFL + filtar merenja','Location','Best')

%% L robustness eFL
k1 = 4*w0;
k0 = 4*w0^2;
ki = 0;
num_NF = [1];
den_NF = [1];


t = 1;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_eFL',t);
    figure(11), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(12), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(13), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(14), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(15), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');               hold on; grid on;
end

figure(11); legend();
figure(12); legend();
figure(13); legend();
figure(14); legend();
figure(15); legend();



%% eFL + I no noise
w0 = 38;
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1];

t = 1;
noise_var = 0;
L = 15.91e-3;
sel = 1;
sim('buck_boost_eFL',t);
figure(16), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(17), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(18), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(19), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(20), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% eFL + I noise, no filter vs filter
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 1;
for i = 1:2
    if i == 1
        num_NF = [1];
        den_NF = [1];
    else
        num_NF = [1];
        den_NF = [1/400 1];
    end
    sim('buck_boost_eFL',t);
    figure(21), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(22), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(23), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(24), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(25), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(21); legend('eFL+I', 'eFL+I + filtar merenja','Location','Best')
figure(22); legend('eFL+I', 'eFL+I + filtar merenja','Location','Best')
figure(23); legend('eFL+I', 'eFL+I + filtar merenja','Location','Best')
figure(24); legend('eFL+I', 'eFL+I + filtar merenja','Location','Best')
figure(25); legend('eFL+I', 'eFL+I + filtar merenja','Location','Best')

%% L robustness eFL + I
k1 = 9*w0;
k0 = 27*w0^2;
ki = 27*w0^3;
num_NF = [1];
den_NF = [1];

t = 1;
Ls = [0.8 0.9 1 1.1 1.2];
for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    sim('buck_boost_eFL',t);
    figure(26), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(27), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(28), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(29), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(30), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');   hold on; grid on;
end

figure(26); legend();
figure(27); legend();
figure(28); legend();
figure(29); legend();
figure(30); legend();



%% Inicijalizacija SMC parametara
k0 = 1;
k1 = 1;
ki = 1;
fi = 1;



%% SMC no noise
t = 1;
noise_var = 0;
L = 15.91e-3;
sel = 1;
beta = 2e3;
k0 = 110;
k1 = 1;
num_NF = [1];
den_NF = [1];

sim('buck_boost_SMC',t);
figure(31), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(32), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(33), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(34), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(35), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

% sigma = k0*z1_out + z2_out;
% figure; plot(t_out,sigma)

%% SMC noise, no filter vs filter

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 1;
beta = 2e3;
k0 = 110;
k1 = 1;


for i = 1:2
    if i == 1
        num_NF = [1];
        den_NF = [1];
    else
        num_NF = [1];
        den_NF = [1/400 1];
    end
    sim('buck_boost_SMC',t);
    figure(36), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(37), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(38), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(39), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(40), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(36); legend('SMC', 'SMC + filtar merenja','Location','Best')
figure(37); legend('SMC', 'SMC + filtar merenja','Location','Best')
figure(38); legend('SMC', 'SMC + filtar merenja','Location','Best')
figure(39); legend('SMC', 'SMC + filtar merenja','Location','Best')
figure(40); legend('SMC', 'SMC + filtar merenja','Location','Best')

%% L robustness SMC
t = 1;
Ls = [0.8 0.9 1 1.1 1.2];
num_NF = [1];
den_NF = [1];

for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 1;
    beta = 2e3;
    k0 = 110;
    k1 = 1;
    sim('buck_boost_SMC',t);
    figure(41), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on; 
    figure(42), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(43), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(44), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(45), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');  hold on; grid on;
end

figure(41); legend();
figure(42); legend();
figure(43); legend();
figure(44); legend();
figure(45); legend();


%% SMC+I no noise

t = 1;
noise_var = 0;
L = 15.91e-3;
sel = 2;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
num_NF = [1];
den_NF = [1];

sim('buck_boost_SMC',t);
figure(46), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(47), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(48), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');  hold on; grid on;
figure(49), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(50), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

%% SMC+I noise, no filter vs fitler

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 2;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;

for i = 1:2
    if i == 1
        num_NF = [1];
        den_NF = [1];
    else
        num_NF = [1];
        den_NF = [1/400 1];
    end
    sim('buck_boost_SMC',t);
    figure(51), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(52), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(53), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(54), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(55), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(51); legend('SMC+I', 'SMC+I + filtar merenja','Location','Best')
figure(52); legend('SMC+I', 'SMC+I + filtar merenja','Location','Best')
figure(53); legend('SMC+I', 'SMC+I + filtar merenja','Location','Best')
figure(54); legend('SMC+I', 'SMC+I + filtar merenja','Location','Best')
figure(55); legend('SMC+I', 'SMC+I + filtar merenja','Location','Best')
%% L robustness SMC+I
t = 1;
Ls = [0.8 0.9 1 1.1 1.2];
num_NF = [1];
den_NF = [1];

for i = 1:length(Ls)
    L = Ls(i)*15.91e-3;
    noise_var = 0;
    sel = 2;
    beta = 2e3;
    k0 = 4*30;
    ki = 4*30^2;
    k1 = 1;
    sim('buck_boost_SMC',t);
    figure(56), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(57), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(58), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(59), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(60), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');               hold on; grid on;
end

figure(56); legend();
figure(57); legend();
figure(58); legend();
figure(59); legend();
figure(60); legend();

%% BLSMC+I no noise

t = 1;
noise_var = 0;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;
num_NF = [1];
den_NF = [1];

sim('buck_boost_SMC',t);
figure(61), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
figure(62), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(63), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
figure(64), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
figure(65), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

 

%% BLSMC+I noise, no filter vs filter

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;
sel = 3;
beta = 2e3;
k0 = 4*30;
ki = 4*30^2;
k1 = 1;
fi = 5;

for i = 1:2
    if i == 1
        num_NF = [1];
        den_NF = [1];
    else
        num_NF = [1];
        den_NF = [1/400 1];
    end
    sim('buck_boost_SMC',t);
    figure(66), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(67), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(68), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(69), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(70), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
end

figure(66); legend('BLSMC+I', 'BLSMC+I + filtar merenja','Location','Best')
figure(67); legend('BLSMC+I', 'BLSMC+I + filtar merenja','Location','Best')
figure(68); legend('BLSMC+I', 'BLSMC+I + filtar merenja','Location','Best')
figure(69); legend('BLSMC+I', 'BLSMC+I + filtar merenja','Location','Best')
figure(70); legend('BLSMC+I', 'BLSMC+I + filtar merenja','Location','Best')
%% L robustness BLSMC+I
t = 1;
Ls = [0.8 0.9 1 1.1 1.2];
num_NF = [1];
den_NF = [1];

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
    figure(71), plot(t_out,x1_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
    figure(72), plot(t_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(73), plot(t_out,u_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
    figure(74), plot(x1_out,x2_out,'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
    figure(75), plot(t_out,y_out, 'DisplayName',['$L = ' num2str(Ls(i)) '\,L_{\mathrm{nom}}$']); xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$');     hold on; grid on;
end

figure(71); legend();
figure(72); legend();
figure(73); legend();
figure(74); legend();
figure(75); legend();


%% Poredjenje BLSMC+I i eFL+I sa sumom i filtrom merenja

t = 1;
noise_var=(0.03*Vc)^2;
L = 15.91e-3;

for i = 1:2
    if i == 1
        sel = 3;
        beta = 2e3;
        k0 = 4*30;
        ki = 4*30^2;
        k1 = 1;
        fi = 5;
        num_NF = [1];
        den_NF = [1/400 1];

        sim('buck_boost_SMC',t);
        figure(81), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
        figure(82), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
        figure(83), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
        figure(84), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
        figure(85), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;
    else
        w0 = 38;
        k1 = 9*w0;
        k0 = 27*w0^2;
        ki = 27*w0^3;
        sel = 1;
        num_NF = [1];
        den_NF = [1/400 1];

        sim('buck_boost_eFL',t);
        figure(81), plot(t_out,x1_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_1\,[\mathrm{A}]$'); hold on; grid on;
        figure(82), plot(t_out,x2_out); xlabel('$t\,[\mathrm{s}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
        figure(83), plot(t_out,u_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$u$');                hold on; grid on;
        figure(84), plot(x1_out,x2_out); xlabel('$x_1\,[\mathrm{A}]$'); ylabel('$x_2\,[\mathrm{V}]$'); hold on; grid on;
        figure(85), plot(t_out,y_out);  xlabel('$t\,[\mathrm{s}]$'); ylabel('$v_c\,[\mathrm{V}]$'); hold on; grid on;

    end
    
end

figure(81); legend('BLSMC+I', 'eFL+I','Location','Best')
figure(82); legend('BLSMC+I', 'eFL+I','Location','Best')
figure(83); legend('BLSMC+I', 'eFL+I','Location','Best')
figure(84); legend('BLSMC+I', 'eFL+I','Location','Best')
figure(85); legend('BLSMC+I', 'eFL+I','Location','Best')

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


