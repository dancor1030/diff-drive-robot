%% First Order Identification with delay F.O.D.
clc
clear all
close all

filename = 'data50.txt';

data = importdata(filename);

ts = 0.04; 


uvec_raw = data(:, 3);
yvec_raw = data(:, 2);
t_raw = data(:, 1); % in samples



%----------------------
for n=2:length(uvec_raw)
    if (uvec_raw(n) - uvec_raw(n-1) ~= 0)
        idx = n;
        break
    end
end    
idx = 1
uvec = uvec_raw(idx:end);
yvec = yvec_raw(idx:end);
% uvec = uvec_raw;
% yvec = yvec_raw;

%%

% u0 = uvec_raw(1);
u0 = 0;
uss = uvec_raw(end);
deltaU = uss-u0;

y0 = yvec(1);
yss = yvec(end);
deltaY = yss-y0;

%%
t = 0:1:(length(yvec)-1); % obtain a vector of samples [1 2 3 4 ...]
t = ts.*t; % obtain real t vector (using sample time) in seconds.

%----------------------
%FOR PLOTTING

% yvecPLOT = [y0*ones([round(0.05*length(yvec)) 1]); yvec];
% uvecPLOT = [u0*ones([round(0.05*length(uvec)) 1]); uvec];
yvecPLOT = yvec;
uvecPLOT = uvec;

tPLOT = 0:1:(length(yvecPLOT)-1); % obtain a vector of samples [1 2 3 4 ...]
tPLOT = ts.*tPLOT; % obtain real t vector (using sample time) in seconds.

figure(1)
set(gca, 'TickLabelInterpreter', 'latex')
hold on
plot(tPLOT, yvecPLOT, 'LineWidth',1, 'Color','r')
plot(tPLOT, uvecPLOT, 'LineWidth',1, 'Color','b')
grid on
ylabel('Magnitude', 'Interpreter','latex')
xlabel('Time [s]', 'Interpreter','latex')
% ----------------------------------------

y28 = y0+deltaY*0.283;
y63 = y0+deltaY*0.632;

[~, idx_y28] = min(abs(yvec - y28));
[~, idx_y63] = min(abs(yvec - y63));
t1 = t(idx_y28);
t2 = t(idx_y63);

A = [1 1/3;
     1 1];
b = [t1; t2];
x = inv(A)*b;

theta = x(1);
tau = x(2);

if theta < 0
    theta = 0;
end    

K = deltaY/deltaU;

G = tf(K, [tau 1], 'InputDelay', theta)

[ystep, tstep] = step(deltaU*G);
ystep = ystep + y0;

plot(tstep, ystep,'LineWidth',1, 'Color','g')
legend('$y(t)$ real', '$u(t)$', '$y(t)$ model', 'Interpreter', 'latex')








