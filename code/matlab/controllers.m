clc
clear all
close all
%% PLANT DEF.
clc

Gs = tf(0.9627, [0.2025 1]); % G SPEED

[Ts, crit, limits] = generateTs(Gs);

% Ts = limits(2, 2)

Gsd = c2d(Gs, Ts, 'zoh') % G SPEED DISCRETE


%% ZN-RC PID : ziegler nichols REACTION CURVE
clc

[kp_pid, ti_pid, td_pid] = funZNRC(Gs, Ts, 'pid');
[Dz_pid, q0_pid, q1_pid, q2_pid] = dpidfun(Ts, kp_pid, ti_pid, td_pid);
[kp_pid, ti_pid, td_pid]'
Dz_pid

%% ZN-UG PI : ziegler nichols ULTIMATE GAIN
clc

[kp_pi, ti_pi, ~] = funZNUG(Gs, 'pi');
[Dz_pi, q0_pi, q1_pi, ~] = dpidfun(Ts, kp_pi, ti_pi);
[kp_pi, ti_pi]'
Dz_pi_original = Dz_pi
%% 
clc
% ------------------------------------------------

% TUNING %

% create the PID controller you want using 
% kp, ti and td (based on the original parameters computed)

kp = 1.63;
ti = 0.2;
td = 0;
[kp ti td]'
[~, q0, q1, q2] = dpidfun(Ts, kp, ti, td);

if q2 == 0
    num = [q0 q1];
    den = [1 -1];
else
    num = [q0 q1 q2];
    den = [1 -1 0];
end

[q0 q1 q2]'

Dz = tf(num, den, Ts);
Dz

%% for SIMULINK
clc
G = Gs
Gd = Gsd

% LIMITS FOR SATURATION
maxlim = 80;
inflim = 15;



