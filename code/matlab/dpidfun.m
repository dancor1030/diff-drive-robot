function [Dz, q0, q1, q2] = dpidfun(Ts, kp, ti, td)
    switch nargin
        case 2 % P
            q0 = kp;
            q1 = 0;
            q2 = 0;
            num = q0;
            den = 1;
        case 3 % PI
            q0 = kp*(1+Ts/(2*ti));
            q1 = kp*(Ts/(2*ti)-1);
            q2 = 0;
            num = [q0 q1];
            den = [1 -1];
        case 4 % PID
            q0 = kp*(1 + Ts/(2*ti) + td/Ts);
            q1 = kp*(Ts/(2*ti) - 2*td/Ts - 1);
            q2 = kp*td/Ts;
            num = [q0 q1 q2];
            den = [1 -1 0];
    end
    Dz = tf(num, den, Ts);
end
    