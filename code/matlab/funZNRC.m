function [kp, ti, td] = funZNRC(G, Ts, type)
    theta_mod = G.inputdelay + Ts/2
    K = G.Numerator{1}(end);
    tau = G.Denominator{1}(1);
    
    if theta_mod/tau < 0.1 || theta_mod/tau > 1
        disp("theta_mod/tau = " + theta_mod/tau + ". Method is not suitable")
    end

    switch type
        case 'p'
            kp = tau/(K*theta_mod);
            ti = 0;
            td = 0;
        case 'pi'
            kp = 0.9*tau/(K*theta_mod);
            ti = 3*theta_mod;
            td = 0;
        case 'pid'
            kp = 1.2*tau/(K*theta_mod);
            ti = 2*theta_mod;
            td = 0.5*theta_mod;
    end
end    
    
