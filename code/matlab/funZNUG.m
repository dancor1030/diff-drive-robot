function [kp, ti, td] = funZNUG(G, type)
    [Ku, ~, wc, ~] = margin(G);
    Tu = 2*pi/wc;
    
    % K is static gain
    K = G.Numerator{1}(end);
    disp("K*Ku = " + K*Ku)
    
    if K*Ku < 2
        disp('Need a compensator')
    elseif K*Ku > 20 
        disp('use more complex algorithm')
    end    

    switch type
        case 'p'
            kp = 0.5*Ku;
            ti=0;
            td=0;
        case 'pi'
            kp = 0.45*Ku;
            ti=0.83*Tu;
            td=0;
        case 'pid'
            kp = 0.6*Ku;
            ti=0.5*Tu;
            td=0.125*Tu; 
    end
end    

