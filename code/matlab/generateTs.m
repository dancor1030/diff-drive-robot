%% COMPUTE Ts FUNCTION
function [Ts, crit, limits] = generateTs(G)

    GCLdelay = feedback(G, 1); % WITH DELAY
    % BW 
    wc = bandwidth(GCLdelay);
    UB_BW = 2*pi/(8*wc); % UPPER BOUND
    LB_BW = 2*pi/(12*wc); % LOWER BOUND

    theta = G.inputdelay;
    G.inputdelay = 0;
    GCL = feedback(G, 1); % RE-DEFINE WITH NO DELAY

    % SETTLING TIME
    ts = stepinfo(GCL).SettlingTime + theta;
    LB_TS = 0.05*ts;
    UB_TS = 0.15*ts;
    % TAU EQ
    taueq = ts/5 + theta;
    LB_TAU = 0.2*taueq;
    UB_TAU = 0.6*taueq;
    
    Ts = min([LB_BW LB_TAU LB_TS]); % sampling time
    idx = find([LB_BW LB_TAU LB_TS]==Ts); % SHOWS WHICH ONE IS SELECTED
    
    limits = [LB_BW  UB_BW; % BANDWIDTH BOUNDS
              LB_TAU UB_TAU; % EQ TAU BOUNDS
              LB_TS  UB_TS]; % SETTLING TIME BOUNDS

    switch idx
        case 1
            crit = 'BANDWIDTH';
        case 2
            crit = 'EQ. TAU';
        case 3
            crit = 'SETTLING TIME';
    end        
end    
    

