function [ISE, IAE, ITAE] = computeIndices(error, Ts)
    ISE = 0;
    IAE = 0;
    ITAE = 0;
    
    n = length(error);
    
    for k = 1:n
        t = (k - 1) * Ts; % Tempo corrente
        e = error(k);     % Errore corrente
        
        ISE = ISE + (e^2) * Ts;
        IAE = IAE + abs(e) * Ts;
        ITAE = ITAE + t * abs(e) * Ts;
    end
end