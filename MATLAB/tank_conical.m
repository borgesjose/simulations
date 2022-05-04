function dh_dt = tank_conical(t,y,Qin)
    % calculate derivative of the Level
    
    g = 9.81;
    H = 70; % Altura
    R = 9.00; % Raio
    Cd=0.97*0.97;
    b = Cd*sqrt(2*g)
    
    k1 = 1/(pi*(R/H)^2);
    k2 = b/(pi*(R/H)^2);

    dh_dt = (Qin*k1*y^(-2)) - k2*y^(-3/2);
end