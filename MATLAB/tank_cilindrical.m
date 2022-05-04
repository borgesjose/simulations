function dh_dt = tank_cilindrical(t,y,Qin)
    % calculate derivative of the Level
    
    g = 9.81;
    H = 70; % Altura
    R = 9.00; % Raio
    Cd=0.97*0.97;
    b = Cd*sqrt(2*g);

    k1 = 1/(pi*(R)^2);
    k2 = b/(pi*(R)^2);

    dh_dt = Qin*k1 - k2*y^(-1/2);
end