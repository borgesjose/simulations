function dh_dt = tank_cilindrical(t,y,A,Qin,Cd)
    % calculate derivative of the Level
    
    g = 9.81;
    R = 0.042; % Raio em metros
    
    b = A*Cd*sqrt(2*g);
    
    k1 = 1/(pi*(R)^2);
    k2 = b/(pi*(R)^2);

    dh_dt = Qin*k1 - k2*y^(1/2);
end