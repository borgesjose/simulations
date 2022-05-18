function dh_dt = tank_conical(t,y,A,Qin,Cd,R1,R2)
    % calculate derivative of the Level
    
    g = 9.81;
    H = 0.7; % Altura
    %R1 = 0.15; % Raio em metros
    %R2 = 0.025; % Raio da base do cilindro
    
    b = A*Cd*sqrt(2*g);

    dh_dt = (Qin - b*y^(1/2))/(R2 + ((R1-R2)/H)*y)^(2);
end