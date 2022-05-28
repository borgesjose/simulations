function K = AT_PID_FG(Am,L,a,b,c),


    %% Definições do controlador AT-PID-FG: 
        Am_min = 2; 
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;

        Theta_m = (180/2)*(1-(1/Am));

    %% Sintonizanodo o PID:

        K = (pi/(2*Am*L))*[b;c;a];


end

