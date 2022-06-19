%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto           %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Otimização metaheuristica dos controladores PID    %
%  Fuzzy tipo 1 e  tipo 2 Intervalar                  %
%                                                     %
%  -- Version: 0.1  - 14/06/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Main script for metaheuristic optimization algorithms applied to a conical tank

 %% Step 1, simulation definition:
 
        Tsim = 500; % Total simulation time
        
        PIDtype = 'ZN'; %'ZN' = Ziegle-Nichols , 'CC' = Choen Coon,'AT' = Astrom, 'PR' = Teacher tunning;
        PIDflag = 0;
        FuzzyType = 'T1';% 'T1' = Tipo 1, 'T2' = Tipo 2;
        FT1type = 'L'; % L = input linear ; N = input non linear
        FT2Itype = 'L'; % L = input linear ; N = input non linear
        
        flag_load_dist = 0; 
        flag_noise = 0;
        flag_model_severance = 0;
        
        Opt_type = 'NO'; % AG = Genetic Algorithm ; PS = Particle Swarm Optimization; NO = No optimization

    
        %% Step 2 - Problem definition:
        %Tank definition structure
        tank.h0 = 0.001; % initial point
                 
        tank.R1 = 0.125;
        tank.R2 = 0.01;
        
        tank.Cv = 0.97; %velocity coefficient (water 0.97)
        tank.Cc = 0.97; %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        tank.Cd = tank.Cc*tank.Cv; % discharge coefficient

        tank.r = 0.005;% output ratio in meters

        tank.A = pi*tank.r^2;% output Area
        
        %% Step 3 - Controller definition: 

        [Kc,Ti,Td] = PID(PIDtype); % Type PID selection 
%% Step 4  - Controller definition:        

    if (PIDflag)
        disp('lol')
    else
        
    if (FuzzyType == 'T1'),
        Am_min = 2;
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;
        L = 2;
        
        % o vetor parametros dá os valores das MF's
        if (FT1type == 'L')
            param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];
        elseif (FT1type == 'N')
            gene = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];;
        end;
        
        
    end
    
    if (FuzzyType == 'T2'),
        Am_min = 2;
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;
        L = 2;
        
        % o vetor parametros dá os valores das MF's:
        
        if (FT2Itype == 'L')
            gene = [0.2377,0.0306,-0.2588,0.4572,0.5397,0.2005,0.0634,0.0350,0.4868,0.2303,0.1049,-0.0324,0.0481,0.3489,0.4641,0.2081];
        elseif (FT2Itype == 'N')
            gene =[0.2146,0.3760,-0.1644,0.4906,0.0376,0.2273,0.2379,-0.0310,0.4428,0.5785,0.3263,0.3500];
        end;
        
        Param =[gene,1,1];
    end

            
        %% Step 5, simulation setings:
        
        Ts = 5; %  5~10s( Digital control systems,Landau,2006,p.32)
        nptos = Tsim/Ts; %number point of simulation
        ts = linspace(0,Tsim,nptos); % time vector
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida
        
        ref_type = 'st'; % st = step ; us = upper stair ; ls = lower stair;
        patamar = 0.05;
        passo = 0.00;
        Tamostra = Ts;
    
        ref = ref_def(patamar,passo,nptos);
                
        %clear h;
        h(4)=tank.h0 ; h(3)=tank.h0 ; h(2)=tank.h0 ; h(1)=tank.h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;
        
        if( flag_load_dist) load('disturbio.mat'); end;
        if( flag_noise) load('ruido.mat'); end;
        

        %% Simulation with ode45;

        for i=4:nptos
            
            %Model severance
            if( flag_model_severance == 1) 
                if(i > (nptos/2)) r = 0.006; end;
                A = pi*r^2;% output area
            end;
            
            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i)% + ruido(i); %Erro

            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            Ami = 1; 

                        %Controlador:

                        Kp(i)= Kc/Ami;
                        Kd(i)= (Td)*Kc/Ami;
                        Ki(i)= (Kc/Ami)/(Ti);

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

                        u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2) ;%+ disturbio(i);

                        %saturation:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>2*3.8000e-04) u(i)=2*3.8000e-04;end;

                        tempo(i)=i*Tamostra;

        end
        
        

