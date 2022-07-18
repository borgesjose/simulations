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
        %clear;clc;
        
        Tsim = 500; % Total simulation time
        
        PIDtype = 'ZN'; %'ZN' = Ziegle-Nichols , 'CC' = Choen Coon,'AT' = Astrom, 'PR' = Teacher tunning;
        PIDflag = 0;
        FuzzyType = 'T1';% 'T1' = Tipo 1, 'T2' = Tipo 2;
        FT1type = 'L'; % L = input linear ; N = input non linear
        FT2Itype = 'N'; % L = input linear ; N = input non linear
        
        flag_load_dist = 0; 
        flag_noise = 0;
        flag_sinusoidal_dist = 0;
        flag_model_severance = 0;
        
        Opt_type = 'AG'; % AG = Genetic Algorithm ; PS = Particle Swarm ; NO = No optimization

%%        
        if(PIDflag) simName = 'PID';
        else simName = FuzzyType;
        end;
        
        if(flag_load_dist) simName = 'Load_disturbace'; end;
        if(flag_noise) simName = 'Noise'; end;
        if(flag_sinusoidal_dist) simName = 'Sinusoidal_Noise'; end;
        if(flag_model_severance) simName = 'Model_Severance'; end;
  
        %% Step 2 - Problem definition:
        %Tank definition structure
        tank.h0 = 0.001; % initial point
        
        tank.H = 0.375;          
        tank.R1 = 0.125;
        tank.R2 = 0.01;
        
        tank.Cv = 0.97; %velocity coefficient (water 0.97)
        tank.Cc = 0.97; %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        tank.Cd = tank.Cc*tank.Cv; % discharge coefficient

        tank.r = 0.005;% output ratio in meters

        tank.A = pi*tank.r^2;% output Area
        
        %% Step 3 - Controller definition: 

        [Kc,Ti,Td] = PID(PIDtype); % Type PID selection 

        %% Step 6, Definições de otimização:
       
        % PSO
            pso.noP = 8;
            pso.maxIter = 100;
            pso.wMax = 0.9;
            pso.wMin= 0.2;
            pso.c1= 2;
            pso.c2= 2;


            pso.visFlag = 1;
            

            pso.fobj = @objfunc;

        
       % AG
       
            ag.prob_mutation = 0.10;%rand(1);
            ag.prob_crossover = 0.9;%rand(1);
            ag.geracoes = 50;

            ag.populacao_size = 128; %defino o tamanho da população
            ag.N_mais_aptos = 32;

            ag.objfunction = @objfunc;
            
            ag.visFlag = 1;
   
            L = 2;
        %% Step 7, Otimização:
        
        if(Opt_type == 'AG') 
            [param] = opt_AG(FuzzyType,FT1type,FT2Itype,L,ag);
        end;
        if(Opt_type == 'PS')         
            [param] = opt_PSO(FuzzyType,FT1type,FT2Itype,L,pso);
        end;
        
        
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
            %param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];
        elseif (FT1type == 'N')
            %param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];;
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
            %gene = [0.2377,0.0306,-0.2588,0.4572,0.5397,0.2005,0.0634,0.0350,0.4868,0.2303,0.1049,-0.0324,0.0481,0.3489,0.4641,0.2081];
            
            %Resultado´para AG:
            %param = [0.1809,0.3258,0.1362,0.2374,0.0156,0.2217,0.6816,0.0021,0.5516,0.2100,0.0979,0.0679,0.0178,0.2848,1.8989,0.0048]
            
            %Resultado para PSO:

            %param = .3*ones(1,16)
           Param =[param,1,1];
        elseif (FT2Itype == 'N')
            %gene =[0.2146,0.3760,-0.1644,0.4906,0.0376,0.2273,0.2379,-0.0310,0.4428,0.5785,0.3263,0.3500];
            %param=[0.3232,0.4712,0.0218,0.4454,0.5986,0.1102,0.2554,0.0081,0.3159,1.9916,0.9286,0.2525];
        end;
        
    end

    end      
        %% Step 5, simulation setings:
        
        Ts = 5; %  5~10s( Digital control systems,Landau,2006,p.32)
        nptos = Tsim/Ts; %number point of simulation
        ts = linspace(0,Tsim,nptos); % time vector
        H=nptos; % Horizon
       
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida
        
        ref_type = 'st'; % st = step ; us = upper stair ; ls = lower stair;
        patamar = 0.05;
        passo = 0.10;
        Tamostra = Ts;
    
        ref = ref_def(patamar,passo,nptos);
                
        %clear h;
        h(4)=tank.h0 ; h(3)=tank.h0 ; h(2)=tank.h0 ; h(1)=tank.h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;
        
        if( flag_load_dist) load('disturbio.mat'); end;
        if( flag_noise) load('ruido.mat'); end;
        

               
        %% Step 8, Simulation with ode45;

        for i=4:nptos
            
            %Model severance
            if( flag_model_severance) 
                if(i > (nptos/2)) tank.r = 0.006; end;
                tank.A = pi*tank.r^2;% output area
            end;
            
            [~,y] = ode45(@(t,y) tank_conical(t,y,u(i-1),tank),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting
            
            
            if( flag_model_severance)
                erro(i)=ref(i) - h(i) ; %Erro
            elseif( flag_sinusoidal_dist)
                 erro(i)=ref(i) - h(i) + sinusoidal_dist(ts(i))
            else
                erro(i)=ref(i) - h(i);
            end
            
            
            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            if (PIDflag)
                Ami = 1;
            else
                if (FuzzyType == 'T1'),
                    
                    Am(i) = FT1_pid_ag(erro(i),rate(i),L,param,FT1type);
                    Ami = Am(i)*Am_max + Am_min*(1 - Am(i));
                end
                
                if (FuzzyType == 'T2'),
                    
                    Am(i) =Inferencia_T2(erro(i),rate(i),L,param,FT2Itype);
                    Ami = Am(i)*Am_max + Am_min*(1 - Am(i));
                    
                end
                
            end
                        %Controlador:

                        Kp(i)= Kc/Ami;
                        Kd(i)= (Td)*Kc/Ami;
                        Ki(i)= (Kc/Ami)/(Ti);

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

                        if (flag_load_dist) 
                            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2) + disturbio(i);
                        else
                            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
                        end;

                        %saturation:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>2*3.8000e-04) u(i)=2*3.8000e-04;end;

                        tempo(i)=i*Tamostra;

        end
        %% Step 7, Saving and ploting results
        
        if (PIDflag)
            ISE_pid  = objfunc(erro,tempo,'ISE')
            ITSE_pid = objfunc(erro,tempo,'ITSE')
            ITAE_pid = objfunc(erro,tempo,'ITAE')
            IAE_pid  = objfunc(erro,tempo,'IAE')
            
            I_pid = esforco_ponderado(erro,u,H,100)
            IG_pid = IG(H,1e4,1e9,1,u,ref,h)
            
            sy_pid= var(h)
            su_pid = var(u)
            
            fileName = ['Resluts for PID - ' , PIDtype,' - ',ref_type,' - ',simName];
            save( ['./results/',fileName])
           [fig1,fig2] =  p_pid(ts,h,ref,u,tempo,Kp,Kd,Ki)
            
        elseif (FuzzyType == 'T1'),
            
            if (FT1type == 'L')
                I_t1 = esforco_ponderado(erro,u,H,100)
                
                ISE_t1  = objfunc(erro,tempo,'ISE')
                ITSE_t1 = objfunc(erro,tempo,'ITSE')
                ITAE_t1 = objfunc(erro,tempo,'ITAE')
                IAE_t1  = objfunc(erro,tempo,'IAE')
                
                IG_t1 = IG(H,1e4,1e9,1,u,ref,h)
                
                sy_t1= var(h)
                su_t1 = var(u)
                
            fileName = ['Resluts for PID - FT1-FG ' , PIDtype, ' - ', FuzzyType ,' - ' , FT1type, ' - ',ref_type,' - ',simName];
            save( ['./results/',fileName])
            
             p_ft1(ts,h,ref,u,tempo,Kp,Kd,Ki,Am)
                
            elseif (FT1type == 'N')
                I_t1 = esforco_ponderado(erro,u,H,100)
                
                ISE_t1  = objfunc(erro,tempo,'ISE')
                ITSE_t1 = objfunc(erro,tempo,'ITSE')
                ITAE_t1 = objfunc(erro,tempo,'ITAE')
                IAE_t1  = objfunc(erro,tempo,'IAE')
                
                IG_t1 = IG(H,1e4,1e9,1,u,ref,h)
                
                sy_t1= var(h)
                su_t1 = var(u)
                
                fileName = ['Resluts for PID - FT1-FG ' , PIDtype, ' - ', FuzzyType ,' - ' , FT1type, ' - ',ref_type,' - ',simName];
                save( ['./results/',fileName])
                
               p_ft1_nl(ts,h,ref,u,tempo,Kp,Kd,Ki,Am)
                
            end;
            
            
        elseif (FuzzyType == 'T2'),
            
            if (FT2Itype == 'L')
                I_t2_li = esforco_ponderado(erro,u,H,100)
                
                ISE_t2_li  = objfunc(erro,tempo,'ISE')
                ITSE_t2_li = objfunc(erro,tempo,'ITSE')
                ITAE_t2_li = objfunc(erro,tempo,'ITAE')
                IAE_t2_li  = objfunc(erro,tempo,'IAE')
                
                IG_t2_li = IG(H,1e4,1e9,1,u,ref,h)
                
                sy_t2_li= var(h)
                su_t2_li = var(u)
                
                fileName = ['Resluts for PID - FT2-FG ' , PIDtype, ' - ', FuzzyType ,' - ' , FT2Itype, ' - ',ref_type,' - ',simName];
                save( ['./results/',fileName])
                
                p_ft2(ts,h,ref,u,tempo,Kp,Kd,Ki,Am)
                
            elseif (FT2Itype == 'N')
                I_t2_nli = esforco_ponderado(erro,u,H,100)
                
                ISE_t2_nli  = objfunc(erro,tempo,'ISE')
                ITSE_t2_nli = objfunc(erro,tempo,'ITSE')
                ITAE_t2_nli = objfunc(erro,tempo,'ITAE')
                IAE_t2_nli  = objfunc(erro,tempo,'IAE')
                
                IG_t2_nli = IG(H,1e4,1e9,1,u,ref,h)
                
                sy_t2_nli= var(h)
                su_t2_nli = var(u)
              
                fileName = ['Resluts for PID - FT2-FG ' , PIDtype, ' - ', FuzzyType ,' - ' , FT2Itype, ' - ',ref_type,' - ',simName];
                save( ['./results/',fileName])
                
                p_ft2(ts,h,ref,u,tempo,Kp,Kd,Ki,Am)
                
            end;
             
        end
        if(flag_sinusoidal_dist) plot_ruido_senoide(ts); end;
        if( flag_noise) plot_ruido(ts,ruido); end;

        