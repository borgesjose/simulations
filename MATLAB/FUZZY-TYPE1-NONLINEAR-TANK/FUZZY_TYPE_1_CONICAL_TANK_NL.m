%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui­                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID FUZZY TIPO 1 aplied a conical tank             %
%                                                     %
%  -- Version: 1.0  - 21/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Passo 1, definir o vetor tempo:
            Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 2000;
            nptos = Tsim/Ts;
            ts = linspace(0,Tsim,nptos);
            
        %% Passo 2 - Dados do probelma:

        h0 = 0.001; % ponto inicial
                 
        R1 = 0.20;
        R2 = 0.01;
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida

        Cv = 0.97 %velocity coefficient (water 0.97)
        Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        Cd = Cc*Cv % discharge coefficient

        r = 0.007;% raio do orificio de saida em metros

        A = pi*r^2;% Area do orificio de saida
        
        %% Passo 3 - PID Controller definition: 
        % Ctype definie o tipo de sintonia do controaldor: 
        % 'ZN' é Ziegle-Nichols , 
        % 'CC' é Choen Coon, 
        % 'AT' é Astrom,
        % 'FG' é AT-PID-FG,
        % 'PR' é a sintomnia do professor

        Ctype = 'ZN'%'ZN'; 
        patamar = 0.05
        passo = 0.1
        Tamostra = Ts;
    
        % definindo a referencia de controle 
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + 2*passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

        %clear h;
        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;


        if (Ctype == 'ZN')
            Tdead =  0.158;
            T1 = 2.83;
            Kc = 1.2*(3.8000e-04/0.2056)*((T1)/(Tdead))*10^-3
            Ti = 2*Tdead
            Td = 0.5*Tdead         
        end;
        
        if (Ctype == 'CC')
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
        end;
        
        if (Ctype == 'AT')
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
            
        end;
        
        if (Ctype == 'FG')
            
            
            K = AT_PID_FG(Am,L,a,b,c)
            
            Kc = K(1);
            Ti = Kc/K(2);
            Td = K(3)/Kc;
            
        end; 
        
        if(Ctype == 'PR')
            disp("Selecione um controlador: ZN , CC, AT ") 
            %SINTONIA PROFESSOR:
            Kc = .00005;
            Ti = 0.2;
            Td = 0.079;
        end;    
        
%% Passo 4 - FUZZY Controller definition:        
        Am_min = 2; 
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;
        L = 2;        
        
       % o vetor parametros dá os valores das MF's
       
       %param = [a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14];
       
       param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];
       
        %% Simulation with ode45;

        for i=4:nptos

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i); %Erro
            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            Am(i) = FT1_pid_ag(erro(i),rate(i),L,param);
            %Am(i) = Inferencia_T1(erro(i),rate(i),L);
            
            Ami = Am(i)*Am_max + Am_min*(1 - Am(i));
            
            
                        %Controlador:

                        Kp(i)= Kc/Ami;
                        Kd(i)= (Td)*Kc/Ami;
                        Ki(i)= (Kc/Ami)/(Ti);

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

                        u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);

                        %saturadores:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>3.8000e-04) u(i)=3.8000e-04;end;

                        tempo(i)=i*Tamostra;

        end
        
        %%
        % plot results
        
        figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Fuzzy tipo 1 - Resposta Tanque - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        
        %saveas(gcf,['resultado_R1=',num2str(r),'.png'])
        
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['Fuzzy tipo 1 - Sinal de Controle - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2), 'r_',num2str(r),'.png'])
        
        
            %%
             ISE_ft1  = objfunc(erro,tempo,'ISE')
             ITSE_ft1 = objfunc(erro,tempo,'ITSE')
             ITAE_ft1 = objfunc(erro,tempo,'ITAE')
             IAE_ft1  = objfunc(erro,tempo,'IAE')
       
            %% plotar Kp,Kd,Ki
            figure;
            grid;
            plot(tempo,Kp,'g-');
            hold on;
            plot(tempo,Kd);
            hold on;
            plot(tempo,Ki);
            title('FT1-PID-FG: Kp,Ki,Kd')
            legend('Kc','Kd','Ki')
            %%
            figure;
            grid;
            plot(tempo,Am,'g-');
            title('FT1-PID-FG: Am')