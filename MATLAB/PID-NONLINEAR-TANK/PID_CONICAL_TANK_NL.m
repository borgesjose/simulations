%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui�                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID ziegler nichols aplied a conical tank          %
%                                                     %
%  -- Version: 1.0  - 21/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Passo 1, definir o vetor tempo:
            Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 500;
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

        r = 0.01;% raio do orificio de saida em metros

        A = pi*r^2;% Area do orificio de saida
        
        %% Passo 3 - Controller definition: 
        % Ctype definie o tipo de sintonia do controaldor: 
        % 'ZN' � Ziegle-Nichols , 
        % 'CC' � Choen Coon, 
        % 'AT' � Astrom .

        Ctype = 'ZN'; 
        
        Tamostra = Ts;
        
        % definindo a referencia de controle 
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= .05; end;
            if (i>nptos/4)   ref(i) = .05; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= .05; end;
            if (i>3*nptos/4)   ref(i) = .05; end;

        end ;

        %clear h;
        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;



        if Ctype == 'ZN':
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
        end;
        
        if Ctype == 'CC':
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
        end;
        
        if Ctype == 'AT':
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
            
            
        else:
            disp("Selecione um controlador: ZN , CC, AT ") 
            %SINTONIA PROFESSOR:
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;
        end;    
            
        

        %% Simulation with ode45;

        for i=4:nptos

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i); %Erro

            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            Ami = 1; 

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
                        if(u(i)>5e-3) u(i)=5e-3;end;

                        tempo(i)=i*Tamostra;

        end
        
        %%
        % plot results
        
        figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Resposta Tanque - R1: ', num2str(R1) , '  R2: ' , num2str(R2)])
       
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada');
        xlabel('Time (s)');
        legend();

        %savefig(['asdd',num2str(ii),num2str(jj)])
        saveas(gcf,['asdd',num2str(ii),num2str(jj),'.png'])
        %['Resposta-Tanque-R1:', num2str(R1) , 'R2:' , num2str(R2),'.fig']
        
        %%
             ISE_pid  = objfunc(erro,tempo,'ISE')
             ITSE_pid = objfunc(erro,tempo,'ITSE')
             ITAE_pid = objfunc(erro,tempo,'ITAE')
             IAE_pid  = objfunc(erro,tempo,'IAE')
       