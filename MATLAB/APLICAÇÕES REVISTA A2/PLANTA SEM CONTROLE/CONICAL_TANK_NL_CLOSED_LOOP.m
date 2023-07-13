%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui­                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID ziegler nichols aplied a conical tank          %
%                                                     %
%  -- Version: 1.0  - 21/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Passo 1, definir o vetor tempo:
            Ts = .1; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 500;
            nptos = Tsim/Ts;
            ts = linspace(0,Tsim,nptos);

        %% Passo 2 - Dados do probelma:

        h0 = 0.001; % ponto inicial
                 
        R1 = 0.125;
        R2 = 0.01;
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida

        Cv = 0.97 %velocity coefficient (water 0.97)
        Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        Cd = Cc*Cv % discharge coefficient

        r = 0.005;% raio do orificio de saida em metros

        A = pi*r^2;% Area do orificio de saida
        
        %% Passo 3 - Controller definition: 
        % Ctype definie o tipo de sintonia do controaldor: 
        % 'ZN' é Ziegle-Nichols , 
        % 'CC' é Choen Coon, 
        % 'AT' é Astrom 
        % 'PR' é a sintonia do professor

        Ctype = 'PR'%'ZN'; 
        patamar = 0.15
        passo = 0.00
        Tamostra = Ts;
    
        % definindo a referencia de controle 
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;


        %% Simulation with ode45;

        for i=4:nptos
            
            %RUPTURA NO MODELO
            % raio do orificio de saida em metros
            if(i > (nptos/2)) r = 0.005; end;
            A = pi*r^2;% Area do orificio de saida

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i);
                        u(i)= erro(i);%u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2) ;%+ disturbio(i);

                        %saturadores:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>2*3.8000e-04) u(i)=2*3.8000e-04;end;

                        tempo(i)=i*Tamostra;

        end
        
        %%
        % plot results
        
        figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Resposta Tanque PID - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        
        %saveas(gcf,['resultado_R1=',num2str(r),'.png'])
 %%       
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['REALIMENTAÇÃO: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2),
        %'r_',num2str(r),'.png']) 
        
        
        %%
        
%              H=nptos;
%              ISE_pid  = objfunc(erro,tempo,'ISE')
%              ITSE_pid = objfunc(erro,tempo,'ITSE')
%              ITAE_pid = objfunc(erro,tempo,'ITAE')
%              IAE_pid  = objfunc(erro,tempo,'IAE')
%              
%              I_pid = esforco_ponderado(erro,u,H,100)
%              IG_pid = IG(H,1e4,1e9,1,u,ref,h)
%              
%              sy_pid= var(h)
%              su_pid = var(u)
             