%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui­                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID AT-FG aplied a conical tank                  %
%                                                     %
%  -- Version: 1.0  - 21/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Passo 1, definir o vetor tempo:
            Ts = .1; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 1000;
            nptos = Tsim/Ts;
            ts = linspace(0,Tsim,nptos);

        %% Passo 2 - Dados do probelma:

        h0 = 0.1; % ponto inicial
                 
        R1 = 12.5; % Raio superior
        R2 = 1; %Raio inferior
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida

        Cv = 0.97 %velocity coefficient (water 0.97)
        Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        Cd = Cc*Cv % discharge coefficient

        r = 0.5;% raio do orificio de saida em centimetros

        A = pi*r^2;% Area do orificio de saida 
% para um resultado aproximado, multiplique o valor de volume / tempo por 3,785
%% 2 - Aplicando o relé a malha de controle:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 200; % Numero de pontos de análise

    eps = 0.0; 
    d = 150;

    %nptos = 1000;

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 
  
    [yr,ur,refr] = rele_nh_nl_cm(n, Ts, d, A,Cd); %Aplica o rele não linear com funções em cm.

    % Plotando o resultado da aplicação do relé: 
    figure;
    grid;
    hold on;
    plot(yr,'c-');
    plot(refr,'b-');
    figure;
    grid;
    plot(ur);

%% 3 Identificar os parametros a partir do experimento com relé:

% --- Calcula período
kont = 0;								
for t = 4:n,								
   if ur(t) ~= ur(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end
%%
Tamostra = Ts;
Tu1 = (ch(7) - ch(6))*Tamostra;
Tu2 = (ch(8) - ch(7))*Tamostra;
Tu = Tu1 + Tu2 %Periodo critico;
omega = (2*pi)/(Tu) % frequencia critica
aux1 = ch(5);aux2 = ch(7);
% --- Calcula valor de pico positivo
arm = eps;										
for t = aux1:aux2,
   if yr(t) >= arm  arm = yr(t); end;
end;
Au = arm;
% --- Calcula valor de pico negativo
arm = eps;										
for t = aux1:aux2,
   if yr(t) <= arm  arm = yr(t); end;
end;
Ad = arm;
a = (abs(Au) + abs(Ad))/2 % Amplitude sinal de saida planta

w=omega;

% --- Calcula ganho critico
Ku = (4*d)/(pi*sqrt(a^2 - eps^2));
% --- Constantes da planta

%%
% Ku = 36.310242889675990;
% Tu = 1.100000000000000;
% w = 5.711986642890532;
%% 
% Informações obtidas pela resposta ao degrau:
% Ganho estático:
Kp = 21/150;
% Atraso de transporte:
L = .3;

%% 4 Sintonia do controlador PID-AT-FG  

%Calculo de a,b e c
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);

%Definições das margens de ganho e de fase:
    Am = 1;
    Theta_m = (180/2)*(1-(1/Am));

% Sintonia:
    K = (pi/(2*Am*L))*[b;c;a];
    Kc = K(1);
    Ki = K(2);
    Kd = K(3);
    K
    Ti=Kc/Kd;
    Td=Kd/Kc;


%% 5 Definições da simulação

        patamar = 5; % Valor da referencia de saida do processo
        passo = 10; % Valor do degrau a ser aplicado ao patamar inicial.
        Tamostra = Ts; % definição do tempo de amostragem
    
        % definindo a referencia de controle 
            for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

        %Inicialização das variaveis 
        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; % sinal de saida
        u(1)=1 ; u(2)=1 ; u(3)=1; u(4)=1; % sinal de controle
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1; %valor do erro


%% 6 Simulation with ode45;

        for i=4:nptos
            
            %RUPTURA NO MODELO
%             % raio do orificio de saida em metros
%             if(i > (nptos/2)) r = 0.005; end;
%             A = pi*r^2;% Area do orificio de saida

            [~,y] = ode45(@(t,y) tank_conical_cm(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i);

            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            Ami = 1; 

                        %Controlador:

                        %Kp(i)= Kc/Ami;
                        %Kd(i)= (Td)*Kc/Ami;
                       % Ki(i)= (Kc/Ami)/(Ti);

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

                        u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2) ;%+ disturbio(i);

                        %saturadores:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>2*750) u(i)=2*750;end;

                        tempo(i)=i*Tamostra;

        end
        
%% 7 Plotar os Resultados:

        figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Resposta Tanque PID - Am: ', num2str(Am) , '  Kc: ' , num2str(Kc), '  Ki: ' , num2str(Ki), '  Kd: ' , num2str(Kd), ' Ti: ' , num2str(Ti), '  Td: ' , num2str(Td)])
        
        %saveas(gcf,['resultado_R1=',num2str(r),'.png'])
        
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['Sinal de Controle PID - Am: ', num2str(Am) , '  Kc: ' , num2str(Kc), '  Ki: ' , num2str(Ki), '  Kd: ' , num2str(Kd), ' Ti: ' , num2str(Ti), '  Td: ' , num2str(Td)])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2),
        %'r_',num2str(r),'.png']) 
        
        
%% 8 Calculo das métricas de desempenho:
        
             H=nptos;
             ISE_pid_FG  = objfunc(erro,tempo,'ISE')
             ITSE_pid_FG = objfunc(erro,tempo,'ITSE')
             ITAE_pid_FG = objfunc(erro,tempo,'ITAE')
             IAE_pid_FG  = objfunc(erro,tempo,'IAE')
             
             I_pid_FG = esforco_ponderado(erro,u,H,100)
             IG_pid_FG = IG(H,1e4,1e9,1,u,ref,h)
             
             sy_pid_FG = var(h)
             su_pid_FG = var(u)
             