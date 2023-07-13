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
            Tsim = 1000;
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

 %% 2 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 200; % Numero de pontos de análise

    eps = 0.0; 
    d = .00023;

    %nptos = 1000;

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 

    [yr,ur] = rele_nh_nl(n, Ts, d, A,Cd); 
%%
    figure;
    grid;
    plot(yr,'c-');
    hold on;
    %%
    figure;
    grid;
    plot(ur);

%% 3 Identificar os parametros a partir do experimento com relé:

    %[gw,w,arm,Kp]=Identificar(n, d, eps,Ts,yr,ur);

    Ku = (4*d)/pi*0.0076;
  %%
    %Tu = (2*pi)/w;
    Tu = 11;
    w = (2*pi)/Tu;
%%

Kp = 2130;

L = .3;

%%   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
 
%% 3.1 teste modelo:
%  a = 1/0.2133;
%  b = 0.6667/0.2133;
%  c = 0.1067/0.2133;
%% Definições do controlador AT-PID-FG: 

    Am = 1;

    Am_min = 1; 
    Am_max = 5;
    Theta_m_min = 45;
    Theta_m_max = 72;
    
    %Theta_m = (180/2)*(1-(1/Am));

%% Sintonizanodo o PID:

    K = (pi/(2*Am*L))*[b;c;a];
    Kc = K(1);
    Ki = K(2);
    Kd = K(3);
    K
        %% Passo 3 - Controller definition: 
Ti=Kc/Kd;
Td=Kd/Kc;
%%
        Ctype = 'PR'%'ZN'; 
        patamar = 0.10
        passo = 0.00
        Tamostra = Ts;
    
        % definindo a referencia de controle 
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

        %Quebra no processo
        qp_value = 0.05;
        qp = zeros(1,nptos)
        tempo_qp = 5

        for i=1:nptos,
            if (i >= nptos/2 & i<=(nptos/2+tempo_qp))  qp(i) = qp(i) + qp_value; end;
        end ;
        %clear h;
        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;


        %% Simulation with ode45;

        for i=4:nptos
            
            %RUPTURA NO MODELO
%             % raio do orificio de saida em metros
%             if(i > (nptos/2)) r = 0.005; end;
%             A = pi*r^2;% Area do orificio de saida

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            
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
        
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['Sinal de Controle PID - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2),
        %'r_',num2str(r),'.png']) 
        
        
        %%
        
             H=nptos;
             ISE_pid_FG  = objfunc(erro,tempo,'ISE')
             ITSE_pid_FG = objfunc(erro,tempo,'ITS E')
             ITAE_pid_FG = objfunc(erro,tempo,'ITAE')
             IAE_pid_FG  = objfunc(erro,tempo,'IAE')
             
             I_pid_FG = esforco_ponderado(erro,u,H,100)
             IG_pid_FG = IG(H,1e4,1e9,1,u,ref,h)
             
             sy_pid_FG = var(h)
             su_pid_FG = var(u)
             