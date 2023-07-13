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

%% 3 Identificar os parametros a partir do experimento com relé:

Ku = 36.310242889675990;
Tu = 1.100000000000000;
w = 5.711986642890532;

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

 
    %% Definições do controlador AT-PID-FG: 

    Am_min = 1; 
    Am_max = 5;
    Theta_m_min = 45;
    Theta_m_max = 72;

%% MINIMUM

H1 = 0.1;
H2 = 0.1;
H3 = 0.9;
H4 = 0.9;


%% Aplicando o controlador - OLD version

        patamar = 5; % Valor da referencia de saida do processo
        passo = 10; % Valor do degrau a ser aplicado ao patamar inicial.
        Tamostra = Ts; % definição do tempo de amostragem
        
for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

rlevel = 1;
ruido = rlevel*rand(1,nptos);

for i=4:nptos,



            [~,y] = ode45(@(t,y) tank_conical_cm(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i);

            r(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro
     %    {***********MINIMUM FT-PID-FG*****************}
   
         
         mi1 = (r(i)/(2*L) + 1/2)*(erro(i)/(2*L) + 1/2); mi2 = (erro(i)/(2*L) + 1/2)*(-r(i)/(2*L) + 1/2); mi3 = (-erro(i)/(2*L) + 1/2)*(r(i)/(2*L) + 1/2); mi4 = (-erro(i)/(2*L) + 1/2)*(-r(i)/(2*L) + 1/2); 
            
         Am(i) = mi1*H1 + mi2*H2 + mi3*H3 + mi4*H4;
         
   


            Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
            Ami = 1;
      %Controlador:

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

        Kci(i) = Kc/Ami;
        Kdi(i) = Kd/Ami;
        Kii(i) = Ki/Ami;

            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
                        
            %saturadores:
            if(u(i)<5e-5) u(i)=5e-5;end;
            if(u(i)>2*750) u(i)=2*750;end;

       tempo(i)=i*Tamostra;
      %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 
 
     ISE_t1_m = objfunc(erro,tempo,'ISE')
     ITSE_t1_m = objfunc(erro,tempo,'ITSE')
     ITAE_t1_m = objfunc(erro,tempo,'ITAE')
     IAE_t1_m = objfunc(erro,tempo,'IAE')
     
      I_t1_m = esforco_ponderado(erro,u,H,100)
      IG_t1_m = IG(H,1e4,1e9,1,u,ref,h)
             
      sy_t1_m = var(h)
      su_t1_m = var(u)
     
%% plotar seinal de saida e  de controle:    

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

%%
figure;
grid;
plot(ts,h,'g-');
hold on;
plot(ts,u);
plot(ts,ref);
title(['FT1- m -PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t1_m), ', ITSE:' ,num2str(ITSE_t1_m),', IAE:' ,num2str(IAE_t1_m), ', ITAE:' ,num2str(ITAE_t1_m)])
%%
% %plotar P1 e P2
% figure;
% grid;
% plot(tempo,P1,'g-');
% hold on;
% plot(tempo,P2);
%%
%plotar Kp,Kd,Ki
figure;
grid;
plot(tempo,Kci,'g-');
hold on;
plot(tempo,Kdi);
hold on;
plot(tempo,Kii);
title('FT1- m -PID-FG: Kp,Ki,Kd')
legend('Kc','Kd','Ki')
%%
figure;
grid;
plot(tempo,Am,'g-');
title('FT1- m -PID-FG: Am')