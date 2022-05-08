%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID controllers for the Phase                      %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 30/04/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%Passo 1, definir o vetor tempo:
    Ts = .1; % periodo de amostragem para processo de um tanque ( Landau,2006)
    Tsim = 150
    nptos = Tsim/Ts;
    ts = linspace(0,Tsim,nptos+1);
%% Passo 2 - Definições:

%Dados do probelma:

h0 = 0.01; % ponto inicial

u = zeros(nptos+1,1); % variavel de entrada
h = zeros(nptos+1,1); % variavel de saida

Cv = 0.97 %velocity coefficient (water 0.97)
Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

Cd = Cc*Cv % discharge coefficient

r = 0.008;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

% definindo a referencia de controle 
for i=1:nptos+1,
    if (i<=nptos/5)  ref(i)=.5; end;
    if (i>nptos/5)   ref(i) = .5; end;
end ;

% Calculando o input
 for i=1:nptos, 
    if (i<=nptos/3)  u(i)=.0010; end;
    if (i>nptos/3 & i<=2*nptos/3 )   u(i) = .0012; end;
    if (i>2*nptos/3)   u(i) = .0001; end;
    %u(i)=.0009;
end ;

%% 3 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 2000; % Numero de pontos de análise

    eps = 0.5; 
    d = 10e-4;

    nptos = 1000;

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 

    [yr,ur] = rele_h_nl(n,Ts,d,eps,A,Cd); 
%%
    figure;
    grid;
    plot(yr,'c-');
    hold on;
    figure;
    plot(ur);
%% 4 Identificar os parametros a partir do experimento com relé:
    
    
    
    [gw,w,arm,Kp]=Identificar(n, d, eps,Ts,yr,ur);
%%
    Ku = -1/gw;
    Tu = (2*pi)/w; 

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
    
%% Definições do controlador AT-PID-FG: 

    Am = 3;

    Am_min = 2; 
    Am_max = 5;
    Theta_m_min = 45;
    Theta_m_max = 72;
    
    Theta_m = (180/2)*(1-(1/Am));

%% Sintonizanodo o PID:

    K = (pi/(2*Am*L))*[b;c;a];
    Kc = K(1);
    Ki = K(2);
    Kd = K(3);
    
%% Aplicando o controlador - OLD version
for i=1:nptos,
    if (i<=nptos/2)  ref(i)=1; end;
    if (i>nptos/2)   ref(i) = 2; end;
end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

rlevel = 0.0;
ruido = rlevel*rand(1,nptos);

for i=5:nptos,
    

     [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Ts],h0);
     h0 = y(end); % take the last point
     h(i+1) = h0; % store the height for plotting
     
     y(i) = h(end); % take the last point
     z(i+1) = Level0; % store the level for plotting
     sps(i+1) = ref(i);
     
     erro(i)=ref(i)-y(i); %Erro
      
     
      % new version
            alpha = Kc+ Kd/Tamostra + (Ki*Tamostra)/2;
            beta = -(Kc) - 2*((Kd)/Tamostra)+(Ki*Tamostra)/2;
            gama = (Kd)/Tamostra;


            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
      
       tempo(i)=i*Tamostra;
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 
 
      ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
     
%plotar seinal de saida e  de controle:    
figure;
grid;
plot(tempo,y,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['AT-PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
%%
%plotar P1 e P2
figure;
grid;
plot(tempo,P1,'g-');
hold on;
plot(tempo,P2);
%%
save('C://Users/joseb\OneDrive/MESTRADO/Pesquisa - GRADUATE/MESTRADO/2 - PESQUISA/4 - RESULTADOS/SIMULATION/MATLAB/AT PID FG/simples/simples.mat')
