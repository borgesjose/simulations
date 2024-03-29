%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID controllers for the Phase                      %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 30/04/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%Passo 1, definir o vetor tempo:
    Ts = 5; % periodo de amostragem para processo de um tanque ( Landau,2006)
    Tsim = 1500;
    nptos = Tsim/Ts;
    ts = linspace(0,Tsim,nptos);
    Tamostra = Ts;
%% Passo 2 - Defini��es:

%Dados do probelma:

h0 = 0.01; % ponto inicial

u = zeros(nptos,1); % variavel de entrada
h = zeros(nptos,1); % variavel de saida

Cv = 0.97 %velocity coefficient (water 0.97)
Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

Cd = Cc*Cv % discharge coefficient

r = 0.008;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

% definindo a referencia de controle 
for i=1:nptos,
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
% Agora � o momento de aplicar o rel� a planta: (rele com histerese)

    n = 30; % Numero de pontos de an�lise

    eps = 0.01;
    
    d = 12e-4;

    %nptos = 100;

% Chama a fun��o rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplica��o do rel�: 

    [yr,ur] = rele_h_nl(n,Ts,d,eps,A,Cd); 
%%
    figure;
    grid;
    plot(yr,'c-');
    hold on;
    figure;
    ddd = 1*10^3;
    plot(ur);
%% 4 Identificar os parametros a partir do experimento com rel�:
    
%    [gw,w,arm,Kp]=Identificar(n, d, eps,Ts,yr,ur);
tc1 = Ts;
y = yr;
u = ur;
%%
    %********** Calculo do Valor de Pico **************
arm=0;
for t=1:n,
   if y(t)>=arm arm=y(t); end;
end;
%%
%**************  Calcula Passagens pelo Zero *******************
    
kk=0;
for t=2:n, 
   if ((y(t-1)>0) & (y(t)<0))       
      f0=y(t-1);
      f1=y(t);
%     interpol(t,f0,f1,kk,x);
      kk=kk+1;
      m=f1-f0;
      b=-m*(t-1)+f0;
      x(kk)=-b/m;
   end;
   if ((y(t-1)<0) & (y(t)>0)) 
      f0=y(t-1);
      f1=y(t);
      kk=kk+1;
%     interpol(t,f0,f1,kk,x);
      m=f1-f0;
      b=-m*(t-1)+f0;
      x(kk)=-b/m;
   end;
end;

%%    calcula_periodo

p=0;
for i=1:kk-2, p=p+x(i+2)-x(i); end;
    p=((p/(kk-2))-1.5)*tc1;
    w=2*pi/p;
    gw=-(pi*sqrt(arm^2-eps1^2))/(4*d);
num = 0;
den = 0;
for j=(n/2):(n/2)+ceil(p),
    num = num + y(j);
    den = den + u(j);
end
Kp = num/den;



%%
    Ku = -1/gw;
    Tu = (2*pi)/w; 

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
    
%% Defini��es do controlador AT-PID-FG: 

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
%     z(i+1) = Level0; % store the level for plotting
     sps(i+1) = ref(i);
     
     erro(i)=ref(i)-h(i); %Erro
      
     
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
