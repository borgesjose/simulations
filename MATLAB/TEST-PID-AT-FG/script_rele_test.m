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
    Ts = 10; % periodo de amostragem para processo de um tanque ( Landau,2006)
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


%% 3 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 20; % Numero de pontos de análise

    eps = 0.2; 
    d = 10e-4;

    nptos = 100;

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 
    dmax = d + 9e-4;
    dmin = d - 9e-4;
    
    for i=1:n,
        y(i)=0;
        ref(i)= 1.5;
    end;

    e(1)=0; e(2)=0; 
    h(1)=0.01 ; h(2)=0.01 ; h(3)=0.01 ; h(4)=0.01; h(5)=0.01;

    u(1)=0.00001 ; u(2)=0.00001 ; u(3)=0.00001 ; u(4)=0.00001; u(5)=dmax;

    for i=5:n,
        
        [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Ts],h(i));
        h0 = y(end); % take the last point
        h(i+1) = h0; % store the height for plotting

       %y(t)= -theta*y(t-1) - phi*y(t-2) + alpha*u(t-2) + beta*u(t-3) + gama*u(t-4);
       e(i)= ref(i)- h(i);

       if ((abs(e(i))>eps) & (e(i)>0))  u(i+1)=dmax; end;
       if ((abs(e(i))>eps) & (e(i)<0))  u(i+1)=dmin; end;
       if ((abs(e(i))<eps) & (u(i)==dmax))  u(i+1)=dmax; end;
       if ((abs(e(i))<eps) & (u(i)==dmin))  u(i+1)=dmin; end;
       if (e(i)==eps)  u(i+1)=dmax; end;
       if (e(i)==-eps)  u(i+1)=dmin; end;

    end;

    figure;
    grid;
    plot(h,'c-');
    hold on;
    plot(ref(1:n));
    figure;
    plot(u);
    
%% 4 Identificar os parametros a partir do experimento com relé:
    %[gw,w,arm,Kp]=Identificar(n, d, eps,Ts,h,u);
%********** Calculo do Valor de Pico **************
arm=0;
for t=1:n,
   if y(t)>=arm arm = h(t); end;
end;

%% **************  Calcula Passagens pelo Zero *******************
    
kk=0;
for t=2:n, 
   if ((h(t-1)>0) & (h(t)<0))       
      f0=h(t-1);
      f1=h(t);
%     interpol(t,f0,f1,kk,x);
      kk=kk+1;
      m=f1-f0;
      b=-m*(t-1)+f0;
      x(kk)=-b/m;
   end;
   if ((h(t-1)<0) & (h(t)>0)) 
      f0=h(t-1);
      f1=h(t);
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

%%
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