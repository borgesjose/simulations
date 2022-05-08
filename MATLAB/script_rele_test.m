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
    Ts = .01; % periodo de amostragem para processo de um tanque ( Landau,2006)
    Tsim = 150
    nptos = Tsim/Ts;
    ts = linspace(0,Tsim,nptos+1);
%% Passo 2 - Defini��es:

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
% Agora � o momento de aplicar o rel� a planta: (rele com histerese)

    n = 20; % Numero de pontos de an�lise

    eps = 0.1; 
    d = 10e-4;

    nptos = 100;

% Chama a fun��o rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplica��o do rel�: 
    dmax = d + 5e-4;
    dmin = d - 3e-4;
    
    for i=1:n,
        y(i)=0;
        ref(i)= 0.8;
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
    %[yr,ur] = rele_h_nl(n,Ts,d,eps,A,Cd); 
%%
    figure;
    grid;
    plot(h,'c-');
    hold on;
    figure;
    plot(u);
%% 4 Identificar os parametros a partir do experimento com rel�:
    
    
    
    [gw,w,arm,Kp]=Identificar(n, d, eps,Ts,yr,ur);
%%
    Ku = -1/gw;
    Tu = (2*pi)/w; 

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);