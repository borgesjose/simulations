%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui­                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID Ziegler–Nichols method aplied a conical tank          %
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
                 
        R1 = 12.5;
        R2 = 1;
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida

        Cv = 0.97 %velocity coefficient (water 0.97)
        Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        Cd = Cc*Cv % discharge coefficient

        r = 0.5;% raio do orificio de saida em centimetros

        A = pi*r^2;% Area do orificio de saida 
% para um resultado aproximado, multiplique o valor de volume / tempo por 3,785
 %% 2 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 200; % Numero de pontos de análise

    eps = 0.0; 
    d = 150;

    %nptos = 1000;

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 
    
    [y,u,refr] = rele_nh_nl_cm(n, Ts, d, A,Cd); 
%%
    figure;
    grid;
    hold on;
    plot(yr,'c-');
    plot(refr,'b-');

    %%
    figure;
    grid;
    plot(ur);
    
%%
% --- Calcula período
kont = 0;								
for t = 4:n,								
   if u(t) ~= u(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end
%%
Tamostra = Ts
Tu1 = (ch(7) - ch(6))*Tamostra;
Tu2 = (ch(8) - ch(7))*Tamostra;
Tu = Tu1 + Tu2 %Periodo critico;
omega = (2*pi)/(Tu)
aux1 = ch(5);aux2 = ch(7);
% --- Calcula valor de pico positivo
arm = eps;										
for t = aux1:aux2,
   if y(t) >= arm  arm = y(t); end;
end;
Au = arm;
% --- Calcula valor de pico negativo
arm = eps;										
for t = aux1:aux2,
   if y(t) <= arm  arm = y(t); end;
end;
Ad = arm;
a = (abs(Au) + abs(Ad))/2

%%
% --- Calcula ganho critico
Ku = (4*d)/(pi*sqrt(a^2 - eps^2));
% --- Constantes da planta

%% --- Gráfico da saída e entrada
figure(1)
rele = [u;y];
plot(tempo,rele);
%% --- Sintonia por Ziegler-Nichols
Kc = 0.6*Ku
Ti = 0.5*Tu
Td = Ti/4
%%
%********Sintonia de Controladores PID Método do Astron (Ziegler-Nichols
%                                                             Modificado)
P=0;
I=0;
D=0;
%*********  Período de amostragem  *******
% Ler do prompt - Tamostra=0.5;

%***********  Dados do Relé  ****************
% Ler o prompt
eps = 0;
%******************Identifica a FT do processo na frequência*******
gw=-(pi*sqrt(a^2-eps^2))/(4*d) %PROCESSO: valor da ft pro relé.

%gw = -0.2134+0.4488i;
rp=abs(gw); 
fip=atan(eps/sqrt(a^2-eps^2))
%fip=angle(gw);
%%
omega = w;

%*********Especificações*************
fim=5;
rs=0.15*rp;
fis=pi*fim/180;

%*************Cálculo dos Parâmetros do Controlador***********
Kc=rs*cos(fis-fip)/rp     
aux1=sin(fis-fip)/cos(fis-fip);
aux2=sqrt(1+aux1^2);
aux3=aux1+aux2;
Td=aux3/(2*omega);
Ti=4*Td;

P=Kc
I=Kc/Ti
D=Kc*Td




%    P=0.35
 %   I=0.04523
  %  D=0.81648