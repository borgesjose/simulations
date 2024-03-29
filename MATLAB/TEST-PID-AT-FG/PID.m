%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID controller for the Phase                      %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 2.0  - 13/08/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PIDtype = 'FG'; %'ZN' = Ziegle-Nichols , 'CC' = Choen Coon,'AT' = Astrom, 'PR' = Teacher tunning;
%% Definir o vetor tempo:
Ts = 5; % periodo de amostragem para processo de um tanque ( Landau,2006)
Tsim = 1500;
nptos = Tsim/Ts;
ts = linspace(0,Tsim,nptos);
Tamostra = Ts;

%% Dados do problema:

h0 = 0.01; % ponto inicial

u = zeros(nptos,1); % variavel de entrada
h = zeros(nptos,1); % variavel de saida

Cv = 0.97; %velocity coefficient (water 0.97)
Cc = 0.97; %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

Cd = Cc*Cv; % discharge coefficient

r = 0.008;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

%% Relay Identification:
% Agora � o momento de aplicar o rel� a planta: (rele com histerese)
% Chama a fun��o rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplica��o do rel�:

n = 30; % Numero de pontos de an�lise
d0 = 2.4e-4; %3.8000e-04
d = 1.4e-4;
dmax = d0+d;
dmin = d0-d;
eps = 0.017;


[yr,ur] = histeresis_relay(n, Ts, d0, d, d, eps, A,Cd);
    
figure;
grid;
plot(yr,'c-');
hold on;
figure;
ddd = 1*10^3;
plot(ur);

%%
% --- Calcula per�odo
kont = 0;								
for t = 4:n,								
   if ur(t) ~= ur(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end

%%
Tu1 = (ch(7) - ch(6))*Tamostra;
Tu2 = (ch(8) - ch(7))*Tamostra;
Tu = Tu1 + Tu2 %Periodo critico;
omega = (2*pi)/(Tu)

%%
aux1 = ch(5);aux2 = ch(7);
i=0;
for t=aux1:aux2;
    i=i+1;
    yi(i) = yr(t);ui(i)=ur(t)*10^3;
    ti(i)=i*Tamostra;
end
%%
a1 = 0.5*([0 yi]+[yi 0]).*([ti 0]-[0 ti]);
a1 = sum(a1(1,2:length(yi)));
%%
a2 = 0.5*([0 ui]+[ui 0]).*([ti 0]-[0 ti]);
a2 = sum(a2(1,2:length(ui)));
%%
Kp = a1/a2;

%% --- Calcula valor de pico positivo
arm = 0.15;										
for t = aux1:aux2,
   if yr(t) >= arm  arm = yr(t); end;
end;
Au = arm;

%% --- Calcula valor de pico negativo
arm = 0.15;										
for t = aux1:aux2,
   if yr(t) <= arm  arm = yr(t); end;
end;
Ad = arm;
aa = (abs(Au) - abs(Ad));

%% --- Calcula ganho critico
Ku = (4*d)/(pi*sqrt(aa^2 - eps^2));
%% --- Calculo do atraso de transporte

teta = log(((d-d0)*Kp-eps)/((d-d0)*Kp+Ad))
x1=(d+d)*Kp*exp(teta)-(d-d0)*Kp+eps
x2=(d+d0)*Kp-eps

tau1=Tu1/log(x1/x2)
L=tau1*teta

%% PID-FG Tuning:
if(PIDtype == 'FG') 
    % Defini��es do controlador:
        Am = 5;

        Am_min = 2; 
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;

        Theta_m = (180/2)*(1-(1/Am));
    % Equacionando:
        w = (2*pi)/(Tu); 

        L = 0.158;

        c = 1/Kp;
        b = sin(w*L)/(w*Ku);
        a = (c + (cos(w*L)/Ku))/(w^2);
    
    % Sintonizanodo o controlador:

        K = (pi/(2*Am*L))*[b;c;a];
        Kc = K(1);
        Ki = K(2);
        Kd = K(3);
        
    
    Td = Kd/Kc;
    Ti = Kc/Ki;   
    
end;

%% ---  Ziegler-Nichols Tuning:
if(PIDtype == 'ZN')    
    Kc = 0.6*Ku
    Ti = 0.5*Tu
    Td = Ti/4
    

end;


%% Aplicando o controlador - OLD version
for i=1:nptos,
    if (i<=nptos/2)  ref(i)=0.15; end;
    if (i>nptos/2)   ref(i) = 0.2; end;
end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

rlevel = 0.0;
ruido = rlevel*rand(1,nptos);

for i=5:nptos,
    
    Ami = 1;
    [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Ts],h0);
    h0 = y(end); % take the last point
    h(i) = h0; % store the height for plotting

    erro(i)=ref(i)-h(i); %Erro
    
    
    % new version
%     alpha = Kc+ Kd/Tamostra + (Ki*Tamostra)/2;
%     beta = -(Kc) - 2*((Kd)/Tamostra)+(Ki*Tamostra)/2;
%     gama = (Kd)/Tamostra;
    
    %Controlador:
    
    %Kp(i)= Kc/Ami;
    %Kd(i)= (Td)*Kc/Ami;
    %Ki(i)= (Kc/Ami)/(Ti);
    
    alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
    beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
    gama = (Kc/Ami)*(Td)/Tamostra;
    
    u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
   
    %saturation:
    if(u(i)<5e-5) u(i)=5e-5;end;
    if(u(i)>2*3.8000e-04) u(i)=2*3.8000e-04;end;
    
    tempo(i)=i*Tamostra;
    fprintf('amostra:  %d \t entrada:  %6.6f \t saida:  %4.0f\n',i,u(i),h(i));
    
end ;
 
 
      ISE_t2 = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2 = objfunc(erro,tempo,'IAE')
     
%plotar seinal de saida e  de controle:    
figure;
grid;
plot(ts,h,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['AT-PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])

