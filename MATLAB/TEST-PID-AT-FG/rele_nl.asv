%	Modelagem da função de transferência: Relé com histerese
% --- Função de tranferência contínua
%                  1
%          ------------------
%          (s + 1)(5.17s + 1)
%
% --- Função de transferência discreta
%          0.0199 z + 0.0163
%       -------------------------
%       z^2 - 1.5143 z^1 + 0.5506
% --- Período de amostragem: 0.5
%

%Dados do probelma:

h0 = 0.01; % ponto inicial

u = zeros(nptos,1); % variavel de entrada
h = zeros(nptos,1); % variavel de saida

h(1)=0.01 ; h(2)=0.01 ; h(3)=0.01 ; h(4)=0.01; h(5)=0.01;
u(1)=0.00001 ; u(2)=0.00001 ; u(3)=0.00001 ; u(4)=0.00001; u(5)=dmax;

Cv = 0.97 %velocity coefficient (water 0.97)
Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

Cd = Cc*Cv % discharge coefficient

r = 0.008;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

Ts = 5; % periodo de amostragem para processo de um tanque ( Landau,2006)
Tsim = 1500
Tc = Ts;
Tamostra = Tc;
    n = 30; % Numero de pontos de análise

    eps = 0.01;
    
    %d = ;
%Tamostra = 240
nptos = 30;
d0 = 12e-4;
d = 9e-4;
dmax = d0+d;
dmin = d0 - d;
%eps = 1;

for t=1:2,
   u(t) = dmin;e(t) = 0;y(t) = 0;tempo(t) = t*Tamostra;
end;

% --- Experimentação com o relé
for t = 3:nptos,								
   %y(t) = 1.5143*y(t-1) - 0.5506*y(t-2) + 0.0199*u(t-1)+ 0.0163*u(t-2);
   %y(t) = 0.8825*y(t-1) + 0.235*u(t-1);
   %y(t)= r1*y(t-1)+c0*u(t-1);
   
   [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Tc],h(i));
   h0 = y(end); % take the last point
   h(i+1) = h0; % store the height for plotting
   
   e(t) = ref-y(t);
   if ((abs(e(t)) >= eps) & (e(t)  >0))      u(t) =  dmax; end;
   if ((abs(e(t)) > eps) & (e(t) < 0))      u(t) = dmin; end;
   if ((abs(e(t)) < eps) & (u(t-1) == dmax))   u(t) =  dmax; end;
   if ((abs(e(t)) < eps) & (u(t-1) == dmin))  u(t) = dmin; end;
   tempo(t) = t*Tamostra;
end;
%%
% --- Calcula período
kont = 0;								
for t = 4:nptos,								
   if u(t) ~= u(t-1)
      kont = kont + 1;
      ch(kont) = t;
   end
end
%%
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







