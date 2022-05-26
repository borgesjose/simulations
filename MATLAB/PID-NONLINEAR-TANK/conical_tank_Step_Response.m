%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Simulation conical tank                            %
%                                                     %
%  -- Version: 1.0  - 02/05/2022                      %
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

r = 0.01;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

    R1 = 0.10; % Raio em metros
    R2 = 0.01; % Raio da base do cilindro
%%
% Calculando o input
vazao_max = .00038;
 for i=1:nptos, 
    if (i<=nptos/3)  u(i)=.000001; end;
    if (i>nptos/3 & i<=2*nptos/3 )   u(i) = 2*vazao_max; end;
    if (i>2*nptos/3)   u(i) = 2*vazao_max; end;
end ;

% Simulation with ode45;
for i=1:nptos
    
    [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd,R1,R2),[0,Ts],h0);
    h0 = y(end); % take the last point
    h(i+1) = h0; % store the height for plotting
    %if (y<=0) h(i+1)=0;end;
    %if (y>=.7) h(i+1)=.7; end;

end
%%
% plot results
figure;
plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
ylabel('Tank Height');
xlabel('Time (s)');
legend();
figure;
plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
ylabel('Sinal de entrada');
xlabel('Time (s)');
legend();
%%
L =  0.02;
T1 = 2.83
K = h(nptos)

% Kc = 1.2*((T1)/(K*L))
% Ti = 2*L
% Td = 0.5*L

Kc = 1.2*(u(nptos)-u(1)/h(nptos))*((T1)/(0.158))
Ti = 2*0.158
Td = 0.5*0.158

