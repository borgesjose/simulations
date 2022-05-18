%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui�                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  PID Control of conical tank                        %
%                                                     %
%  -- Version: 1.0  - 17/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
clc;

%Passo 1, definir o vetor tempo:
    Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
    Tsim = 1000
    nptos = Tsim/Ts;
    ts = linspace(0,Tsim,nptos);
    
%% Passo 2 - Definition:

%Dados do probelma:

h0 = 0.01; % ponto inicial

u = zeros(nptos+1,1); % variavel de entrada
h = zeros(nptos+1,1); % variavel de saida

Cv = 0.97 %velocity coefficient (water 0.97)
Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

Cd = Cc*Cv % discharge coefficient

r = 0.008;% raio do orificio de saida em metros

A = pi*r^2;% Area do orificio de saida

%%
% definindo a referencia de controle 


% for i=1:nptos,
%     if (i<=nptos/4)  ref(i)= .4; end;
%     if (i>nptos/4)   ref(i) = .3; end;
%     if (i>nptos/2 & i<=3*nptos/4)  ref(i)= .2; end;
%     if (i>3*nptos/4)   ref(i) = .1; end;
% end ;
for i=1:nptos,
    if (i<=nptos/4)  ref(i)= .4; end;
    if (i>nptos/4)   ref(i) = .5; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)= .6; end;
    if (i>3*nptos/4)   ref(i) = .65; end;
end ;

%  for i=1:nptos,ref(i)= .4;end ;

for i=1:nptos,
    if(i>nptos/2 & i< 5+nptos/2 ) 
        disturbio(i) = ref(i)*0.2e-3;
    else disturbio(i) = 0; end;
end ;

ref = ref+disturbio

clear h;
h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;


erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=2;% valor de limite das memberships functions

%%
Tc = Ts;
Tamostra = Tc;

Kc = 0.0001;
Ti = 1.1;
Td = 0.0794;

%% Controller definition: 
PID = 0
Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;

%%
%randpmo criado aleatoriamente por mim

%gene =[0.2146,0.3760,-0.1644,0.4906,0.0376,0.2273,0.2379,-0.0310,0.4428,0.5785,0.3263,0.3500];

%gene = [0.6585,0.3185,0.2140,-0.0218,0.7204,0.3362,0.0011,0.3658,-0.1151,0.0806,0.0513,0.2939]; ...
    
gene = [0.2377,0.0306,-0.2588,0.4572,0.5397,0.2005,0.0634,0.0350,0.4868,0.2303,0.1049,-0.0324,0.0481,0.3489,0.4641,0.2081];

%gene = thebest{1:1}(1:16);

Param =[gene,1,1];
rlevel = 0;
ruido = rlevel*rand(1,1000);


%% Simulation with ode45;
for i=4:nptos
    
    %u(i+1) = ref(i);   % store the Qin value
   
    [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd),[0,Ts],h(i-1));
    h0 = y(end); % take the last point
    h(i) = h0; % store the height for plotting
    
    erro(i)=ref(i) - h(i) + ruido(i); %Erro

    rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

         if (PID == 0)
             %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
             %Am(i) = Inferencia_T2(erro(i),rate(i),L,Param,'LI');
             %Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param,'LI');
             Am(i) =Inferencia_T2(erro(i),rate(i),L,Param,'LI');
             %[Am(i),yl(i),yr(i),l(i),r(i)]=EIASC(R(:,1)',R(:,2)',Y(:,1)',Y(:,2)'); 
             Ami = Am(i)*Am_max + Am_min*(1 - Am(i));
         else
             Ami = 1; end;%Am(i) = 1;

         %Controlador:
                
                Kp(i)= Kc/Ami;
                Kd(i)= (Td)*Kc/Ami;
                Ki(i)= (Kc/Ami)/(Ti);
                
                alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                gama = (Kc/Ami)*(Td)/Tamostra;

                u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
                
                %saturadores:
                if(u(i)<5e-5) u(i)=5e-5;end;
                if(u(i)>5e-3) u(i)=5e-3;end;
             
                tempo(i)=i*Tamostra;

end
%%
% plot results
figure;
plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
ylabel('Tank Height');
xlabel('Time (s)');
legend();
%% M�tricas de desempenho

     ISE_pid  = objfunc(erro,tempo,'ISE')
     ITSE_pid = objfunc(erro,tempo,'ITSE')
     ITAE_pid = objfunc(erro,tempo,'ITAE')
     IAE_pid  = objfunc(erro,tempo,'IAE')
  
%%
%plotar Kp,Kd,Ki
figure;
grid;
plot(tempo,Kc,'g-');
hold on;
plot(tempo,Kd);
hold on;
plot(tempo,Ki);
title('FT2-PID-FG: Kp,Ki,Kd')
legend('Kc','Kd','Ki')
