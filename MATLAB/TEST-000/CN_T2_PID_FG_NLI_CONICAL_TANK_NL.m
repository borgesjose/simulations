%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  -- Level control of a conical tank using Interval  %
%  Type-2 Fuzzy Logic PID controllers for the Phase   %
%  and Gain Margins of the System                     % 
%  nas margens de fase e de ganho                     %
%  -- Version: 1.0  - 20/02/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%
% O processo: 
% Hz_planta =
%  
%     b1 z^-1 + b2 z^-2
%   --------------------
%       1 - a1 z^-1 

%% Passo 1, definir o vetor tempo:
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

%%
Tc = 10;

b1 = 0.0396;
b2 = 0.0090;

a1 = -0.8226;
a2 = 0; 

z = tf('z', Tc )

ftz = (b1*(z^-1)+ b2*(z^-2))/(1 + a1*(z^-1)+ a2*(z^-2));

% figure;
% step(ftz,50)

%% Aplicando o relé:
Tc = 10;
Tamostra = Tc;
nptos = 800;
Kc = 4.7561;
Ti = 38.6378;
Td = 9.6594;

% d = 80;
% eps = 1;
% %[Kc,Ti,Td] = Ident_com_rele(Tamostra,nptos,d,eps);
% [Kc,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps);

%% Definições do Controlador: 
PID = 0
Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;
Theta_m_max = 72;
     
%Aplicando o CN-PID-FG:

 %turnpoint = 500;%floor(rand*nptos);


for i=1:nptos,
    if (i<=nptos/4)  ref(i)=10; end;
    if (i>nptos/4)   ref(i) = 50; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=25; end;
    if (i>3*nptos/4)   ref(i) = 60; end;
end ;
% for i=1:nptos,
%     ref(i)=30;
% end ;

for i=1:nptos,
    if(i>nptos/2 & i< 5+nptos/2 ) 
        disturbio(i) = ref(i)*0.2;
    else disturbio(i) = 0; end;
end ;

ref = ref+disturbio

clear y;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;


erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=4;%Provavelmente o valor de limite das memberships functions

%%
%randpmo criado aleatoriamente por mim

%gene =[0.2146,0.3760,-0.1644,0.4906,0.0376,0.2273,0.2379,-0.0310,0.4428,0.5785,0.3263,0.3500];

%gene = [0.6585,0.3185,0.2140,-0.0218,0.7204,0.3362,0.0011,0.3658,-0.1151,0.0806,0.0513,0.2939]; ...
    
%gene = [0.2377,0.0306,-0.2588,0.4572,0.5397,0.2005,0.0634,0.0350,0.4868,0.2303,0.1049,-0.0324,0.0481,0.3489,0.4641,0.2081];

gene = thebest{1:1}(1:16);

Param =[gene,1,1];
rlevel = 0;
ruido = rlevel*rand(1,1000);

    for i=5:nptos,

         %y(i) =-a1*y(i-1)-a2*y(i-2)+b1*u(i-1)+b2*u(i-2);
         [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Ts],h(i));
         h0 = y(end); % take the last point
         h(i+1) = h0; % store the height for plotting
          
         erro(i)=ref(i)- h(i)+ruido(i); %Erro

         rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro

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


           tempo(i)=i*Tamostra;
            
          %fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));

     end ;

     if (PID == 0)
     ISE_t2  = objfunc(erro,tempo,'ISE')
     ITSE_t2 = objfunc(erro,tempo,'ITSE')
     ITAE_t2 = objfunc(erro,tempo,'ITAE')
     IAE_t2  = objfunc(erro,tempo,'IAE')
     else
     ISE_pid  = objfunc(erro,tempo,'ISE')
     ITSE_pid = objfunc(erro,tempo,'ITSE')
     ITAE_pid = objfunc(erro,tempo,'ITAE')
     IAE_pid  = objfunc(erro,tempo,'IAE')
     end;
