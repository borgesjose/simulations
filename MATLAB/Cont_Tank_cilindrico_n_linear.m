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

%% 1 - Tratando o processo:

% Nesta etapa o processo é discretizado:
% Sendo:

    p1 = (1/2.5);
    p2 = (1/3.75);

    k = 2*p1*p2;
    
    Tc=0.2;
    Tamostra = Tc;
    
% Discretizamos o processo utilizando um segurador de ordem zero:

    s = tf('s');

    ft = k/((s+p1)*(s+p2))

    ftz = c2d(ft,Tc,'zoh')

%% 2 - Aplicando o rele:
% Agora é o momento de aplicar o relé a planta: (rele com histerese)

    n = 200; % Numero de pontos de análise

    eps = 0.2; 
    d = 0.5;

    nptos = 1000

% Chama a função rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplicação do relé: 

    [yr,ur] = rele_h(n,Tc,d,eps,[p1,p2],k); 

%     figure;
%     grid;
%     plot(yr,'c-');
%     hold on;
%     plot(ur);

%% 3 Identificar os parametros a partir do experimento com relé:

    [gw,w,arm,Kp]=Identificar(n, d, eps,Tc,yr,ur);

    Ku = -1/gw;
    Tu = (2*pi)/w; 

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
    
%% 3.1 teste modelo:
% 
% % step(ft,50)
% % hold on;
%  Gp = exp(-s*L)/(a*s^2 + b*s + c)
% % step(Gp,50)
% % 
% % ft
% 
% nyquist(ft)
% hold on
% nyquist(Gp)
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

P1(i) = p1+rlevel*rand; % Aplicando ruido na modelagem
P2(i) = p2+ruido(i);  % Aplicando ruido na modelagem
k = 2*P1(i)*P2(i); 
    
    %[c0,c1,c2,r0,r1,r2] = discretiza_zoh(P1(i),P2(i),k,Tc); %chama a função que discretiza o processo utilizano um ZOH;
    % if (i==550),r1 = - 1.84;r2 = 0.9109;  end % Ruptura no modelo
     
     [~,h] = ode45(@(t,y) tank_cilindrical(t,y,u(i)),[0 0.1],Level0);
     
     y(i) = h(end); % take the last point
     z(i+1) = Level0; % store the level for plotting
     sps(i+1) = ref(i);
     
     %-r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4); % equação da diferença do processo
     
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
