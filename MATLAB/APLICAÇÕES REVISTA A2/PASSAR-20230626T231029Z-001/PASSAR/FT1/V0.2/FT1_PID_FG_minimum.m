%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piauí                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -José Borges do Carmo Neto-          %
% @author José Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Fuzzy PID controllers for the Phase                %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 01/05/2022                      %
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

    [gw,w,arm,Kp]=identificar(n, d, eps,Tc,yr,ur);

    Ku = -1/gw;
    Tu = (2*pi)/w;
    %Tu = (2*180)/w;

    L = 2;
   
    c = 1/Kp;
    b = sin(w*L)/(w*Ku);
    a = (c + cos(w*L))/(w^2);
    
%% 3.1 teste modelo:
%  a = 1/0.2133;
%  b = 0.6667/0.2133;
%  c = 0.1067/0.2133;
%% Definições do controlador AT-PID-FG: 

    Am = 1;

    Am_min = 1; 
    Am_max = 3;
    Theta_m_min = 45;
    Theta_m_max = 72;
    
    %Theta_m = (180/2)*(1-(1/Am));
H1 = 0.1;
H2 = 0.1;
H3 = 0.9;
H4 = 0.9;
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

patamar = 1
passo = 1
for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;

erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

rlevel = 1;
ruido = rlevel*rand(1,nptos);

for i=5:nptos,

P1(i) = p1+rlevel*rand; % Aplicando ruido na modelagem
P2(i) = p2+ruido(i);  % Aplicando ruido na modelagem
k = 2*P1(i)*P2(i); 
    
[c0,c1,c2,r0,r1,r2] = discretiza_zoh(P1(i),P2(i),k,Tc); %chama a função que discretiza o processo utilizano um ZOH;

     %if (i==550),r1 = - 1.84;r2 = 0.9109;  end % Ruptura no modelo
     
     y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4); % equação da diferença do processo
     
     erro(i)=ref(i)-y(i); %Erro
     r(i)=(erro(i)-erro(i-1));%Erro rate
 
     %    {***********MINIMUM FT-PID-FG*****************}
   
         
         mi1 = (r(i)/(2*L) + 1/2)*(erro(i)/(2*L) + 1/2); mi2 = (erro(i)/(2*L) + 1/2)*(-r(i)/(2*L) + 1/2); mi3 = (-erro(i)/(2*L) + 1/2)*(r(i)/(2*L) + 1/2); mi4 = (-erro(i)/(2*L) + 1/2)*(-r(i)/(2*L) + 1/2); 
            
         Am(i) = mi1*H1 + mi2*H2 + mi3*H3 + mi4*H4;
         
   


            Ami = Am(i)*Am_max + Am_min*(1 - Am(i)); 
            %Ami = 1;
      %Controlador:

%             alpha = (Kc)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
%             beta = -(Kc)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
%             gama = (Kc)*(Td)/Tamostra;

        Kci(i) = Kc/Ami;
        Kdi(i) = Kd/Ami;
        Kii(i) = Ki/Ami;

      % new version
            alpha = Kci(i)+ Kdi(i)/Tamostra + (Kii(i)*Tamostra)/2;
            beta = -Kci(i) - 2*(Kdi(i)/Tamostra)+(Kii(i)*Tamostra)/2;
            gama = Kdi(i)/Tamostra;


            u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);
      
       tempo(i)=i*Tamostra;
      fprintf('amostra:  %d \t entrada:  %6.3f \t saida:  %4.0f\n',i,u(i),y(i));
      
 end ;
 
 
     ISE_t1_m = objfunc(erro,tempo,'ISE')
     ITSE_t1_m = objfunc(erro,tempo,'ITSE')
     ITAE_t1_m = objfunc(erro,tempo,'ITAE')
     IAE_t1_m = objfunc(erro,tempo,'IAE')
     
%plotar seinal de saida e  de controle:    
figure;
grid;
plot(tempo,y,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['FT1- m -PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t1_m), ', ITSE:' ,num2str(ITSE_t1_m),', IAE:' ,num2str(IAE_t1_m), ', ITAE:' ,num2str(ITAE_t1_m)])
%%
% %plotar P1 e P2
% figure;
% grid;
% plot(tempo,P1,'g-');
% hold on;
% plot(tempo,P2);
%%
%plotar Kp,Kd,Ki
figure;
grid;
plot(tempo,Kci,'g-');
hold on;
plot(tempo,Kdi);
hold on;
plot(tempo,Kii);
title('FT1- m -PID-FG: Kp,Ki,Kd')
legend('Kc','Kd','Ki')
%%
figure;
grid;
plot(tempo,Am,'g-');
title('FT1- m -PID-FG: Am')