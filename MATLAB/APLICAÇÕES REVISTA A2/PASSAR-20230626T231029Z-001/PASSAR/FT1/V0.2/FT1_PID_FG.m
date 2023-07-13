%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Fuzzy PID controllers for the Phase                %
%  and Gain Margins of the System                     % 
%                                                     %
%  -- Version: 1.0  - 01/05/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 

%% 1 - Tratando o processo:

% Nesta etapa o processo � discretizado:
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
% Agora � o momento de aplicar o rel� a planta: (rele com histerese)

    n = 200; % Numero de pontos de an�lise

    eps = 0.2; 
    d = 0.5;

    nptos = 1000

% Chama a fun��o rele com histerese passando os paramentros do rele e os polos e ganho do proceso de 2 ordem
% Retorna o vetor yr, e ur com os resultados da aplica��o do rel�: 

    [yr,ur] = rele_h(n,Tc,d,eps,[p1,p2],k); 

%     figure;
%     grid;
%     plot(yr,'c-');
%     hold on;
%     plot(ur);

%% 3 Identificar os parametros a partir do experimento com rel�:

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
%% Defini��es do controlador AT-PID-FG: 

    Am = 2;

    Am_min = 1; 
    Am_max = 3;
    Theta_m_min = 45;
    Theta_m_max = 72;
    
    %Theta_m = (180/2)*(1-(1/Am));

%% Sintonizanodo o PID:

    K = (pi/(2*Am*L))*[b;c;a];
    Kc = K(1);
    Ki = K(2);
    Kd = K(3);
    K
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
    
[c0,c1,c2,r0,r1,r2] = discretiza_zoh(P1(i),P2(i),k,Tc); %chama a fun��o que discretiza o processo utilizano um ZOH;

     %if (i==550),r1 = - 1.84;r2 = 0.9109;  end % Ruptura no modelo
     
     y(i)= -r1*y(i-1)-r2*y(i-2)+c0*u(i-2)+c1*u(i-3)+c2*u(i-4); % equa��o da diferen�a do processo
     
     erro(i)=ref(i)-y(i); %Erro
     r(i)=(erro(i)-erro(i-1));%Erro rate
 
     %    {***********Regi�o IC-1*****************}
     if ( erro(i)>0  & r(i)>0 ) 
         
         mi5 = (-r(i)/L + 1)*(-erro(i)/L + 1); mi6 = (-erro(i)/L + 1)*(r(i)/L); mi8 = (erro(i)/L)*(-r(i)/L + 1); mi9 = (r(i)/L)*(erro(i)/L); 
            
         Am(i) = mi5*(1-exp(-mi5*4)) + mi6*(1-exp(-mi6*4)) + mi8*(exp(-mi8*4)) + mi9*(exp(-mi9*4));
         
     end;

%      {***********Regi�o IC-2*****************}
      if ( erro(i)<=0  & r(i)<=0) 
         
         mi1 = (-r(i)/L )*(-erro(i)/L); mi2 = (-erro(i)/L)*(r(i)/L + 1); mi4 = (-r(i)/L)*(erro(i)/L + 1); mi5 = (r(i)/L + 1)*(erro(i)/L + 1); 
            
         Am(i) = mi1*(exp(-mi1*4)) + mi2*(exp(-mi2*4)) + mi4*(1 - exp(-mi4*4)) + mi5*( 1 - exp(-mi5*4));  
      
      end;

 %     {***********Regi�o IC-3*****************}
      if ( erro(i)>0  & r(i)<=0) 
      
           mi5 = (-r(i)/L + 1)*(-erro(i)/L + 1); mi4 = (-erro(i)/L + 1)*(r(i)/L); mi8 = (erro(i)/L)*(-r(i)/L + 1); mi7 = (-r(i)/L)*(erro(i)/L); 
            
           Am(i) = mi5*(1-exp(-mi5*4))+mi4*(1-exp(-mi4*4))+mi8*(exp(-mi8*4))+mi7*(exp(-mi7*4));
      
      end;
      
 %      {***********Regi�o IC-4*****************}
      if ( erro(i)<=0  & r(i)>0)
          
          mi5 = (-r(i)/L + 1)*(erro(i)/L + 1); mi6 = (erro(i)/L + 1)*(r(i)/L); mi2 = (-erro(i)/L)*(-r(i)/L + 1); mi3 = (-r(i)/L)*(-erro(i)/L); 
            
          Am(i) = mi5*(1-exp(-mi5*4)) + mi6*(1-exp(-mi6*4)) + mi2*(exp(-mi2*4)) + mi3*(exp(-mi3*4));
      
      end
     
      
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
 
 
     ISE_t1 = objfunc(erro,tempo,'ISE')
     ITSE_t1 = objfunc(erro,tempo,'ITSE')
     ITAE_t1 = objfunc(erro,tempo,'ITAE')
     IAE_t1 = objfunc(erro,tempo,'IAE')
     
%plotar seinal de saida e  de controle:    
figure;
grid;
plot(tempo,y,'g-');
hold on;
plot(tempo,u);
plot(tempo,ref);
title(['FT1-PID-FG:',num2str(rlevel), ' ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)])
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
title('FT1-PID-FG: Kp,Ki,Kd')
legend('Kc','Kd','Ki')
%%
figure;
grid;
plot(tempo,Am,'g-');
title('FT1-PID-FG: Am')

%%
for i=5:nptos,
    
    
    Amii(i) = Am(i)*Am_max + Am_min*(1 - Am(i));
    
end   

figure;
grid;
plot(tempo,Amii,'g-');
title('FT1-PID-FG: Am')