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
%  -- Version: 1.0  - 07/06/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Passo 1, definir o vetor tempo:
            Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 500;
            nptos = Tsim/Ts;
            ts = linspace(0,Tsim,nptos);
            
        %% Passo 2 - Dados do probelma:

        h0 = 0.001; % ponto inicial
                 
        R1 = 0.125;
        R2 = 0.01;
        
        u = zeros(nptos,1); % variavel de entrada
        h = zeros(nptos,1); % variavel de saida

        Cv = 0.97 %velocity coefficient (water 0.97)
        Cc = 0.97 %contraction coefficient (sharp edge aperture 0.62, well rounded aperture 0.97)

        Cd = Cc*Cv % discharge coefficient

        r = 0.005;% raio do orificio de saida em metros

        A = pi*r^2;% Area do orificio de saida

        Tc = Ts;

%% Aplicando o relé:

% Tc = 10;
% Tamostra = Tc;
% nptos = 800;
% Kc = 4.7561;
% Ti = 38.6378;
% Td = 9.6594;

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

        Ctype = 'ZN'%'ZN';
        patamar = 0.25
        passo = 0.0
        Tamostra = Ts;
    
        % definindo a referencia de controle 
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + 2*passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

%ref = ref+disturbio

        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

        if (Ctype == 'ZN')
            Tdead =  0.158;
            T1 = 2.83;
            Kc = 1.2*(3.8000e-04/0.2056)*((T1)/(Tdead))*10^-3
            Ti = 2*Tdead
            Td = 0.5*Tdead         
        end;
        
        if (Ctype == 'CC')
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
        end;
        
        if (Ctype == 'AT')
            Kc = .0001;
            Ti = 0.2;
            Td = 0.079;            
            
        end;
        
        if (Ctype == 'FG')
            
            
            K = AT_PID_FG(Am,L,a,b,c)
            
            Kc = K(1);
            Ti = Kc/K(2);
            Td = K(3)/Kc;
            
        end; 
        
        if(Ctype == 'PR')
            disp("Selecione um controlador: ZN , CC, AT ") 
            %SINTONIA PROFESSOR:
            Kc = .00005;
            Ti = 0.2;
            Td = 0.079;
        end;    
        

%% Passo 4 - FUZZY Controller definition:        
        Am_min = 2; 
        Am_max = 5;
        Theta_m_min = 45;
        Theta_m_max = 72;
        L = 2; %Provavelmente o valor de limite das memberships functions       
        
       % o vetor parametros dá os valores das MF's
       
       %param = [a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14];
       
       %param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];

%%
%randpmo criado aleatoriamente por mim

%gene =[0.2146,0.3760,-0.1644,0.4906,0.0376,0.2273,0.2379,-0.0310,0.4428,0.5785,0.3263,0.3500];

%gene = [0.6585,0.3185,0.2140,-0.0218,0.7204,0.3362,0.0011,0.3658,-0.1151,0.0806,0.0513,0.2939]; ...
    
gene = [0.2377,0.0306,-0.2588,0.4572,0.5397,0.2005,0.0634,0.0350,0.4868,0.2303,0.1049,-0.0324,0.0481,0.3489,0.4641,0.2081];

%gene = thebest{1:1}(1:12);

Param =[gene,1,1];
rlevel = 0.2;
ruido = rlevel*rand(1,1000);
Itype = 'L'
%%
    for i=4:nptos,

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting
          
            erro(i)=ref(i) - h(i); %Erro
            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

         if (PID == 0)
             %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
             %Am(i) = Inferencia_T2(erro(i),rate(i),L,Param,'LI');
             %Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param,'LI');
             Am(i) =Inferencia_T2(erro(i),rate(i),L,Param,Itype);
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
H=nptos;
     if (Itype == 'L')
     I_t2_li = esforco_ponderado(erro,u,H,100)
     ISE_t2_li  = objfunc(erro,tempo,'ISE')
     ITSE_t2_li = objfunc(erro,tempo,'ITSE')
     ITAE_t2_li = objfunc(erro,tempo,'ITAE')
     IAE_t2_li  = objfunc(erro,tempo,'IAE')
     elseif (Itype == 'N')
     I_t2_nli = esforco_ponderado(erro,u,H,100)
     ISE_t2_nli  = objfunc(erro,tempo,'ISE')
     ITSE_t2_nli = objfunc(erro,tempo,'ITSE')
     ITAE_t2_nli = objfunc(erro,tempo,'ITAE')
     IAE_t2_nli  = objfunc(erro,tempo,'IAE')
     end;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%PLOTAGEM
%PID
figure;
grid;
plot(tempo,Kp,'b-');
hold on;
plot(tempo,Ki,'r-');
plot(tempo,Kd,'m-');
legend('Kp','Ki','Kd')
%%
        figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Fuzzy tipo 2 - Resposta Tanque - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        
        %saveas(gcf,['resultado_R1=',num2str(r),'.png'])
        
        figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['Fuzzy tipo 2 - Sinal de Controle - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2), 'r_',num2str(r),'.png'])




%% Saida do processo:
% figure;
% grid;
% plot(tempo,y,'b-');
% hold on;
% plot(tempo,u,'r-');
% 
% %title(['T2,NLI: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
% %title('Resposta ao degrau com mudança referência')
% %legend('Type-1-CN-PID-FG','Type-2-CN-PID-FG')
% plot(tempo,ref,'r--');
% xlabel('tempo(s)');
% ylabel('y(t)');
% % text1= {'T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)}
% % text2= {'T2,IM-1: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)}
% % text([100],[0.5],text2)

 %% plotar Kp,Kd,Ki
%             figure;
%             grid;
%             plot(tempo,Kp,'g-');
%             hold on;
%             plot(tempo,Kd);
%             hold on;
%             plot(tempo,Ki);
%             title('FT1-PID-FG: Kp,Ki,Kd')
%             legend('Kc','Kd','Ki')
            %%
            figure;
            grid;
            plot(tempo,Am,'g-');
            title('FT2-PID-FG: Am')