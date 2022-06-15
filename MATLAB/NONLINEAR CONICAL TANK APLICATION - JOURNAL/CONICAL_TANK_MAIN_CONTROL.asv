%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piaui­                      %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jose Borges do Carmo Neto-          %
% @author Jose Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  Conical tank control using PID, Fuzzy              %
%  tipo 1 e Fuzzy tipo 2                              %
%                                                     %
%  -- Version: 1.0  - 14/06/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Script que serve como main da aplicação.

        %% Passo 1, definir o vetor tempo:

        Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
        Tsim = 500;
        nptos = Tsim/Ts;
        ts = linspace(0,Tsim,nptos);

        %% Passo 2 - Dados do problema:

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
        



        %% Passo 3 - Controller definition: 
        % Ctype definie o tipo de sintonia do controaldor: 
        % 'ZN' é Ziegle-Nichols , 
        % 'CC' é Choen Coon, 
        % 'AT' é Astrom 
        % 'PR' é a sintomnia do professor

        Ctype = 'ZN'; 
        
        [Kc,Ti,Td] = PID(Ctype); % Seleciona o PID 
        
        %% definindo a referencia de controle 
        
        patamar = 0.05
        passo = 0.00
        Tamostra = Ts;
    
        
        
        for i=1:nptos,

            if (i<=nptos/4)  ref(i)= patamar; end;
            if (i>nptos/4)   ref(i) = patamar + passo ; end;
            if (i>nptos/2 & i<=3*nptos/4)  ref(i)= patamar + passo; end;
            if (i>3*nptos/4)   ref(i) = patamar + 2*passo; end;

        end ;

        %% Quebra no processo
        qp_value = 0.05;
        qp = zeros(1,nptos)
        tempo_qp = 5

        for i=1:nptos,
            if (i >= nptos/2 & i<=(nptos/2+tempo_qp))  qp(i) = qp(i) + qp_value; end;
        end ;
        %clear h;
        h(4)=h0 ; h(3)=h0 ; h(2)=h0 ; h(1)=h0 ; 
        u(1)=1e-5 ; u(2)=1e-5 ; u(3)=1e-5; u(4)=1e-5;
        erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;



%%

load('ruido.mat')
load('disturbio.mat')


        %% Simulation with ode45;

        for i=4:nptos
            
            %RUPTURA NO MODELO
            % raio do orificio de saida em metros
            if(i > (nptos/2)) r = 0.006; end;
            A = pi*r^2;% Area do orificio de saida

            [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i-1),Cd,R1,R2),[0,Ts],h(i-1));
            h0 = y(end); % take the last point
            h(i) = h0; % store the height for plotting

            erro(i)=ref(i) - h(i)% + ruido(i); %Erro

            rate(i)=(erro(i) - erro(i-1));%/Tc; %Rate of erro

            Ami = 1; 

                        %Controlador:

                        Kp(i)= Kc/Ami;
                        Kd(i)= (Td)*Kc/Ami;
                        Ki(i)= (Kc/Ami)/(Ti);

                        alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                        beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                        gama = (Kc/Ami)*(Td)/Tamostra;

                        u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2) ;%+ disturbio(i);

                        %saturadores:
                        if(u(i)<5e-5) u(i)=5e-5;end;
                        if(u(i)>2*3.8000e-04) u(i)=2*3.8000e-04;end;

                        tempo(i)=i*Tamostra;

        end
        
        