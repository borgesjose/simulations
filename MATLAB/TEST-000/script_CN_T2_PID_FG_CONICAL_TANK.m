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
Tc = 10;
PID = 0;

b1 = 0.0396;
b2 = 0.0090;

a1 = -0.8226;
a2 = 0;


% figure;
% step(ftz,50)

%% Aplicando o relé:
Tc = 10;
Tamostra = Tc;
%Kc = 1.7519;
%Ti = 0.6913;
%Td = 0.1728;
nptos = 800;
d = 80;
eps = 1;
[Kc,Ti,Td] = Ident_com_rele(a1,a2,b1,b2,Tamostra,nptos,d,eps);
% [Kc,Ti,Td] = Ident_com_rele_ZN(Tamostra,nptos,d,eps);
%% Definições do Controlador: 

Am_min = 2; 
Am_max = 5;
Theta_m_min = 45;%45;
Theta_m_max = 72;%72;
     
%Aplicando o CN-PID-FG:

 turnpoint = floor(rand*nptos);
% for i=1:nptos,
%     if (i<=turnpoint)  ref(i)=2; end;
%     if (i>turnpoint)   ref(i) = 1; end;
% end ;
for i=1:nptos,
    if (i<=nptos/4)  ref(i)=10; end;
    if (i>nptos/4)   ref(i) = 50; end;
    if (i>nptos/2 & i<=3*nptos/4)  ref(i)=25; end;
    if (i>3*nptos/4)   ref(i) = 60; end;
end ;
clear y;
y(4)=0 ; y(3)=0 ; y(2)=0 ; y(1)=0 ; 
u(1)=0 ; u(2)=0 ; u(3)=0; u(4)=0;


erro(1)=1 ; erro(2)=1 ; erro(3)=1; erro(4)=1;

L=4;%Provavelmente o valor de limite das memberships functions



%%
  
 gene = populacao{h,1}(1:16);
 
 Param = [gene,1,1];

    for i=5:nptos,

         y(i) = -a1*y(i-1)-a2*y(i-2)+b1*u(i-1)+b2*u(i-2);

         erro(i)=ref(i)-y(i); %Erro

         rate(i)=(erro(i)-erro(i-1));%/Tc; %Rate of erro


         %Aqui na chamada da função é possivel escolher entre linear input(LI) e não linear input(NLI)
%          Am(i) = Inferencia_T2_S(erro(i),rate(i),L,Param,'LI');
%          [R, Y] = Inferencia_T2(erro(i),rate(i),L,Param,'LI');
%          [Am(i),yl(i),yr(i),l(i),r(i)]=EIASC(R(:,1)',R(:,2)',Y(:,1)',Y(:,2)'); 
           
           if (PID == 0)
               Am(i) = Inferencia_T2(erro(i),rate(i),L,Param,'LI');
               Ami = Am(i)*Am_max + Am_min*(1 - Am(i));
           else
               Ami = 1; end;
          
          %Controlador:

                Kp(i)= Kc/Ami;
                Kd(i)= (Td)*Kc/Ami;
                Ki(i)= (Kc/Ami)/(Ti);
                
                alpha = (Kc/Ami)*(1+((Td)/Tamostra)+(Tamostra/(2*(Ti))));
                beta = -(Kc/Ami)*(1+2*((Td)/Tamostra)-(Tamostra/(2*(Ti))));
                gama = (Kc/Ami)*(Td)/Tamostra;

                u(i)= u(i-1) + alpha*erro(i) + beta*erro(i-1) + gama*erro(i-2);

           tempo(i)=i*Tamostra;
            
     end ;

     
