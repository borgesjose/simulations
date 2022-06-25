
clear; clc; close all;

L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

param = [-L,0,-L,0,L,0,L,-L,0,-L,0,L,0,L];        
%%
M=[];
M1=[];
for i=1:length(x), % De 1 at� o No. total de medidas da variavel linguistica...
  [mi, mo] = pertinencias_T1(x(i),y(i),param);  % pertinencia aos conjuntos fuzzy (curvatura)
  M=[M; mi];
  M1=[M1; mo];
end

figure; hold on
plot(x,M(:,1),'r-'); % gr�fico conjunto fuzzy erro NEGATIVO
plot(x,M(:,2),'b-'); % gr�fico conjunto fuzzy erro ZERO
plot(x,M(:,3),'m-'); % gr�fico conjunto fuzzy erro POSITIVO
hold off
%axis([0 0.2 0 1.2]);
xlabel('ERRO');
legend('NEGATIVO','ZERO','POSITIVO')

figure; hold on
plot(y,M1(:,1),'r-'); % gr�fico conjunto fuzzy rate NEGATIVO
plot(y,M1(:,2),'b-'); % gr�fico conjunto fuzzy rate ZERO
plot(y,M1(:,3),'m-'); % gr�fico conjunto fuzzy rate POSITIVO
hold off
%axis([0 0.2 0 1.2]);
xlabel('RATE');
legend('NEGATIVO','ZERO','POSITIVO')
