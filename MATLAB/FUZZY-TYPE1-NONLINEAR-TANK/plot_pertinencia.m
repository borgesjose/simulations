% Script para testas am Membership functions

L = 8;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

erro = -2*L:0.001:2*L;
rate = -2*L:0.001:2*L;

param = [-L/2,0,L/2];
paramT = [-L,-L/2,L/2,L];

paramS = [0.8,0];


for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
  zero(i) = sigmoid_mf_t1(x(i),paramS);
  rate(i) = 1-sigmoid_mf_t1(x(i),paramS); 
  %pertinencias(x(i),y(i),L,'LI');  % pertinencia aos conjuntos fuzzy (curvatura)
%   M=[M; mi];
%   M1=[M1; mo];
end 

figure; hold on
plot(x,zero,'r-'); % gráfico conjunto fuzzy erro NEGATIVO
plot(x,rate,'b-'); % gráfico conjunto fuzzy erro NEGATIVO