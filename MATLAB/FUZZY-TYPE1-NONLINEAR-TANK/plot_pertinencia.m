% Script para testas am Membership functions

L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

erro = -2*L:0.001:2*L;
rate = -2*L:0.001:2*L;

param = [-L/2,0,L/2];
paramT = [-L,-L/2,L/2,L]
for i=1:length(x), % De 1 at� o No. total de medidas da variavel linguistica...
  erro(i) = tri_mf_t1(x(i),param);
  rate(i) = tra_mf_t1(x(i),paramT); 
  %pertinencias(x(i),y(i),L,'LI');  % pertinencia aos conjuntos fuzzy (curvatura)
%   M=[M; mi];
%   M1=[M1; mo];
end

figure; hold on
plot(x,erro,'r-'); % gr�fico conjunto fuzzy erro NEGATIVO
plot(x,rate,'b-'); % gr�fico conjunto fuzzy erro NEGATIVO