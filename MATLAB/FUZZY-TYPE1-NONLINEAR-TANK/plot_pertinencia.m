% Script para testas am Membership functions

L = 2;

x=-2*L:0.001:2*L;
y=-2*L:0.001:2*L;

erro = -2*L:0.001:2*L;
rate = -2*L:0.001:2*L;

param = [-L/2,0,L/2];
for i=1:length(x), % De 1 at� o No. total de medidas da variavel linguistica...
  out(i) = tri_mf_t1(x(i),param);
  rate(i) =  
  %pertinencias(x(i),y(i),L,'LI');  % pertinencia aos conjuntos fuzzy (curvatura)
%   M=[M; mi];
%   M1=[M1; mo];
end

figure; hold on
plot(x,out,'r-'); % gr�fico conjunto fuzzy erro NEGATIVO