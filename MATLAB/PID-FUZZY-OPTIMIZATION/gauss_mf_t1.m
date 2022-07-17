function y = gauss_mf_t1(x,m,rho),

%Retorna os valores de uma MF gausiana com os paramentros, 
% m e rho, em um dado ponto x

y = exp(-(1/2)*((x-m)/rho)^2);

end