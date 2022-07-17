function y = gauss_mf_t1(x,param),

%Retorna os valores de uma MF gausiana com os paramentros, 
% m e rho, em um dado ponto x
m = param(1);
rho = param(2);

y = exp(-(1/2)*((x-m)/rho)^2);

end