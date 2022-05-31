function y = cauchy_mf_t1(x,param),

%Retorna os valores de uma MF Cauchy ou Sino com os paramentros, 
% [a,b,c], em um dado ponto x

    a = param(1);
    b = param(2);
    c = param(3);
    y = 1/(1 + abs((x-c)/a)^(2*b));

end