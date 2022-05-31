function y = tra_mf_t1(x,param)

 %Retorna os valores de uma MF trapezoidal com os paramentros, 
 % [a, b, c, d], em um dado ponto x
 
    a = param(1);
    b = param(2);
    c = param(3);   
    d = param(4);
    
    y = max(min([(x-a)/(b-a),1,(d-x)/(d-c)]),0);

end