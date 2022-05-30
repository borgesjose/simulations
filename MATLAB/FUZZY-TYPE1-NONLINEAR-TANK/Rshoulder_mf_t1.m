function y = Rshoulder_mf_t1(x,param)
    c = param(1);   
    d = param(2);
    
    y = max(min([1,(d-x)/(d-c)]),0);

end