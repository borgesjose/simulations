function y = tra_mf_t1(x,param)
    a = param(1);
    b = param(2);
    c = param(3);   
    d = param(4);
    
    y = max(min([(x-a)/(b-a),1,(d-x)/(d-c)]),0);

end