function y = Lshoulder_mf_t1(x,param)
    a = param(1);
    b = param(2);
    
    
    y = max(min([(x-a)/(b-a),1]),0);

end