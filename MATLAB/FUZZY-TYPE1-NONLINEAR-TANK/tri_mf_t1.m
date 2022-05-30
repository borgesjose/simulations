function y = tri_mf_t1(x,param)
    a = param(1);
    b = param(2);
    c = param(3);   
    
    y = max(min((x-a)/(b-a),(c-x)/(c-b)),0);

end