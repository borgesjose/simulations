function y = sigmoid_mf_t1(x,param)
    a = param(1);
    c = param(2);
    
    y = 1/(1 + exp(-a*(x-c)));

end