function y = cauchy_mf_t1(x,param),
    a = param(1);
    b = param(2);
    c = param(3);
    y = 1/(1 + abs((x-c)/a)^(2*b));

end