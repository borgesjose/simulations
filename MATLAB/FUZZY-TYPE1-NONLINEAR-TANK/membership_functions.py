def singleton_t1(c,x):
    if x==c: y = 1;
    else: y=0;

    return y

def tri_mf_t1(x,param):
    a = param[0];
    b = param[1];
    c = param[2];  

    y = max(min((x-a)/(b-a),(c-x)/(c-b)),0);
    
    return y

