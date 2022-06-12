function ig = IG(a1,a2,a3,u,r,e),


e1 = sum(abs(u'))/size(u,1)
e11 = (u'*u)%/size(u,1)

e2 = 0

e3 = 0


ig = a1*e1+a2*e2+a3*e3;

end 