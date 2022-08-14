function [h,u]=histeresis_relay(n, Tc, d0, d1, d2, eps, A,Cd);
    
    dmax = d0 + d1;
    dmin = d0 - d2;
    
    for i=1:n,
        y(i)=0;
        ref(i)= 0.15;
    end;
    e(1)=0; e(2)=0; 
    h(1)=0.01 ; h(2)=0.01 ; h(3)=0.01 ; h(4)=0.01; h(5)=0.01;

    u(1)=0.00001 ; u(2)=0.00001 ; u(3)=0.00001 ; u(4)=0.00001; u(5)=dmax;
h0 = h(5);
    for i=5:n,
        
        [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd),[0,Tc],h0);
        h0 = y(end); % take the last point
        h(i) = h0; % store the height for plotting

       e(i)= ref(i)- h(i);

       if ((abs(e(i))>eps) & (e(i)>0))  u(i+1)=dmax; end;
       if ((abs(e(i))>eps) & (e(i)<0))  u(i+1)=dmin; end;
       if ((abs(e(i))<eps) & (u(i)==dmax))  u(i+1)=dmax; end;
       if ((abs(e(i))<eps) & (u(i)==dmin))  u(i+1)=dmin; end;
       if (e(i)==eps)  u(i+1)=dmax; end;
       if (e(i)==-eps)  u(i+1)=dmin; end;

    end;

end