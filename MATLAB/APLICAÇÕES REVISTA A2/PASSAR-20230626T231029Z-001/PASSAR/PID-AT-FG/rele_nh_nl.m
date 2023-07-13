
function [h,u]=rele_nh_nl(n, Tc, d, A,Cd);
   
   R1 = 0.125;
   R2 = 0.01;
   
   dmax = d ;%+ 9e-4;
   dmin = -d ;%- 9e-4;
    
    for i=1:n,
        y(i)=0;
        ref(i)= .2;
    end;
    e(1)=0; e(2)=0; 
    h(1)=0.01 ; h(2)=0.01 ; h(3)=0.01 ; h(4)=0.01; h(5)=0.01;

    u(1)=0.00001 ; u(2)=0.00001 ; u(3)=0.00001 ; u(4)=0.00001; u(5)=dmax;

    for i=5:n,
        
        [~,y] = ode45(@(t,y) tank_conical(t,y,A,u(i),Cd,R1,R2),[0,Tc],h(i));
        h0 = y(end); % take the last point
        h(i+1) = h0; % store the height for plotting

       %y(t)= -theta*y(t-1) - phi*y(t-2) + alpha*u(t-2) + beta*u(t-3) + gama*u(t-4);
       e(i)= ref(i)- h(i);

       if (e(i)>0) u(i+1)= dmax; end;
       if (e(i)<0)  u(i+1)= dmin; end;


    end;

end