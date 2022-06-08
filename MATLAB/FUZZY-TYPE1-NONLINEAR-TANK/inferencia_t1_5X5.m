function output = inferencia_t1_5X5(pert_erro,pert_rate),
%
% Implementacao de um controlador Fuzzy baseado em margem de fase e de ganho 
% do tipo Gain Scheduling
%
% Autor: jose borges
% Data: 07/03/2022 


% REGRA 1:  

R1 = pert_erro(1)*pert_rate(1);

% REGRA 2: 

R2 =pert_erro(1)*pert_rate(2)  ;

% REGRA 3: 

R3= pert_erro(1)*pert_rate(3);

% REGRA 4: 

R4 = pert_erro(1)*pert_rate(4);

% REGRA 5:

R5 = pert_erro(1)*pert_rate(5);

% REGRA 6: 

R6= pert_erro(2)*pert_rate(1);

% REGRA 7: 

R7 = pert_erro(2)*pert_rate(2);

% REGRA 8: 

R8 = pert_erro(2)*pert_rate(3);

% REGRA 9: 

R9= pert_erro(2)*pert_rate(4);

% REGRA 10:  

R10 = pert_erro(2)*pert_rate(5);

% REGRA 11: 

R11 =pert_erro(3)*pert_rate(1)  ;

% REGRA 12: 

R12= pert_erro(3)*pert_rate(2);

% REGRA 13: 

R13 = pert_erro(3)*pert_rate(3);

% REGRA 14:

R14 = pert_erro(3)*pert_rate(4);

% REGRA 15: 

R15= pert_erro(3)*pert_rate(5);

% REGRA 16: 

R16 = pert_erro(4)*pert_rate(1);

% REGRA 17: 

R17 = pert_erro(4)*pert_rate(2);

% REGRA 18: 

R18= pert_erro(4)*pert_rate(3);

% REGRA 19:  

R19 = pert_erro(4)*pert_rate(4);

% REGRA 20: 

R20 =pert_erro(4)*pert_rate(5)  ;

% REGRA 21: 

R21= pert_erro(5)*pert_rate(1);

% REGRA 22: 

R22 = pert_erro(5)*pert_rate(2);

% REGRA 23:

R23 = pert_erro(5)*pert_rate(3);

% REGRA 24: 

R24= pert_erro(5)*pert_rate(4);

% REGRA 25: 

R25 = pert_erro(5)*pert_rate(5);



output = R1*(exp(-R1*4)) + R2*(exp(-R2*4)) + R3*(exp(-R3*4))+ R4*(1 - exp(-R4*4)) + R5*(1 - exp(-R5*4)) + R6*(1 - exp(-R6*4))+ R7*(exp(-R7*4)) + R8*(exp(-R8*4)) + R9*(exp(-R9*4));
end