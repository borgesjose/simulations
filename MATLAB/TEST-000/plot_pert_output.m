%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
%PLOTAGEM
figure;
grid;
plot(tempo,Kp,'b-');
hold on;
plot(tempo,Ki,'r-');
plot(tempo,Kd,'m-');
legend('Kp','Ki','Kd')
%%
%Saida do processo:
%figure;
%grid;
plot(tempo,u,'r-');
hold on;
%plot(tempo,ref,'g--');
%xlabel('tempo(s)');
%ylabel('level (h)');

%plot(tempo,u,'b-');
%%
%plot(tempo,ref,'g--');
xlabel('tempo(s)');
ylabel('control variable (u)');
legend('PID','Fuzzy-Type-2-PID-FG')
%%
legend('PID','Fuzzy-Type-2-PID-FG', 'reference')
%%
%title(['T2,NLI: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)])
%title('Resposta ao degrau com mudança referência')

plot(tempo,ref,'g--');
xlabel('tempo(s)');
ylabel('control variable (u)');
%legend('PID','Type-2-CN-PID-FG', 'reference')
% text1= {'T1: ISE:', num2str(ISE_t1), ', ITSE:' ,num2str(ITSE_t1),', IAE:' ,num2str(IAE_t1), ', ITAE:' ,num2str(ITAE_t1)}
% text2= {'T2,IM-1: ISE:', num2str(ISE_t2), ', ITSE:' ,num2str(ITSE_t2),', IAE:' ,num2str(IAE_t2), ', ITAE:' ,num2str(ITAE_t2)}
% text([100],[0.5],text2)
%%
% versão minha
n = 1 - (1./(1+exp(-10.*x + 5)))

p = 1./(1+exp(-10.*x + 5))
%%
z = exp((-1/2).*((x - 0.5)./0.08).^2)
%%
%versão codigo
n =  -(1/4).*log(1.-x)

p = -(1/4).*log(x)
%%
z = 1 - exp(-(0.1./abs(0.5-x)).^2.5)
%%
x = 0:0.001:1;

for i=1:length(x), % De 1 até o No. total de medidas da variavel linguistica...
    
    if n(i) > 1
        n(i) = 1;
    end
    if p(i) > 1
        p(i) = 1;
    end  
end

figure;
grid;
plot(x,n,'b-');
hold on;
plot(x,z,'m-');
plot(x,p,'r-');
legend('N','Z','P')

ylim([0 1.3])
