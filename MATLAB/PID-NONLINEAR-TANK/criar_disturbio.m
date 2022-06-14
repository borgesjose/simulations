            Ts = 5; % periodo de amostragem para processo de nivel em um tanque  5~10s( Digital control systems,Landau,2006,p.32)
            Tsim = 2000;
            nptos = Tsim/Ts;
            ts = linspace(0,Tsim,nptos);

disturbio = randn(1,nptos)*1.0e-06

figure;
        plot(ts,disturbio,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Ruido');
        xlabel('Time (s)');
        legend();
        title(['Sinal de Ruido aplicado ao sinal de controle'])
        %saveas(gcf,['Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2),
        %'r_',num2str(r),'.png']) 
        
%%
