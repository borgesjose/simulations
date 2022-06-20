function [fig1,fig2,fig3,fig4] = p_ft1(ts,h,ref,u,tempo,Kp,Kd,,Ki,Am))
fig1 = figure;
        plot(ts,h,'-r','LineWidth', 3,'DisplayName','height'); hold on
        plot(ts,ref,'k:','LineWidth', 3,'DisplayName','reference'); hold off
        ylabel('Tank Height (m)');
        xlabel('Time (s)');
        title(['Fuzzy tipo 1 - Resposta Tanque - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        saveas(gcf,['./figures/',num2str(h0),'.png'])
        
        fig2 = figure;
        plot(ts,u,'k:','LineWidth', 3,'DisplayName','input'); hold off
        ylabel('Sinal de entrada m³/s');
        xlabel('Time (s)');
        legend();
        title(['Fuzzy tipo 1 - Sinal de Controle - R1: ', num2str(R1) , '  R2: ' , num2str(R2), '  r: ' , num2str(r)])
        saveas(gcf,['./figures/','Sinal_de_controle_R1_',num2str(R1),'R2_',num2str(R2), 'r_',num2str(r),'.png'])
        
        fig3 = figure;
        grid;
        plot(tempo,Kp,'g-');
        hold on;
        plot(tempo,Kd);
        hold on;
        plot(tempo,Ki);
        title('FT1-PID-FG: Kp,Ki,Kd')
        legend('Kc','Kd','Ki')
        saveas(gcf,['./figures/',num2str(h0),'.png'])

        fig4 = figure;
        grid;
        plot(tempo,Am,'g-');
        title('FT1-PID-FG: Am')
        saveas(gcf,['./figures/',num2str(h0),'.png'])



end