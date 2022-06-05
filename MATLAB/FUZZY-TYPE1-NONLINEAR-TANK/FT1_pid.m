function Am = FT1_pid(erro,rate,L)
        
        % Retorna o valor de Am para o controlador  fuzzy  pid de tipo 1; 


        %Fun��es de pertinencia para o erro:
        
        erro_N = Rshoulder_t1(erro,[-L,0]);
        erro_Z = tri_mf_t1(erro,[-L,0,L]); 
        erro_P = Lshoulder_t1(erro,[0,L]);
        
        % Fun��es de pertinencia para o rate:
        rate_N = Rshoulder_t1(erro,[-L,0]);
        rate_Z = tri_mf_t1(erro,[-L,0,L]);
        rate_P = Lshoulder_t1(erro,[0,L]);
        
        
        Am = inferencia_t1_3X3(pert_erro,pert_rate);
        


end