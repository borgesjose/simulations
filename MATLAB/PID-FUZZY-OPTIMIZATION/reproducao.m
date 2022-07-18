function pop = reproducao(populacao,mais_aptos,populacao_size,prob_crossover,FuzzyType,Itype,lb,ub)
        
            chromosome_lenght = size(populacao{1,1},2);
            
            score = 0;
        
  if prob_crossover > rand
                pop = [];
                for k=1:size(mais_aptos,2)%

                  %Crossover:
                        %Sortear pais:
                    
                        
                        P1 = Torneio(populacao); 
                        P2 = Torneio(populacao);
    
                        %Cruzamento:
                        a = 0.5;
                        for i=1:chromosome_lenght
                            d(i) = abs( P1(i)-P2(i) );
                            
                            int_min = min(P1(i),P2(i)) - a*d(i);
                            int_max = max(P1(i),P2(i)) + a*d(i);
                            
                            u = int_min + (int_max-int_min)*rand;
                            F1(i) =  abs(u);
                            u = int_min + (int_max-int_min)*rand;
                            F2(i) =  abs(u);
                            
                        if(F1(i)<lb) F1(i)=lb;end;
                        if(F1(i)>ub) F1(i)=ub;end;
                        if(F2(i)<lb) F2(i)=lb;end;
                        if(F2(i)>ub) F2(i)=ub;end;
                                   
                        end
                        %saturation:

                        F1 = eval_candidates(F1,FuzzyType,Itype);
                        F2 = eval_candidates(F2,FuzzyType,Itype);
                        
      
                        pop = [pop;{F1,score};{F2,score}];
                         
                end 
                else 
                         pop = populacao ; 
 
            end
       
end