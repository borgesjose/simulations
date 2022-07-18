function param = eval_candidates(param,FuzzyType,Itype),


    if (FuzzyType == 'T1'),

            if (Itype == 'L'),
                param_erro_N = param(1:2);
                param_erro_Z = param(3:5);
                param_erro_P = param(6:7);

                if(param_erro_N(2)<= param_erro_N(1)) param(1:2) = sort(param_erro_N); end 
                if(~(param_erro_Z(3)<= param_erro_Z(4)<=param_erro_Z(5)))  param(3:5) = sort(param_erro_Z);end
                if(param_erro_P(2)<= param_erro_P(1)) param(6:7) = sort(param_erro_N); end 

                param_rate_N = param(8:9);
                param_rate_Z = param(10:12);
                param_rate_P = param(13:14); 

                if(param_rate_N(2)<= param_rate_N(1)) param(8:9) = sort(param_rate_N); end 
                if(~(param_rate_Z(3)<= param_rate_Z(4)<=param_rate_Z(5)))  param(10:12) = sort(param_rate_Z);end
                if(param_rate_P(2)<= param_rate_P(1)) param(13:14) = sort(param_rate_P); end 
            
            
            elseif (Itype == 'N'),
                param_erro_N = param(1:2);
                param_erro_Z = param(3:4);
                param_erro_P = param(5:6);

                param_rate_N = param(7:8);
                param_rate_Z = param(9:10);
                param_rate_P = param(11:12);

            end;


    end
    
    if (FuzzyType == 'T2'),
             
        if (Itype == 'L')
            

        elseif (Itype == 'N')
            
        end;
        
    end
    


end



