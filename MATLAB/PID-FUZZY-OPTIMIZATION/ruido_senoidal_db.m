function s = ruido_senoidal_db(x,t,db_level),

% Retorna valor de SNR entre o sinal x e o sinal senoidal na forma y = a*sin(b*t)-a*cos(b*t)


aa = 0.1e-2:0.0001:2.29e-2;
for a = aa
    s = a*sin(.3*t)-a*cos(.3*t);
    if snr(x,s) >= db_level, 
        
    break; 
    
    end;
    
end

end