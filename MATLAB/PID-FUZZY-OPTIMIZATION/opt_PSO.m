function param = opt_PSO(FuzzyType,FT1type,FT2Itype,L,pso)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2022 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  -- Otimiza��o de fun��es de pertinenecia de um     %
% Cont. Fuzzy Tipo 2 com Particle Swarm Optimization  %
%  -- Version: 1.0  - 03/07/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Particle Swarm Optimization:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

noP = pso.noP; 
maxIter = pso.maxIter; 
dataVis = pso.visFlag;


% ub = pso.ub;
% lb = pso.lb;
fobj = pso.fobj;

wMax = pso.wMax;
wMin = pso.wMin; 
c1 = pso.c1; 
c2 = pso.c2; 


        if (FuzzyType == 'T1'),
            
            if (FT1type == 'L')
                nVar = 14;
                
            elseif (FT1type == 'N')
                
                nVar =  6;
    
            end;
            
            
         elseif (FuzzyType == 'T2'),
            
            if (FT2Itype == 'L')
                
                nVar =  16;
                
            elseif (FT2Itype == 'N')
                
                nVar =  12;
                
            end;
        end

         ub = L * ones(1, nVar);
         lb = -L * ones(1, nVar);
         
         vMax =  (ub - lb).*0.2;
         vMin =  -vMax;

         
% Extra variables for data visualization
average_objective = zeros(1, maxIter);
cgCurve = zeros(1, maxIter);
FirstP_D1 = zeros(1 , maxIter);
position_history = zeros(noP , maxIter , nVar );        


%% Step1: Randomly initialize Swarm population of N particles

for k = 1 : noP
    Swarm.Particles(k).X = (ub-lb) .* rand(1,nVar) + lb;
    Swarm.Particles(k).V = zeros(1, nVar);
    Swarm.Particles(k).PBEST.X = zeros(1,nVar);
    Swarm.Particles(k).PBEST.O = inf;
    
    Swarm.GBEST.X = zeros(1,nVar);
    Swarm.GBEST.O = inf;
end

for t = 1 : maxIter
    
    % Calcualte the objective value
    for k = 1 : noP
        
        currentX = Swarm.Particles(k).X;
        position_history(k , t , : ) = currentX;
        
        script_PID_FXX_FG_PSO;
        
        Swarm.Particles(k).O = fobj(erro,tempo,'ITAE');%fobj(currentX);
        average_objective(t) =  average_objective(t)  + Swarm.Particles(k).O;
        
        % Update the PBEST
        if Swarm.Particles(k).O < Swarm.Particles(k).PBEST.O
            Swarm.Particles(k).PBEST.X = currentX;
            Swarm.Particles(k).PBEST.O = Swarm.Particles(k).O;
        end
        
        % Update the GBEST
        if Swarm.Particles(k).O < Swarm.GBEST.O
            Swarm.GBEST.X = currentX;
            Swarm.GBEST.O = Swarm.Particles(k).O;
        end
    end
    
    % Update the X and V vectors
    w = wMax - t .* ((wMax - wMin) / maxIter);
    
    FirstP_D1(t) = Swarm.Particles(1).X(1);
    
    for k = 1 : noP
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* rand(1,nVar) .* (Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
            + c2 .* rand(1,nVar) .* (Swarm.GBEST.X - Swarm.Particles(k).X);
        
        
        % Check velocities
        index1 = find(Swarm.Particles(k).V > vMax);
        index2 = find(Swarm.Particles(k).V < vMin);
        
        Swarm.Particles(k).V(index1) = vMax(index1);
        Swarm.Particles(k).V(index2) = vMin(index2);
        
        Swarm.Particles(k).X = Swarm.Particles(k).X + Swarm.Particles(k).V;
        
        % Check positions
        index1 = find(Swarm.Particles(k).X > ub);
        index2 = find(Swarm.Particles(k).X < lb);
        
        Swarm.Particles(k).X(index1) = ub(index1);
        Swarm.Particles(k).X(index2) = lb(index2);
        
    end
    
    if dataVis == 1
        outmsg = ['Iteration# ', num2str(t) , ' Swarm.GBEST.O = ' , num2str(Swarm.GBEST.O)];
        disp(outmsg);
    end
    
    cgCurve(t) = Swarm.GBEST.O;
    average_objective(t) = average_objective(t) / noP;
    
    fileName = ['Resluts after iteration # ' , num2str(t)];
    save( ['./results/PSO/',fileName])
end
GBEST = Swarm.GBEST;

        
end