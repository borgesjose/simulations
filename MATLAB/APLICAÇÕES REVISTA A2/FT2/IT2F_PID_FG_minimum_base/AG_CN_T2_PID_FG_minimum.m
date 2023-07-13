%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
% Universidade Federal do Piau�                       %
% Campus Ministro Petronio Portela                    %
% Copyright 2018 -Jos� Borges do Carmo Neto-          %
% @author Jos� Borges do Carmo Neto                   %
% @email jose.borges90@hotmail.com                    %
%  -- Otimiza��o de fun��es de pertinenecia de um     %
%  controlador fuzzy tipo 2 com algoritmos geneticos  %
%  -- Version: 1.0  - 17/12/2022                      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% Algoritmo genetico:
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Defini��es do AG
%Parametro de reprodu��o:
   prob_mutation = 0.15;%rand(1);
   prob_crossover = 0.85;%rand(1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%CODIFICA��O:Real
geracoes = 100;
%%
%PASSO 1: 
    %INICIAR A POPULA��O:
        populacao_size = 128; %defino o tamanho da popula��o
        populacao = {};% crio a cell que sera a minha popula��o
        N_mais_aptos = 32; %Numero dos individuos mais aptos que ser�o salvos para a proxima gera��o.

        for j=1:populacao_size %A cria��o da popula��o 
           
           genetic_code = [];
           score = 0;
            
           for i=1:8
                genetic_code = [genetic_code ,abs(-0.5+rand);];
            
            populacao{j,1} = genetic_code;
            populacao{j,2} = score;
           end
        end
        thebest = populacao(j,:);
memoria_thebest(1,:)= thebest;
%% PASSO 2:%Criar o loop de evolu��o:
convergencia = 0;
geracao = 1;
while convergencia == 0,
    
     % Avalia��o dos individuos da popula��o:
    
    for h=1:populacao_size %Etapa de avalia��o da popula��o para o AG   
        script_CN_T2_PID_FG; %Chama o script com o controlador implementado
        Mp = max(y);
        J= objfunc(erro,tempo,'ITAE');
        populacao{h,2} = 2*Mp + (1/J)*10^4;
    end
    
  % Retorna um vetor com os valores dos cromossomos do melhores e outro com a cell contendo os melhores e seu score
    [mais_aptos teste] =  selecao_natural(populacao,N_mais_aptos);
    
  %Testar e salvar do melhor individuo j� encotrado:
                v =  teste(1,:);
                if thebest{1,2} < v{1,2}
                    thebest = teste(1,:);
                end
                memoria_thebest(geracao,:) = thebest;
 %% PASSO 3:
    %Sele��o, Reprodu��o e Muta��o:
                
                filhos = reproducao(populacao,mais_aptos,populacao_size,prob_crossover);%Reprodu��o dos individuos mais aptos
                populacao = mutacao(populacao,prob_mutation);% Realiza a muta��o sobre individuos da popula��o
                resto = escolher_resto(populacao,N_mais_aptos,populacao_size); %escolhe os outros valores para a proxima popula��o
                clear populacao; %Limpa populacao
                populacao = [teste;filhos;resto];%cria a proxima popula��o que tera todos os elementos filhos criados com score = 0,
                %portanto a popula��o salva no final da execu��o do
                %algoritmo possui os primeiros valores como os melhores,
                %seguido de valores com score 0 e seguidos de valores
                %aleatoriso escolhidos da popula��o anterior.
 
 %% PASSO 4: 
 %Teste de parada do algoritmo:
                    if  (isequal(populacao{:,2})| geracao == geracoes), 
                            convergencia = 1;
                            break;
                    end
  geracao = geracao +1                  
end

gene = thebest{1,1}
plot_pertinencias_T2
figure;plot((1./[memoria_thebest{:,2}]));