clc;
clear all;
close all;

%%% GA Parameters
% number of individuals (population size)
M = 10;
% number of genes considering 1 gene per motor (= number of joints)
N = 10;
% Number of parents to be selected                                                                                                                                      
R = 40;
% Number of generations
max_gen = 20; gen = 1;
% Crossover Probability
Pc = 0.6;
% Mutation Probability
Pm = 0.5;

%%% Snake parameters
% simulation time per generation
simTime = 15;
amplitude = 1;
frequency = 0.2;
timestamp = 360;

%%% Generating random genes for initial population
minphiDeg = -90;
maxphiDeg = 90;
minphirad = deg2rad(minphiDeg);
maxphirad = deg2rad(maxphiDeg);
xphi = (maxphirad-minphirad).*rand(M,N) + minphirad;
pop = xphi;

%%% starting loop for GA
while gen<max_gen
    % few initializations for child population
    pop_child = []; ophi = []; effParents = []; nonEffParents = [];
    
    % generating angles for simulation
    angles = generateAngles(M, N, amplitude, frequency, pop, timestamp);

    % Fitness by fetching distance from simulation
    F = simDist(angles);

    % data for plots
    [best(gen), idx_best] = min(F);
    worst(gen) = max(F);
    avg(gen) = mean(F);
    bestInd(gen) = pop(idx_best,:);

    % selection probability
    Ps = F./sum(F);
    % Selecting R parents
    Rparents = interp1(cumsum(Ps), 1:M, rand(R,1), 'next', 'extrap');

    % Selecting effective parents
    for i = 1:R
        if rand < Pc
            effParents(i) = Rparents(i);
        else
            nonEffParents(i) = Rparents(i);
        end
    end

    % removing zeros
    effParents = nonzeros(effParents);
    nonEffParents = nonzeros(nonEffParents);

    % crossover loop
    for i = 1:((M-R)/2)
        % select 2 random parents from effective parents
        idx = random("unid", size(effParents,1), 2, 1);
        % random crossover point (total N genes so possible crossover
        % points are only N-1
        cop = randi(N-1);
        % selected parents for crossover
        xphi1 = xphi(effParents(idx(1)),:);
        xphi2 = xphi(effParents(idx(2)),:);
        % crossover
        ophi1 = [xphi1(1:cop) xphi2(cop+1:end)]; 
        ophi2 = [xphi2(1:cop) xphi1(cop+1:end)];
        % children population genotype creation
        ophi = [ophi;ophi1;ophi2];
    end

    % Mutation loop
    for i = 1:(M-R)
        for j = 1:N
            if rand < Pm
                ophi(i,j) = (maxphirad-minphirad).*rand + minphirad;
            end
        end
    end

    % creating child population
    pop_child = [pop_child; ophi];

    % Population update
    pop = [pop_child; pop(Rparents)];

    % generation increment
    gen = gen + 1;
end

%%% Plotting fitness and best individual over time
figure(2);
subplot(2,1,1);
plot(best);
hold on
plot(worst); plot(avg);
hold off
title("Best, worst and average fitness");
xlabel("Generations","FontWeight", "bold"); ylabel("Fitness", "FontWeight", "bold");
legend("Best", "Worst", "Average", "Location", "best");

% subplot(2,1,2);
% mm_bstInd = movmean(bestInd,50);
% plot(bestInd);
% hold on
% plot(mm_bstInd);
% title("Best Individual");
% xlabel("Generations", "FontWeight", "bold"); ylabel("Decision variable", "FontWeight", "bold");
% legend("Best Individual", "moving average", "Location", "best");

%%% function to generate angles for simulation
function [angles] = generateAngles(nind, njnt, amplitude, frequency, phi, timestamp)
    for i=1:nind % for individual snake (third dim)
        for j=1:timestamp % for time stamp (first dim)
            for k=1:njnt % for individual joint per snake (second dim)
                angles(j,k,i) = amplitude*sin(2*pi*frequency*j+phi(i,k));
            end
        end
    end
end