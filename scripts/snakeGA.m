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

%%% Generating random genes for initial population
minphiDeg = -90;
maxphiDeg = 90;
minphirad = deg2rad(minphiDeg);
maxphirad = deg2rad(maxphiDeg);
xphi = (maxphirad-minphirad).*rand(M,N) + minphirad;

angles = generateAngles(M,N,amplitude, frequency, xphi);

function [angles] = generateAngles(nind, njnt, amplitude, frequency, phi)
    for i=1:nind % for individual snake (third dim)
        for j=1:360 % for time stamp (first dim)
            for k=1:njnt % for individual joint per snake (second dim)
                angles(j,k,i) = amplitude*sin(2*pi*frequency*j+phi(i,k));
            end
        end
    end
end