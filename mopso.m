clc;
clear;
close all;

%% Problem Parameters
Parameter.FixPenalty=30000;
Parameter.VarPenalty=3;

Parameter.i=3;
Parameter.h=5;
Parameter.t=3;
Parameter.p=1;
Parameter.v=4;
Parameter.r=3;
Parameter.j=2;
Parameter.k=3;

Parameter.Tomax=[3];
Parameter.Capacity=300;
Parameter.Tmax=20;
Parameter.Av=70;
Parameter.VC=20;
Parameter.Umax=120;
Parameter.Umin=30;

Parameter.G=[1];

Parameter.F=[100,100];

Parameter.SC=[7    5    1];

Parameter.OS=[12  10
              10  12
              10  12];

Parameter.hc=[0.1
              0.1];

Parameter.d=[10    10    10    10    10
    10    10    10    10    10
    10    10    10    10    10
    10    10    10    10    10
    10    10    10    10    10];

Parameter.tc=[0.05  0.05  0.05  0.05  0.05
              0.05  0.05  0.05  0.05  0.05
              0.05  0.05  0.05  0.05  0.05
              0.05  0.05  0.05  0.05  0.05
              0.05  0.05  0.05  0.05  0.05];


Parameter.Demand=[70   70   70
                  70   70   70
                  70   70   70];

Parameter.C=[300
             400
             500];

Parameter.Ps=[2];

Parameter.Ca=[200,200];

%% Problem Definition

CostFunction=@(A,B,C,D) ObjectiveFCN(A,B,C,D,Parameter);   % Cost Function


%% MOPSO Parameters

MaxIt=100;          % Maximum Number of Iterations

nPop=50;            % Population Size

nRep=50;            % Repository Size

w=0.5;                % Inertia Weight
wdamp=1;         % Intertia Weight Damping Rate
c1=1;               % Personal Learning Coefficient
c2=2;               % Global Learning Coefficient

nGrid=5;            % Number of Grids per Dimension
alpha=0.1;          % Inflation Rate

beta=2;             % Leader Selection Pressure
gamma=2;            % Deletion Selection Pressure

%% Initialization

empty_particle.Position1=[];
empty_particle.Position2=[];
empty_particle.Position3=[];
empty_particle.Position4=[];
empty_particle.Velocity1=[];
empty_particle.Velocity2=[];
empty_particle.Velocity3=[];
empty_particle.Velocity4=[];
empty_particle.Cost=[];
empty_particle.Feasible=[];
empty_particle.Best.Position1=[];
empty_particle.Best.Position2=[];
empty_particle.Best.Position3=[];
empty_particle.Best.Position4=[];
empty_particle.Best.Cost=[];
empty_particle.Best.Feasible=[];
empty_particle.IsDominated=[];
empty_particle.GridIndex=[];
empty_particle.GridSubIndex=[];

pop=repmat(empty_particle,nPop,1);

for i=1:nPop
    
    pop(i).Position1=rand([Parameter.j,Parameter.k]);
    pop(i).Position2=rand([Parameter.t*Parameter.r,Parameter.v]);
    pop(i).Position3=rand([Parameter.v*Parameter.t,Parameter.k]);
    pop(i).Position4=rand([Parameter.j,Parameter.i]);
    
    pop(i).Velocity1=zeros(size(pop(i).Position1));
    pop(i).Velocity2=zeros(size(pop(i).Position2));
    pop(i).Velocity3=zeros(size(pop(i).Position3));
    pop(i).Velocity4=zeros(size(pop(i).Position4));
    
    [pop(i).Cost,pop(i).Feasible]=CostFunction(pop(i).Position1,pop(i).Position2,pop(i).Position3,pop(i).Position4);
    
    
    % Update Personal Best
    pop(i).Best.Position1=pop(i).Position1;
    pop(i).Best.Position2=pop(i).Position2;
    pop(i).Best.Position3=pop(i).Position3;
    pop(i).Best.Position4=pop(i).Position4;
    pop(i).Best.Cost=pop(i).Cost;
    pop(i).Best.Feasible=pop(i).Feasible;
    
end

% Determine Domination
pop=DetermineDomination(pop);

rep=pop(~[pop.IsDominated]);

Grid=CreateGrid(rep,nGrid,alpha);

for i=1:numel(rep)
    rep(i)=FindGridIndex(rep(i),Grid);
end


%% MOPSO Main Loop

for it=1:MaxIt
    
    for i=1:nPop
        
        leader=SelectLeader(rep,beta);
        
        pop(i).Velocity1 = w*pop(i).Velocity1 ...
            +c1*rand(size(pop(i).Position1)).*(pop(i).Best.Position1-pop(i).Position1) ...
            +c2*rand(size(pop(i).Position1)).*(leader.Position1-pop(i).Position1);
        pop(i).Position1 = pop(i).Position1 + pop(i).Velocity1;
        
        pop(i).Velocity2 = w*pop(i).Velocity2 ...
            +c1*rand(size(pop(i).Position2)).*(pop(i).Best.Position2-pop(i).Position2) ...
            +c2*rand(size(pop(i).Position2)).*(leader.Position2-pop(i).Position2);
        pop(i).Position2 = pop(i).Position2 + pop(i).Velocity2;
        
        pop(i).Velocity3 = w*pop(i).Velocity3 ...
            +c1*rand(size(pop(i).Position3)).*(pop(i).Best.Position3-pop(i).Position3) ...
            +c2*rand(size(pop(i).Position3)).*(leader.Position3-pop(i).Position3);
        pop(i).Position3 = pop(i).Position3 + pop(i).Velocity3;
        
        pop(i).Velocity4 = w*pop(i).Velocity4 ...
            +c1*rand(size(pop(i).Position4)).*(pop(i).Best.Position4-pop(i).Position4) ...
            +c2*rand(size(pop(i).Position4)).*(leader.Position4-pop(i).Position4);
        pop(i).Position4 = pop(i).Position4 + pop(i).Velocity4;
        
        
        [pop(i).Cost,pop(i).Feasible] = CostFunction(pop(i).Position1,pop(i).Position2,pop(i).Position3,pop(i).Position4);
        
        
        if Dominates(pop(i),pop(i).Best)
            pop(i).Best.Position1=pop(i).Position1;
            pop(i).Best.Position2=pop(i).Position2;
            pop(i).Best.Position3=pop(i).Position3;
            pop(i).Best.Position4=pop(i).Position4;
            pop(i).Best.Cost=pop(i).Cost;
            pop(i).Best.Feasible=pop(i).Feasible;
            
        elseif Dominates(pop(i).Best,pop(i))
            % Do Nothing
            
        else
            if rand<0.5
                pop(i).Best.Position1=pop(i).Position1;
                pop(i).Best.Position2=pop(i).Position2;
                pop(i).Best.Position3=pop(i).Position3;
                pop(i).Best.Position4=pop(i).Position4;
                pop(i).Best.Cost=pop(i).Cost;
                pop(i).Best.Feasible=pop(i).Feasible;
            end
        end
        
    end
    
    % Add Non-Dominated Particles to REPOSITORY
    rep=[rep; pop(~[pop.IsDominated])];
    
    % Determine Domination of New Resository Members
    rep=DetermineDomination(rep);
    
    % Keep only Non-Dminated Memebrs in the Repository
    rep=rep(~[rep.IsDominated]);
    
    % Update Grid
    Grid=CreateGrid(rep,nGrid,alpha);

    % Update Grid Indices
    for i=1:numel(rep)
        rep(i)=FindGridIndex(rep(i),Grid);
    end
    
    % Check if Repository is Full
    if numel(rep)>nRep
        
        Extra=numel(rep)-nRep;
        for e=1:Extra
            rep=DeleteOneRepMemebr(rep,gamma);
        end
        
    end
    
    % Plot Costs
    figure(1);
    PlotCosts(pop,rep);
    
    % Show Iteration Information
    disp(['Iteration ' num2str(it) ': Number of Rep Members = ' num2str(numel(rep))]);
    
    % Damping Inertia Weight
    w=w*wdamp;
    
end

