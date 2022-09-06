function [Z,Feasible]=CostFCN(Pop1,Pop2,Parameter)

for i=1:numel(Pop2)
    S=1/(1+exp(-Pop2(i)));
    if S>rand
        Pop2(i)=0;
    else
        Pop2(i)=1;
    end
end

[~,Input]=sort(Pop1(1,1:Parameter.R));
[~,Output]=sort(Pop1(1,Parameter.R+1:end));

StartInput=zeros(size(Input));
FinishInput=zeros(size(Input));
StartOutput=zeros(size(Output));
FinishOutput=zeros(size(Output));
Invetory=zeros([Parameter.N,Parameter.R]);
Time=0;
for i=1:numel(Input)
    if i==1
        StartInput(i)=Time;
    else
        StartInput(i)=Time+Parameter.D;
    end
    FinishInput(i)=StartInput(i)+sum(Parameter.r(Input(i),:).*Parameter.t(Input(i),:));
    Time=FinishInput(i);
    if i==1
        Invetory(:,i)=(Parameter.r(Input(i),:))';
    else
        Invetory(:,i)=Invetory(:,i-1)+(Parameter.r(Input(i),:))';
    end
end

MatInput=[Input
FinishInput+Parameter.V
Invetory];

for j=1:numel(Output)
    X=zeros([1,Parameter.N]);
    for k=1:Parameter.N
        X(k)=find(MatInput(end-Parameter.N+k,:)>=Parameter.s(Output(j),k),1,'first');        
    end
    
    Y=max(X);
    if Pop2(Output(j))==0
        if j==1
            StartOutput(j)=max([Parameter.DD(Output(j))-sum(Parameter.s(Output(j),:).*Parameter.tp(Output(j),:)),MatInput(2,Y)]);
        else
            StartOutput(j)=max([Parameter.DD(Output(j))-sum(Parameter.s(Output(j),:).*Parameter.tp(Output(j),:)),MatInput(2,Y),FinishOutput(j-1)+Parameter.D]);
        end
        FinishOutput(j)=StartOutput(j)+sum(Parameter.s(Output(j),:).*Parameter.tp(Output(j),:));
        
    elseif Pop2(Output(j))==1
        if j==1
            StartOutput(j)=MatInput(2,Y);
        else
            StartOutput(j)=max([MatInput(2,Y),FinishOutput(j-1)+Parameter.D]);
        end
        FinishOutput(j)=StartOutput(j)+sum(Parameter.s(Output(j),:).*Parameter.tp(Output(j),:));
    end
    for k=1:Parameter.N
        MatInput(end-Parameter.N+k,:)=MatInput(end-Parameter.N+k,:)-Parameter.s(Output(j),k);
    end
end

MatrixInput=[Input
zeros(size(Input))
StartInput
FinishInput
Parameter.r(Input,:)'];

MatrixOutput=[Output
ones(size(Output))
StartOutput
FinishOutput
Parameter.s(Output,:)'];

Inventory=zeros([Parameter.N,Parameter.R+Parameter.S]);

Input=MatrixInput;
Output=MatrixOutput;


Counter=1;
while(~isempty(Input)||~isempty(Output))
    if (isempty(Input))
        Inventory(:,Counter)=Inventory(:,Counter-1)-Output(end-Parameter.N+1:end,1);
        Output(:,1)=[];
    else
        Y=find([Input(4,1),Output(3,1)]==min([Input(4,1),Output(3,1)]));
        if Y(1)==1
            if Counter==1
                Inventory(:,Counter)=Input(end-Parameter.N+1:end,1);
                Input(:,1)=[];
            else
                Inventory(:,Counter)=Inventory(:,Counter-1)+Input(end-Parameter.N+1:end,1);
                Input(:,1)=[];
            end
        elseif Y(1)==2
            Inventory(:,Counter)=Inventory(:,Counter-1)-Output(end-Parameter.N+1:end,1);
            Output(:,1)=[];
        end
    end
    Counter=Counter+1;
    
end

%% Calculate Penalties
Penalty=sum((sum(Inventory.*repmat(Parameter.f',1,Parameter.R+Parameter.S))-Parameter.C).*((sum(Inventory.*repmat(Parameter.f',1,Parameter.R+Parameter.S))-Parameter.C)>0));

%% Calculate Objectives
Tardiness=(MatrixOutput(4,:)-Parameter.DD).*((MatrixOutput(4,:)-Parameter.DD)>0);
Earliness=(Parameter.DD-MatrixOutput(4,:)).*((Parameter.DD-MatrixOutput(4,:))>0);

F1=sum(Earliness.*Parameter.COE);
F2=sum(Tardiness.*Parameter.COT);

B1=sum(sum(repmat((MatrixInput(4,:).*Parameter.LandaI)',1,Parameter.N).*Parameter.r.*Parameter.t));
B2=sum(sum(repmat((MatrixOutput(4,:).*Parameter.LandaJ)',1,Parameter.N).*Parameter.s.*Parameter.tp));

F3=B1+B2;

if Penalty==0
    Feasible=1;
else
    Feasible=0;
    F1=F1+Parameter.FixPenalty+Parameter.VarPenalty*Penalty;
    F2=F2+Parameter.FixPenalty+Parameter.VarPenalty*Penalty;
    F3=F3+Parameter.FixPenalty+Parameter.VarPenalty*Penalty;
end
Z=[F1;F2;F3];

end