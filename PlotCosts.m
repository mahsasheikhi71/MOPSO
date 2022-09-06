function PlotCosts(pop,rep)
    
    for i=numel(pop):-1:1
        if pop(i).Feasible==0
            pop(i)=[];
        end
    end
    
    for i=numel(rep):-1:1
        if rep(i).Feasible==0
            rep(i)=[];
        end
    end
    
    if (~isempty(pop))
        pop_costs=[pop.Cost];
        plot3(pop_costs(1,:),pop_costs(2,:),pop_costs(3,:),'ko');
    end
    hold on;
    if (~isempty(rep))
        rep_costs=[rep.Cost];
        plot3(rep_costs(1,:),rep_costs(2,:),rep_costs(3,:),'r*');
    end
    
    xlabel('1st Objective');
    ylabel('2nd Objective');
    zlabel('3rd Objective');
    
    grid on;
    
    hold off;

end

