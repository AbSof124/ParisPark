function [ Sr ] = RevTravTime( vehinsim,departVeh,parknb,ParkEdgeID )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    
for c=1: vehinsim-1%traci.simulation.getDepartedNumber()
    v=departVeh{1,c};
    j= departVeh{2,c};
    for i=1:parknb     
    pedg=ParkEdgeID{i};
    traveltime=0;
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        x=traci.edge.getTraveltime(road{k});
        traveltime = traveltime+x;
    end
    RevDist(j,i)= traveltime;
    end
end
 Sr = sort(RevDist,2);
end

