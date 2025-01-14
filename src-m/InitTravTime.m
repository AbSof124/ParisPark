function [ traveltime ] = InitTravTime( traveltime,pedg,i,v)
%UNTITLED2 Summary of this functitton goes here
%   Detailed explanation goes here
 
    if i==1
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        x=traci.edge.getTraveltime(road{k});
        traveltime = traveltime+x;
    end
    elseif i==2
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==3
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==4
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==5
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==6
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==7
    traci.vehicle.changeTarget(v,pedg);
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    end
end

