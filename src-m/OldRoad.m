function [ oldroads ] = OldRoad(sub1,j,ParkEdgeID,v)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here 
            if sub1(j)==1
                traci.vehicle.changeTarget(v,ParkEdgeID{1});
                newroad{j}=traci.vehicle.getRoute(v);
            elseif sub1(j)==2
                 traci.vehicle.changeTarget(v,ParkEdgeID{2});
                newroad{j}=traci.vehicle.getRoute(v);
            elseif sub1(j)==3
                 traci.vehicle.changeTarget(v,ParkEdgeID{3});
                newroad{j}=traci.vehicle.getRoute(v);
                elseif sub1(j)==4
                 traci.vehicle.changeTarget(v,ParkEdgeID{4});
                newroad{j}=traci.vehicle.getRoute(v);
                elseif sub1(j)==5
                 traci.vehicle.changeTarget(v,ParkEdgeID{5});
                newroad{j}=traci.vehicle.getRoute(v);
                elseif sub1(j)==6
                 traci.vehicle.changeTarget(v,ParkEdgeID{6});
                newroad{j}=traci.vehicle.getRoute(v);
                elseif sub1(j)==7
                 traci.vehicle.changeTarget(v,ParkEdgeID{7});
                newroad{j}=traci.vehicle.getRoute(v);
            end
oldroads=newroad{j};
end

