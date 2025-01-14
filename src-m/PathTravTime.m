function [ min2 ] = PathTravTime(newroad,olroad,costolroad)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
min2=0;
for k=1:length(newroad)
        min2 = min2+traci.edge.getTraveltime(newroad{k});
end
 
if length(newroad)~=length(olroad)
    min2=min2*5000+costolroad;
elseif ~strcmp(newroad,olroad)
    min2=min2*5000+costolroad;
else
    min2=costolroad;%min2+wlktime;
end

end

