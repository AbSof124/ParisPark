function [ EstAv ] = GetFreeSpace(EstAv,cap,step)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
for j=1:7
     
for i=1:cap(j)
if EstAv{j}(i)<step
EstAv{j}(i)=0;
end
end

end

end

