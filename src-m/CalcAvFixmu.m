function [ParkAv] = CalcAvFixmu(cap,EstAv,ConvtimeV,nb)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
lti=0.7;
travtime=mean(ConvtimeV);
for j=1:nb
    mu(j)=1/exprnd(500);%0.009;
    temp=EstAv{j};
    lamda(j)=lti*mu(j)*(cap(j)-nnz(temp));
    Avail(j)=(cap(j)-nnz(EstAv{j}));
    if length(travtime)>1
    ParkAv(j)= exp((-(lamda(j)+(Avail(j)*mu(j))))*(travtime(j)/60000));
    else
    ParkAv(j)= exp((-(lamda(j)+(Avail(j)*mu(j))))*(mean(ConvtimeV)/60000)); 
    end
end
 
end

