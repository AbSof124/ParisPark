function [mu,ParkAv,Av] = CalcAv(cap,EstAv,ConvtimeV,pl,step,j,nb)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
lti=0.7;
for i=1:nb
    mu(i)=1/exprnd(500);
    temp=EstAv{i};
    lamda(i)=lti*mu(i)*(cap(i)-nnz(temp));
    Avail(i)=(cap(i)-nnz(EstAv{i}));
    ParkAv(i)= exp((-(lamda(i)+(Avail(i)*mu(i))))*(mean(ConvtimeV(j,i))/60000));  
end

    Av=round((1/mu(pl)*10+ConvtimeV(j,pl)/5000))+step+100;


end

 