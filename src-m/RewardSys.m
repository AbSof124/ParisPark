function [ AddT,Sub1,ResPl,A,RsPrio] = RewardSys(tempV,departVeh,DF,vehinsim,vehicles,AddT,dist,Sub1,A,RsPrio,ResPl,ParkEdgeID,parknb,ConvtimeV,step)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%WalkingT=[200,400,600,800,1200,1600];
%vrs=1;

for c=1: vehinsim-1%traci.simulation.getDepartedNumber()
    v=departVeh{1,c};
    j=departVeh{2,c};
    idxp=Sub1(j);

if ismember(v,vehicles) && A(j)==0%&& ismember(v,RewS)
     vehPosition = traci.vehicle.getPosition(v);
      x=DF(tempV{5,j},1); y=DF(tempV{5,j},2);
      z=vehPosition(1);w=vehPosition(2);

      distance2D = traci.simulation.getDistance2D(x, y, z, w);
      Rmt=distance2D/55;
      
%   RemainigTT=(distance2D/1.4)*1000;
        WalkingT=randi([10,30],1);
       if round(Rmt)>WalkingT && round((Rmt-WalkingT))>=5
        AddT=[AddT,round((Rmt-WalkingT))]; 
%   Find closest value to Median
       S = sort(dist,2);
       mean=median(S(j,:));
       idScol = find(S(j,:) >= distance2D,1);
       [row,idMedcol] = find(dist==S(j,3),1);
       Sub1(j)=idMedcol;
       %AddT=[AddT,1];
        elseif round(Rmt)<WalkingT && round((WalkingT-Rmt))>5 && RsPrio(j)~=0
            % verifier s'il est prioritaire (vect 0/non et 1/oui)
            rf=nnz(~ResPl{idxp});
            for i=1:parknb
            if idxp==i && nnz(~ResPl{idxp})>0
                 pl=1/exprnd(500);
                 s=1/pl*10000;
                 pedg=ParkEdgeID{i};
                 traci.vehicle.changeTarget(v,pedg); %set the road
                  traci.vehicle.setStop(v,num2str(i),1,0,s,65);
                   A(j)=1;
                   RsPrio(j)=0;
                 % ResPl(idxp)= ResPl(idxp)-1;
                  
                  Av=round((1/pl*10+ConvtimeV(j,idxp)/5000))+step;
                  e1 = find(ResPl{i}==0,1);
                   ResPl{i}(e1)=Av;
            end
            end
            %mais on doit controler de n'est pas excedé le nb des places
            %reserver (ajouter un compt pour chaque park
            %si tt est bon l'affecter directement ici (setstop)
            %ca veut dire de validé la premiére préaffectation deja faite
            %parce quel est deja faite pour le park le plus proche
           %AddT=[AddT,0]; 
       end  

end
end
%rf=nnz(AddT);
end

