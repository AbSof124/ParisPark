%% SUMO Park TEST
% THIS FILE AIMS TO TEST THE Parking PROTOCOL FOR SUMO.


clear
close all
clc

%% MAIN
% HERE, WE START THE SUMO SIMULATOR AND INITIALIZE IT, ACCORDING TO
% THis EXAMPLE 

system('sumo-gui -c C:\Users\SOFIENE\Downloads\SUMO\parking\esiee_roadside_parking.sumocfg&');

[traciVersion,sumoVersion] = traci.init();
%Step simulation---------------------------------------------------------------------
step = 1;istep=1; parknb=7;
%---------------------------------------------------------------------
%----------------------Preparing%Inputs-------------------------------
%DF final destination for each vehicle/PP parking poition/
%DF=[502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42];
DF=[502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;502.63 1015.87;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;385.52 664.28;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;761.27 660.58;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;640.64 839.23;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;667.71 280.43;502.63 1015.87;502.63 1015.87;505.27 1156.42;505.27 1156.42;505.27 1156.42];
%DF=[691.00 727.04;821.32 679.85;849.01 583.68;844.58 668.70;665.79 683.60;814.87 685.29;736.12 711.66;675.67 701.40;720.73 638.77;886.67 754.89;937.95 692.25;1041.98 666.61;900.16 559.80;910.42 481.97;845.62 383.73;935.26 537.32;667.10 518.26;855.92 645.03;870.10 466.40;760.18 475.71;691.00 727.04;821.32 679.85;849.01 583.68;844.58 668.70;665.79 683.60;814.87 685.29;736.12 711.66;675.67 701.40;720.73 638.77;886.67 754.89;937.95 692.25;1041.98 666.61;900.16 559.80;910.42 481.97;845.62 383.73;935.26 537.32;667.10 518.26;855.92 645.03;870.10 466.40;760.18 475.71];
PP=[469.01 638.83;469.01 638.83;486.88 1070.91;638.53 605.43;638.53 605.43;613.89 375.37;486.97 807.43];
%A to ensure that each vehicle assigned just once//maxspeed of vehicle----
%VehInNet id lists of all the vehicle in the network at that moment//vehP vehicle position--------------
A= zeros(1,540);
maxspeed= zeros(1,540);
vehInNet=cell(3,540);
vehInNet(2,:) = {0};
vehInNet(3,:) = {0};
vehP= zeros(1,540);
test=0;pt= zeros(1,parknb);
%ST Demanded staying time for each user-----------------------------------
St=[50000,50000,55000,50000,50000,50000,50000,55000,50000,300000,400000,250000,150000,150000,150000,250000,250000,250000,250000,250000,50000,50000,55000,50000,50000,50000,50000,55000,50000,300000,400000,250000,150000,150000,150000,250000,250000,250000,250000,250000];
%St=[50000,50000,55000,50000,50000,50000,50000,55000,50000,300000,400000,250000,150000,150000,150000,250000,250000,250000,250000,250000];
%St=[10000,50000,15000,20000,30000,10000,50000,15000,20000,30000,40000,20000,10000,10000,10000,20000,20000,20000,20000,20000];
NewvehInNet=vehInNet;
x=0;vehinsim=1;
%ParkEdgeID edges of parking stations(like parking position)
ParkEdgeID={'405494696#0','-405494696#1','23289034','-23714318#1','260136572#3','23792272#0','-23714286#0'};
%*******************************Starting%Simulation**************************
q=1;%EstAv=zeros(7,300);
receve(1,q)=0;receve(2,q)=0;q=q+1;e1=1;e2=1;e3=1;
Avail1=0;Avail2=0;Avail3=0;
ParkAv3=0;ParkAv2=0;ParkAv1=0;
record1=0;record2=0;record3=0;cmpt=0;stop=0;    sp1=0;sp2=0;sp3=0;
while traci.simulation.getMinExpectedNumber() > 0
    

  % Here, we demonstrate how to use the simulationStep command using an
  % argument. In this case, the simulation is performed each 5 seconds,
  % note the behavior when you increase the delay in the gui
%  traci.simulationStep(5000*step);
  traci.simulationStep();
    
%---------------- GET THE VEHICLES IDS INSIDE THE NETWORK
    
    vehicles = traci.vehicle.getIDList();
  
%-------------------Get initiale capacity for each parking%area------------parkingArea.name
for i=1:parknb
cap(i)= str2num( traci.simulation.getParameter(num2str(i),'parkingArea.capacity'));
occ(i) =str2num( traci.simulation.getParameter(num2str(i),'parkingArea.occupancy'));
end
%------------Simulation current time (like sim step)-----------------
  currentTime = traci.simulation.getCurrentTime ();

%-------------------GET NEW ENTRY VEHICLES---------------------------------       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for j=1:length(vehicles)
   fi=find(strcmp(vehInNet,vehicles{j}));
       if find(strcmp(vehInNet,vehicles{j})) 
          %vehInNet{3,j}= -1;
          NewvehInNet{1,j}=vehicles{j};
       else 
           vehInNet{3,j}=step;
        NewvehInNet{1,j}=vehicles{j};
        NewvehInNet{3,j}=step;
       end
      
      
   end
   for j=1:length(vehicles)
       vehInNet{1,j}=NewvehInNet{1,j};
       vehInNet{3,j}=NewvehInNet{3,j};
   end
   
%---------------Starting implimenting the RS-----!!!!
% if mod(step,10)==0 && ismember(tempV{1,1},vehicles)
%      vehPosition = traci.vehicle.getPosition(tempV{1,1});
%       x=DF(1,1); y=DF(1,2);
%       z=vehPosition(1);w=vehPosition(2);
%       %z=PP(i,1); w=PP(i,2);
%       distance2D = traci.simulation.getDistance2D(x, y, z, w);
%       RemainigTT=distance2D/1.8;
%       WalkingT=100;
%       if RemainigTT>WalkingT
%        VinRS=tempV{3,1}; 
%       end  
% 
% end
%----------------------------------------------------------------

%----------------------------INITIALIZATION-----------------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if test ==0
             test=1;
     deltaT = traci.simulation.getDeltaT();
     for j=1:parknb
         i=cap(j);
         vect=zeros(1,i);
     EstAv{j}=vect;
     %this is how to acess elem of cellarray ( M2{1}(1), M2{1}(2) )
     end
     %--------------------------ADD flow of Vehicle--------------------------
%    edg={'d','-b','-a','a','b','c'};
%    for i=1:30
%       %edg=randsample(edges,1); 
%       Seq1 = randseq(6,'alphabet','amino');
%       traci.route.add(Seq1,edg);
%       Seq = randseq(9);
%       traci.vehicle.add(Seq,Seq1);
%    end
%------------------------------------------------------------------------
%-------Calculate travel time form veh(1) to park and from park to dest-----
    vehInNet{3,1}=0;
    testveh = vehicles{1};
      %Get the position of vehicle j
    fprintf('%s\n',vehicles{1});
   vehInNet{2,1}=vehicles{1};
   fprintf(' Uno \n');
   tempV{3,1}=vehicles{1};
    v=testveh;
  %-----Travel time from veh to park based to edges travel time of veh route-----------  
   for i=1:parknb 
    traveltime=0;
    pedg=ParkEdgeID{i};
    %%%%%%%%%%%%%
    traveltime=InitTravTime(traveltime,pedg,i,v,parknb);
    %%%%%%%%%%%%%
    ConvtimeV(1,i)= traveltime;
   end
%-----Calculate walking time from park to dest------------------------
   for i=1:parknb
      x=DF(1,1); y=DF(1,2);
      z=PP(i,1); w=PP(i,2);
      distance2D = traci.simulation.getDistance2D(x, y, z, w);
      PFD(1,i) = distance2D;
      ConvtimeP(1,i)= (distance2D*1.8)*1000;
   end
%----Cost Matrix (Dist) sum of ConvtimeV(travel cost veh/park) + ConvtimeP(travel cost park/dist)
     ind=1;
     tempV{1,ind}=vehicles{1};
     tempV{2,ind}=0;
 %Sum the final distance and Get the Min for each row  %%%%%%%%
 Dist=ConvtimeP+ConvtimeV;
[min1, sub1] = min(Dist, [], 2);  % Gets the min element of each row
rowVec = [1:size(Dist,1)]';       % A column vector of row numbers to match sub1
ind1 = sub2ind(size(Dist), rowVec, sub1);
newroad{1}=OldRoad(sub1,1,ParkEdgeID,v,parknb);
  end

%--Save only the new entry vehicles and not all vehicles of the Net--------
  for j=1:length(NewvehInNet)
      siz=length(vehInNet{1,j});
      if  isequal(vehInNet{3,j},step) && ~ismember(vehInNet{1,j},traci.simulation.getEndingTeleportIDList())%siz<8
          ind=ind+1;
          tempV{1,ind}= vehInNet{1,j};
          tempV{2,ind}=vehInNet{3,j};     
      end
  end
%-------------------------------------------------------------------- 
%------------------------GET VEHICLES Roads & CALCULATE TRAVEL TIMEs-------------------------        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 [high,weigh]=size(tempV);
 len=weigh;
for j=1:len
%     if A(j)~=2 && ismember(v,vehicles)&&traci.vehicle.isStopped(v)
%         A(j)=2;
%     end
  if ismember(tempV{1,j},vehicles) && tempV{2,j}==step
        testveh = tempV{1,j};
        maxspeed(j)= traci.vehicle.getMaxSpeed(tempV{1,j});
      fprintf('%s\n',tempV{1,j});
      tempV{3,j}=tempV{1,j};
    %Get vehicle edge ID
%    EdgeID = traci.vehicle.getRoadID(tempV{1,j});
    %Travel time vehicle To parking (distance&time)------------------------
    v=tempV{1,j};
    for i=1:parknb
    pedg=ParkEdgeID{i};
    traveltime=0;
    %%%%%%%%%%%%%
    traveltime=InitTravTime(traveltime,pedg,i,v,parknb);
    %%%%%%%%%%%%%
   % ConvtimeV(j,i)= distance2D/maxspeed(j);
    ConvtimeV(j,i)= traveltime;
     end
%--------------------------------------------------------------------------
     %Walking time parking To final destination
     for i=1:parknb
      x=DF(j,1); y=DF(j,2);
      z=PP(i,1); w=PP(i,2);
      distance2D = traci.simulation.getDistance2D(x, y, z, w);
      PFD(j,i) = distance2D;
      ConvtimeP(j,i)= (distance2D*1.8)*1000;
     % Ptraveltime(j,i)= distance2D/1.8;
     end
%----------------------------------------------------------------------------------       
%Final Cost Matrix ( ConvtimeP(j,i)+ ConvtimeV(j,i)) Get the Min for each row  %%%%%%%%
 Dist=ConvtimeP+ConvtimeV;
[min1, sub1] = min(Dist, [], 2);  % Gets the min element of each row
rowVec = [1:size(Dist,1)]';       % A column vector of row numbers to match sub1
ind1 = sub2ind(size(Dist), rowVec, sub1);  % Gets indices of matrix A where mins are
%%%%%%%%
newroad{j}=OldRoad(sub1,j,ParkEdgeID,v,parknb);
%             if sub1(j)==1
%                 traci.vehicle.changeTarget(v,'b');
%                 newroad{j}=traci.vehicle.getRoute(v);
%             elseif sub1(j)==2
%                 traci.vehicle.changeTarget(v,'c');
%                 newroad{j}=traci.vehicle.getRoute(v);
%             else
%                 traci.vehicle.changeTarget(v,'-d');
%                 newroad{j}=traci.vehicle.getRoute(v);
%             end
     end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------Get the number of new entry vehicles
if traci.simulation.getDepartedNumber()~=0
    departVeh=traci.simulation.getDepartedIDList();
receve(1,q)=step;
receve(2,q)=traci.simulation.getDepartedNumber();
vehinsim=vehinsim+traci.simulation.getDepartedNumber();
q=q+1;
lastep=step;
end
if vehinsim==541 && ((lastep+1000)-step)==0
stop=1;
end
EstAv=GetFreeSpace(EstAv,cap,step,parknb);
%Rythme de prise de decision en foncion de nb de veh-----------------------
% if vehinsim-nnz(A)>50 && istep==50
%     istep=1;
%     rythm=3;
% elseif istep==50
%     istep=1;
%     rythm=5;
% end
%------Av Probability------------------------------------------------------
if traci.simulation.getDepartedNumber()~=0%stop==0 && mod(step,5)==0%(vehinsim<=41 && mod(step,2)==0) %receve(1,q)==step; %
    cmpt=cmpt+1;

%   vacsp1(cmpt) = nnz(EstAv1);
%     vacsp2(cmpt) = nnz(EstAv2);
%     vacsp3 (cmpt) = nnz(EstAv3);
%    vacsp1( :, all(~ vacsp1,1) ) = [];
%    vacsp2( :, all(~ vacsp2,1) ) = [];
%    vacsp3( :, all(~ vacsp3,1) ) = [];
%-----------------------------Get parking availibility
k = find(EstAv{1}==0,1);
% for j=1:7
% Avail(j)=cap(j)-occ(j);
% end

 [ParkAv]=CalcAvFixmu(cap,EstAv,ConvtimeV,parknb);

for j=1:parknb
Avail(j)=cap(j)-occ(j);
rec(cmpt,j)=Avail(j);
record(cmpt,j)=1-ParkAv(j);%cap(j)-nnz(EstAv{j});%
%----
end
%-------------------------------
%--------------------------------
%--------------------------VEHICLES ASSIGNMENT (SET STOP TO PARKING)-----------------------        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for j=1:len

   for c=1: traci.simulation.getDepartedNumber()
       if strcmp(tempV{1,j},departVeh{1,c})
        %v=tempV{1,j};
        departVeh{2,c}=j;
       end
    %s=St(j);
   end
end
%Rerouting function based to waitingtime
%  RerouteWaitime()
%----------------------------
for c=1: traci.simulation.getDepartedNumber()
    v=departVeh{1,c};
    j= departVeh{2,c};
%if ismember(v,vehicles)&&traci.vehicle.isStopped(v)
      %  A(j)=2;
 %else
  for i=1:parknb
%       if ismember(v,vehicles)
%       pedg=ParkEdgeID{i};
%       vedg=traci.vehicle.getRoadID(v);
%       end
  if ((sub1(j)==i))&&(A(j)==0)&&ismember(v,vehicles)%&&~strcmp(pedg,vedg)
    pl=sub1(j);
    [mu,ParkAv,Av]=CalcAv(cap,EstAv,ConvtimeV,pl,step,j,parknb);
    sp1=sp1+1;
    re1(sp1)=1-ParkAv(i);
    s=1/mu(i)*10000;
     pedg=ParkEdgeID{i};
    if 1-ParkAv(i)>0%cap(i)-nnz(EstAv{i})>0%
     
    traci.vehicle.changeTarget(v,pedg); %set the road
    newroute{j}=traci.vehicle.getRoute(v); %get the new road
    road=newroute{j};
    traci.vehicle.setStop(v,num2str(i),1,0,s,65);
      A(j)=1;
     olroad=newroad{j};
     costolroad=min1(j,1);
     wlktime=ConvtimeP(j,i);
     min2(j)=PathTravTime(road,olroad,costolroad);

     tempV{4,j}=sub1(j);
     e1 = find(EstAv{i}==0,1);
%      pt(i)=pt(i)+1;
%      e1=pt(i);
     EstAv{i}(e1)=Av;
  
    else
        for l=1:parknb
        nbVacSp(l)= nnz(~EstAv{l});
        end
        [val, idx] = max(nbVacSp);
        sub1(j)=idx;
        l=idx;
        %j=j-1;
                pedg=ParkEdgeID{l};
            traci.vehicle.changeTarget(v,pedg); %set the road
            newroute{j}=traci.vehicle.getRoute(v); %get the new road
            road=newroute{j};
            traci.vehicle.setStop(v,num2str(l),1,0,s,65);
              A(j)=1;
             olroad=newroad{j};
             costolroad=min1(j,1);
             wlktime=ConvtimeP(j,i);
             min2(j)=PathTravTime(road,olroad,costolroad);

             tempV{4,j}=sub1(j);
            e1 = find(EstAv{l}==0,1);
            %      pt(i)=pt(i)+1;
            %      e1=pt(i);
                 EstAv{l}(e1)=Av;
     end
   end
   end

 %end

end
%-------------------------------------------------------------------------
end 
step = step + 1;
istep= istep+1;

end
%---Calculate the fitness of the final solution----------------------------
Fitness1= sum(min1);
fprintf('Fitness1: %d\n',Fitness1);
Fitness2= sum(min2);
fprintf('Fitness2: %d\n',Fitness2);
%---Plot Result-----------------------------------------------------------

nnp=nnz(~A);

%    record1( :, all(~record1,1) ) = [];
%    record2( :, all(~record2,1) ) = [];
%    record3( :, all(~record3,1) ) = [];
%   r1( :, all(~r1,1) ) = [];
%   r2( :, all(~r2,1) ) = [];
%   r3( :, all(~r3,1) ) = [];
%  re1( :, all(~re1,1) ) = [];
%  re2( :, all(~re2,1) ) = [];
%  re3( :, all(~re3,1) ) = [];
%plot(A(:,1), A(:,2:end)) % first column is the x-values
 subplot(721);
 bar(record(:,1));
 subplot(722);
 bar(rec(:,1));
  subplot(723);
 bar(record(:,2));
   subplot(724);
 bar(rec(:,2));
  subplot(725);
 bar(record(:,3));
   subplot(726);
 bar(rec(:,3));
 
  subplot(727);
 bar(record(:,4));
 subplot(728);
 bar(rec(:,4));
  subplot(729);
 bar(record(:,5));
   subplot(7,2,10);
 bar(rec(:,5));
  subplot(7,2,11);
 bar(record(:,6));
   subplot(7,2,12);
 bar(rec(:,6));
   subplot(7,2,13);
 bar(record(:,7));
   subplot(7,2,14);
 bar(rec(:,7));
%--------------end---------------------------------------------------------
traci.close();
fprintf('SUMO version: %s\nTraCI version: %d\n',sumoVersion,traciVersion);