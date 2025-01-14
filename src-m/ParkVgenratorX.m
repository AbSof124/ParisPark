%% SUMO Park TEST
% THIS FILE AIMS TO TEST THE Parking PROTOCOL FOR SUMO.


clear
close all
clc

%% MAIN
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generates New Entry Based Probability
%P = poisspdf(X,y);
V = {'v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12','v13','v14','v15','v16','v17','v18','v19','v20','v21','v22','v23','v24','v25','v26','v27','v28','v29','v30','v31','v32','v33','v34','v35','v36','v37','v38','v39','v40'} ;
%rslt{1}={'v1'};
x=40;
%veh= randsample(V,4);
i=1;
%%%%%%%%
%Delete
%Next Time %%%%%%%%%
% mu=5*1/9;
% std=5*0.1;
% initocc=normrnd(mu,std);
% for q=1:10
% random_sample2 = exprnd(0.06);
% %plot(random_sample2)
% nextime(q)=-log(random_sample2)/0.06;
% cfd(q)=1-exp(-0.012*q);
% nb=randi(20);
% mu=5*10/nb;
% x=normrnd(mu,std);
% Newocc(q)=0.9*initocc+(1-0.9)*x;
% initocc=Newocc(q);
% end
% moy=sum(nextime)/100;

%plot(Newocc)
%%%%%%%%%%%%%%
%%%%%%%%%%
while x~=0
mu=1/exprnd(51);
lamda=0.7*40*mu;
if round(lamda)<=x && round(lamda)~=0
k=round(lamda);
r(i)=k;
% mean=x/2;
% if poissrnd(mean,1)<=x
% k = poissrnd(mean,1);
% %veh = randsample(V,k,false,samples);
veh= randsample(V,k);
V=setdiff(V,veh);
x=x-k;
rslt{1,i}=veh;
rslt{2,i}=lamda;
i=i+1;

end
end
plot(r);

eof=length(rslt);
fileID = fopen('C:\Users\SOFIENE\Downloads\SUMO\parking\test.rouX.xml','w');
%fprintf(fileID,'%6s %12s\r\n','x','exp(x)');
fprintf(fileID,'%6s \r\n','<?xml version="1.0" encoding="UTF-8"?>');
fprintf(fileID,'%6s \r\n','<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route1" edges="','-27282352#1','-27282352#0','517419309','23792302#0','23792302#1','23792302#2','-23792272#1','-23792272#0','584181258','23714295#0','23714295#1','23714295#2','23714307','584181253','405494696#0','405494696#1','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route2" edges="','-187816766#1','-187816766#0','27282359#6', '27282359#7','27282359#8','542026940','23714287#0','23714287#1','23714286#0', '23714286#1','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route3" edges="','23714280#0','23714280#1','23714287#0', '23714287#1','23714286#0','23714286#1','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route4" edges="','213159004','213159005#0','213159005#1','213159005#2','213159005#3','213159005#4','468478393#1','517419311','517419313','517419312','-405494696#2','-405494696#1','-405494696#0','-584181253','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route5" edges="','42138432#0','42138432#1','42138432#2','42138432#3','42138432#4','42138432#5','42138432#6','42138432#7','42138432#8','42138432#9','-23912692#1','-23912692#0','219899010','468478392','468478391','-260136572#4','-260136572#3','-260136572#2','-260136572#1','-260136572#0','-23714284','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route6" edges="','-194463378#2','-194463378#1','-194463378#0','23289033#0','23289033#1','517419314','517419316','232805394','232805393','232805395#0','232805395#1','517419313','517419312','-405494696#2','-405494696#1','-405494696#0','"/>');
fprintf(fileID,'%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%13s%1s \r\n','<route id="route7" edges="','261959646#0','261959646#1','261959646#2','129423851#2','129423851#3','517419306','517419309','23792302#0','189253789#0','189253789#1','-23714318#2','-23714318#1','"/>');

periode=0;
for i=1:eof
    temp=rslt{1,i};
    for j=1:length(temp)
    fprintf(fileID,'%6s%s%6s%3d%s%d%s \n','<vehicle id="',temp{j},'" depart="',periode,'" route="route', randi(7),'">');
    fprintf(fileID,'%6s \r\n','<param key="has.rerouting.device" value="true"/>');
	fprintf(fileID,'%6s \r\n','</vehicle>');
    end
    periode=periode+round(exp(1/rslt{2,i}))*10;
end
%temp=rslt{1,1};
fprintf(fileID,'%6s \r\n','</routes>');

fclose(fileID);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% HERE, WE START THE SUMO SIMULATOR AND INITIALIZE IT, ACCORDING TO
% THis EXAMPLE 

system('sumo-gui -c C:\Users\SOFIENE\Downloads\SUMO\parking\esiee_roadside_parking.sumocfg&');

[traciVersion,sumoVersion] = traci.init();

step = 1;
%for i=1:length(steps)
%A= [1 2 3];
%Preparing Inputs
DF=[691.00 727.04;821.32 679.85;849.01 583.68;844.58 668.70;665.79 683.60;814.87 685.29;736.12 711.66;675.67 701.40;720.73 638.77;886.67 754.89;937.95 692.25;1041.98 666.61;900.16 559.80;910.42 481.97;845.62 383.73;935.26 537.32;667.10 518.26;855.92 645.03;870.10 466.40;760.18 475.71];
PP=[799.85 523.64;955.80 628.81;774.87 702.15];
A= zeros(1,20);
%tempV=cell(3,15);
maxspeed= zeros(1,20);
vehInNet=cell(3,20);
vehInNet(2,:) = {0};
vehInNet(3,:) = {0};
vehP= zeros(1,20);
test=0;
St=[10000,50000,15000,20000,30000,10000,50000,15000,20000,30000,40000,20000,10000,10000,10000,20000,20000,20000,20000,20000];
%matVP=[685.84 724.25;1050.68 595.61;685.84 724.25;1050.68 595.61;685.84 724.25;1039.68 599.80;725.11 299.98;701.78 227.70;743.98 373.79;762.86 418.33;685.84 724.25;657.70 96.66;685.84 724.25;657.70 96.66;742.52 382.06];
NewvehInNet=vehInNet;
x=0;
ParkEdgeID={'b','c','-d'};
%edges={'a' 'b';'-a','a','b';'f','e', 'b';'-b', '-a', 'a', 'b';'d', '-b', '-a', 'a', 'b', 'c';'-d','e' ,'b' ,'c';'f', 'e', 'b';'c', '-d'};
while traci.simulation.getMinExpectedNumber() > 0
%for s=1:115
  % Here, we demonstrate how to use the simulationStep command using an
  % argument. In this case, the simulation is performed each 5 seconds,
  % note the behavior when you increase the delay in the gui
%  traci.simulationStep(5000*step);
%  pause(1);
  traci.simulationStep();
 % programPointer = min(programPointer+1, length(PROGRAM));
    
  % SHOW THE VEHICLES IDS INSIDE THE NETWORK
 
  %if step == 1
      %Get vehicles ID
    
    vehicles = traci.vehicle.getIDList();
   % loadedNumber = traci.simulation.getLoadedNumber();
    %loadedIDList = traci.simulation.getLoadedIDList();
      %Get distance between two coordinates x y
  %   vehPosition1 = traci.vehicle.getPosition('6');
   %   x1= vehPosition1(1);
    %  y1= vehPosition1 (2);
%distance2D = traci.vehicle.getDrivingDistance2D('6', 781.71, 698.55);
%fprintf('%d\n',distance2D);
%distance2D1 = traci.simulation.getDistance2D(x1,y1,955.77,628.34,0,1);
%fprintf('%d\n',distance2D1);
   
%Stop vehicle at a parking area (block the traffic)
%traci.vehicle.setStop('v7','ParkAreaB',1,0,40000,64);
%ol = traci.vehicle.isStopped('v7');
%Stop vehicle at a parking area (need a resume function otherwise the vehicel will stack at the parking area)
%traci.vehicle.setStop('7','ParkAreaB',1,0,64,40000);
% traci.vehicle.setParkingAreaStop('v0','ParkAreaB',50.0,1,80.0);!! not
% working

%vehDrivingDistance = traci.vehicle.getDrivingDistance('7','b',140);
%fprintf('%d\n',vehDrivingDistance);
%vehDrivingDistance1 = traci.vehicle.getDrivingDistance('7','-d',50);
%fprintf('%d\n',vehDrivingDistance1);
%Get vehcile starting to park at step t
% v =  traci.simulation.getParkingStartingVehiclesIDList();
 %Get initiale capacity for each parking area
  cap1 =str2num( traci.simulation.getParameter('ParkAreaB','parkingArea.capacity'));
%  fprintf('%s\n',cap1);
  cap2 =str2num( traci.simulation.getParameter('ParkAreaC','parkingArea.capacity'));
 % fprintf('%s\n',cap2);
  cap3 =str2num( traci.simulation.getParameter('1','parkingArea.capacity'));
  %fprintf('%s\n',cap3);
  %Get parking occupancy for each park area
  occ1 =str2num( traci.simulation.getParameter('ParkAreaB','parkingArea.occupancy'));
 % fprintf('%s\n',occ1);
  occ2 =str2num( traci.simulation.getParameter('ParkAreaC','parkingArea.occupancy'));
  %fprintf('%s\n',occ2);
  occ3 =str2num( traci.simulation.getParameter('1','parkingArea.occupancy'));
 % fprintf('%s\n',occ3);
 %Get park starting veh numb at step t
 % vpn = traci.simulation.getParkingStartingVehiclesNumber();
 % fprintf('Veh PArk Numb!! %d\n',vpn);
 %Get park end veh numb at step t
 % pev = traci.simulation.getParkingStartingVehiclesID();
  currentTime = traci.simulation.getCurrentTime ();
%currentTime2 = traci.simulation.getTime ();

  %GET NEW ENTRY VEHICLES       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   for j=1:length(vehicles)
   fi=find(strcmp(vehInNet,vehicles{j}));
       if find(strcmp(vehInNet,vehicles{j})) % find(ismember(vehInNet{1,j},vehicles)) %any(strcmp(vehInNet,vehicles{j})) %exist(vehInNet{1,j},vehicles) 
          vehInNet{3,j}= -1;
         % vehInNet{1,j}= vehicles{j};
          NewvehInNet{1,j}=vehicles{j};
         % NewvehInNet{3,j}=step-1;
       else %if vehInNet{3,j}==0
           vehInNet{3,j}=step;
       % vehInNet{1,j}= vehicles{j};
        NewvehInNet{1,j}=vehicles{j};
        NewvehInNet{3,j}=step;
       end
      
      
   end
   for j=1:length(vehicles)
       vehInNet{1,j}=NewvehInNet{1,j};
       vehInNet{3,j}=NewvehInNet{3,j};
   end
   %INITIALIZATION        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  if test ==0
   %ADD Vehicle Flow
   edg={'d','-b','-a','a','b','c'};
   for i=1:30
      %edg=randsample(edges,1); 
      Seq1 = randseq(5);
      traci.route.add(Seq1,edg);
      Seq = randseq(9);
      traci.vehicle.add(Seq,Seq1);
   end
   %%%%%%%%%%%%%%%%%%%
      vehInNet{3,1}=0;
   %if (vehInNet{2,1}==0)
       test=1;
    testveh = vehicles{1};
      %Get the position of vehicle j
    vehPosition = traci.vehicle.getPosition(testveh);
   matVP=[vehPosition(1) vehPosition(2)];
   maxspeed= traci.vehicle.getMaxSpeed(vehicles{1});
    fprintf('%s\n',vehicles{1});
      fprintf('%d\n',vehPosition);
   vehInNet{2,1}=vehicles{1};
   fprintf(' Uno \n');
   tempV{3,1}=vehicles{1};
   EdgeID = traci.vehicle.getRoadID(testveh);
    v=testveh;
   for i=1:3
        traveltime=0;
    x=matVP(1,1); y=matVP(1,2);
    z=PP(i,1); w=PP(i,2);
    %distance2D = traci.simulation.getDistance2D(x, y, z, w,0,1);
    pedg=ParkEdgeID{i};
    distance2D = traci.simulation.getDistanceRoad(EdgeID,0,pedg,0,1);
    %%%%%%%%%%%%%
    if i==1
    traci.vehicle.changeTarget(v,'b');
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==2
    traci.vehicle.changeTarget(v,'c');
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    else
    traci.vehicle.changeTarget(v,'-d');
    newroad{1}=traci.vehicle.getRoute(v);
    road=newroad{1,1};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    end
    %%%%%%%%%%%%%
    VPD(1,i) = distance2D;
    %ConvtimeV(1,i)= distance2D/maxspeed(1);
    ConvtimeV(1,i)= traveltime;
    Vtraveltime=ConvtimeV;
   end
   for i=1:3
      x=DF(1,1); y=DF(1,2);
      z=PP(i,1); w=PP(i,2);
      distance2D = traci.simulation.getDistance2D(x, y, z, w);
      PFD(1,i) = distance2D;
      ConvtimeP(1,i)= distance2D/1.8;
      Ptraveltime=ConvtimeP;
   end
     ind=1;
     tempV{1,ind}=vehInNet{1,1};
     tempV{2,ind}=vehInNet{3,1};
     %ind=ind-1;
    % A Revoir!!!!!!!!!!!!!!!!!! 
    %Sum the final distance (VPD+PFD) and Get the Min for each row  %%%%%%%%
 Dist=ConvtimeP+ConvtimeV;
[min1, sub1] = min(Dist, [], 2);  % Gets the min element of each row
rowVec = [1:size(Dist,1)]';       % A column vector of row numbers to match sub1
ind1 = sub2ind(size(Dist), rowVec, sub1);
  end

  for j=1:length(NewvehInNet)
      siz=length(vehInNet{1,j});
      if  isequal(vehInNet{3,j},step) && siz<8
          ind=ind+1;
          tempV{1,ind}= vehInNet{1,j};
          tempV{2,ind}=vehInNet{3,j};     
      end
  end
  
  %GET VEHICLES POSITION & CALCULATE TRAVEL TIMEs        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 [high,weigh]=size(tempV);
 len=weigh;
for j=1:len
       % fprintf('%d !!\n',j);
        if (strcmp(tempV{3,j},tempV{1,j})~=1)
            vx=char(tempV{1,j});
             maxspeed(j)= traci.vehicle.getMaxSpeed(tempV{1,j});
        end
  if ismember(tempV{1,j},vehicles) && tempV{2,j}==step
        %fprintf('%d !\n',j);
           testveh = tempV{1,j};
        maxspeed(j)= traci.vehicle.getMaxSpeed(tempV{1,j});
      %Get the position of vehicle j
      vehPosition = traci.vehicle.getPosition(testveh);
     new_row =[vehPosition(1) vehPosition(2)] ; % new row with values x & y
     matVP= [matVP;new_row]; 
     %   vehPosition = traci.vehicle.getGeoPosition(testveh);!!not work
      fprintf('%s\n',tempV{1,j});
      fprintf('%d\n',vehPosition);
      tempV{3,j}=tempV{1,j};
      % end
       % end
    %end
  EdgeID = traci.vehicle.getRoadID(tempV{1,j});
    %matrix vehicle To parking distance
    %for j=1:length(vehicles)
    v=tempV{1,j};
     for i=1:3
         traveltime=0;
    x=matVP(j,1); y=matVP(j,2);
    z=PP(i,1); w=PP(i,2);
    %distance2D = traci.simulation.getDistance2D(x, y, z, w,0,1);
     pedg=ParkEdgeID{i};
    distance2D = traci.simulation.getDistanceRoad(EdgeID,0,pedg,0,1);
     %%%%%%%%%%%%%
    if i==1
    traci.vehicle.changeTarget(v,'b');
    newroad{j}=traci.vehicle.getRoute(v);
    road=newroad{1,j};
    for k=1:length(road)      
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    elseif i==2
    traci.vehicle.changeTarget(v,'c');
    newroad{j}=traci.vehicle.getRoute(v);
    road=newroad{1,j};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    else
    traci.vehicle.changeTarget(v,'-d');
    newroad{j}=traci.vehicle.getRoute(v);
    road=newroad{1,j};
    for k=1:length(road)
        traveltime = traveltime+traci.edge.getTraveltime(road{k});
    end
    end
    %%%%%%%%%%%%%
    VPD(j,i) = distance2D;
    %ConvtimeV(j,i)= distance2D/maxspeed(j);
    ConvtimeV(j,i)= traveltime;
   % Vtraveltime(j,i)=distance2D/maxspeed(j);
     end
    Vtraveltime=[Vtraveltime;ConvtimeV];
    %end
   % end
     %matrix parking To final destination distance
   %  for j=1:length(vehicles)
     for i=1:3
      x=DF(j,1); y=DF(j,2);
      z=PP(i,1); w=PP(i,2);
      distance2D = traci.simulation.getDistance2D(x, y, z, w);
      PFD(j,i) = distance2D;
      ConvtimeP(j,i)= distance2D/1.8;
     % Ptraveltime(j,i)= distance2D/1.8;
     end
       Ptraveltime=[Ptraveltime;ConvtimeP];
       
 %Sum the final distance (VPD+PFD) and Get the Min for each row  %%%%%%%%
 Dist=ConvtimeP+ConvtimeV;
[min1, sub1] = min(Dist, [], 2);  % Gets the min element of each row
rowVec = [1:size(Dist,1)]';       % A column vector of row numbers to match sub1
ind1 = sub2ind(size(Dist), rowVec, sub1);  % Gets indices of matrix A where mins are
     end
end

%VEHICLES ASSIGNMENT (SET STOP TO PARKING)        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   
%x=str2num(cap1);
%Assignment of vehicle to parking
Avail1=cap1-occ1;
Avail2=cap2-occ2;
Avail3=cap3-occ3;
for j=1:len
    v=tempV{1,j};
    s=St(j);
    %fprintf('Veh %s\n',v);
 if ((sub1(j)==1)&&(Avail1>0) &&(A(j)==0))%(vehInNet{3,j}==0)&&
    % traci.vehicle.moveToXY(v,'-d',1,PP(3,1)+1, PP(3,2)+1,0,2);
    traci.vehicle.changeTarget(v,'b');
    newroute{j}=traci.vehicle.getRoute(v);
%    reroutTime{j}=traci.vehicle.rerouteTraveltime(v);
     traci.vehicle.setStop(v,'ParkAreaB',1,0,s,65);
     A(j)=1;
     vehInNet{3,j}=1;
     %fprintf('%d\n',step);
     %fprintf('Veh %s\n',v);
  elseif ((sub1(j)==2)&&(Avail2>0)&&(A(j)==0))%(vehInNet{3,j}==0)&& (A(j)==0))
    % traci.vehicle.moveToXY(v,'c',1,PP(2,1)+1, PP(2,2)+1,0,2);
    traci.vehicle.changeTarget(v,'c');
    newroute{j}=traci.vehicle.getRoute(v);
  %  reroutTime{j}=traci.vehicle.rerouteTraveltime(v);
    traci.vehicle.setStop(v,'ParkAreaC',1,0,s,65);
    A(j)=1;
    vehInNet{3,j}=1;
     %fprintf('%d\n',sub1(j));
     %fprintf('Veh %s\n',v);
  elseif ((sub1(j)==3)&&(Avail3>0)&&(A(j)==0))%(vehInNet{3,j}==0)&& (A(j)==0))
    % traci.vehicle.moveToXY(v,'b',1,PP(1,1)+1, PP(1,2)+1,0,2);
    traci.vehicle.changeTarget(v,'-d');
    newroute{j}=traci.vehicle.getRoute(v);
   % reroutTime{j}=traci.vehicle.rerouteTraveltime(v);
     traci.vehicle.setStop(v,'1',1,0,s,65);
      A(j)=1;
       vehInNet{3,j}=1;
     % fprintf('%d\n',sub1(j));
      %fprintf('Veh %s\n',v);
 end
% bool{j} = traci.vehicle.isStopped(v);
 
% if traci.vehicle.isStopped(v)
 %    st{j}= step;
 %end
 
end
%end
%  end
 
   % if step == 45
    %    traci.vehicle.resume(v);
    %end
    
 
     
%  steps(i) = i;
  

    step = step + 1;
end
Fitness= sum(min1);
fprintf('Fitness: %d\n',Fitness);
%plot (min1)
%G= graph(dist);
%plot (G)
%end
traci.close();
fprintf('SUMO version: %s\nTraCI version: %d\n',sumoVersion,traciVersion);