%% SUMO TRACI TEST
% THIS FILE AIMS TO TEST THE TRACI PROTOCOL FOR SUMO.

%   Copyright 2016 Universidad Nacional de Colombia,
%   Politecnico Jaime Isaza Cadavid.
%   Authors: Andres Acosta, Jairo Espinosa, Jorge Espinosa.
%   $Id: traci_test.m 31 2016-09-28 15:16:56Z afacostag $

clear
close all
clc

%% MAIN
% HERE, WE START THE SUMO SIMULATOR AND INITIALIZE IT, ACCORDING TO
% THE EXAMPLE FOUND IN http://sumo-sim.org/userdoc/Tutorials/TraCI4Traffic_Lights.html

system('sumo -c C:\Users\SOFIENE\Downloads\SUMO\tuto_par2\angled_roadside_parking.sumocfg&');

[traciVersion,sumoVersion] = traci.init();
vehP= zeros(1,15);
vehInNet=cell(2,15);
vehInNet(2,:) = {0};
step = 1;
subscribedToTestVeh=0;
%for i=1:length(steps)
while traci.simulation.getMinExpectedNumber() > 0
  % Here, we demonstrate how to use the simulationStep command using an
  % argument. In this case, the simulation is performed each 5 seconds,
  % note the behavior when you increase the delay in the gui
%  traci.simulationStep(5000*step);
%  pause(1);
  traci.simulationStep();
 % programPointer = min(programPointer+1, length(PROGRAM));
    
  % Get the number of vehicles that passed through the induction loop in
  % the last simulation step
  %numPriorityVehicles = traci.inductionloop.getLastStepVehicleNumber('0');
    
  % SHOW THE VEHICLES IDS INSIDE THE NETWORK
 % if step == 30
    vehicles = traci.vehicle.getIDList();
  % cap = traci.getParameter("ParkAreaD","parkingArea.capacity");
  %currentTime = traci.simulation.getParkingStartingVehiclesNumber();
  %fprintf('%d\n', currentTime);
  %park = traci.simulation.getTime();
  
    if step ==20
        c=vehicles;
    end
    if step==30 
       % if strcmp(c,vehicles
       %c2=vehicles;
     c2 = setdiff(c,vehicles);
    end
   for j=1:length(vehicles)
      vehInNet{1,j}= vehicles{j};
   end
  if step ==1
   if (vehInNet{2,1}==0)
       
    testveh = vehicles{1};
      %Get the position of vehicle j
    vehPosition = traci.vehicle.getPosition(testveh);
   matVP=[vehPosition(1) vehPosition(2)];
   maxspeed= traci.vehicle.getMaxSpeed(vehicles{1});
   vehInNet{2,1}=1;
   end
  end
    for j=1:length(vehicles)
       if ismember(vehInNet{1,j},vehicles) && (vehInNet{2,j}==0)&& strcmp(vehInNet{1,j},vehicles{j})==1
        testveh = vehicles{j};
        maxspeed(j)= traci.vehicle.getMaxSpeed(vehicles{j});
      %Get the position of vehicle j
      vehPosition = traci.vehicle.getPosition(testveh);
     new_row =[vehPosition(1) vehPosition(2)] ; % new row with values x & y
     matVP= [matVP;new_row]; 
     %   vehPosition = traci.vehicle.getGeoPosition(testveh);!!not work
      fprintf('%s\n',vehicles{j});
      fprintf('%d\n',vehPosition);
      vehInNet{2,j}=1;
       end
    end
  
    % Subscribe to the vehicle with the id contained in the variable "testVehicle" 
	% when it is loaded in the network 
%     for j=1:length(vehicles)
%    if ismember(vehicles{j},vehicles) && (vehP(j)==0)
%        
%       fprintf('IDs of the vehicles in the simulation\n')
%        vehPosition = traci.vehicle.getPosition(vehicles{j});
%       step=traci.simulationStep();
%        fprintf('%s\n',vehicles{j});
%       fprintf('%s\n',vehPosition);
%       vehP(j)=1;
%       
%     end
%     end
%     for j=1:length(vehicles)
%         testveh = vehicles{j};
%       %  traci.vehicle.getSpeed(" testveh");
%         vehPosition = traci.vehicle.getPosition(testveh);
%         new_row =[vehPosition(1) vehPosition(2)] ;
%         vehP(j)=vehPosition;
%       fprintf('%s\n',vehicles{j});
%       fprintf('%s\n',vehPosition);
%     end
  %end
    
  % VEHICLE COMMANDS
%   if ismember(testVehicle, vehicles)      
%     if ~vehicleCommandsTested
     %% VEHICLE GET COMMANDS
%      vehSpeed = traci.vehicle.getSpeed(testVehicle)
%      vehSpeedWOTraci = traci.vehicle.getSpeedWithoutTraCI(testVehicle)
%      vehPosition = traci.vehicle.getPosition(testVehicle)
%      vehPosition3D = traci.vehicle.getPosition3D(testVehicle)
%      vehAngle = traci.vehicle.getAngle(testVehicle)
%      vehRoadID = traci.vehicle.getRoadID(testVehicle)
%      vehLaneID = traci.vehicle.getLaneID(testVehicle)
%      vehLaneIndex = traci.vehicle.getLaneIndex(testVehicle)
%      vehTypeID = traci.vehicle.getTypeID(testVehicle)
%      vehRouteID = traci.vehicle.getRouteID(testVehicle)
%      vehRouteIndex = traci.vehicle.getRouteIndex(testVehicle)
%      vehRoute = traci.vehicle.getRoute(testVehicle)
%      vehLanePos = traci.vehicle.getLanePosition(testVehicle)
%      vehColor = traci.vehicle.getColor(testVehicle)
%      vehCO2Emission = traci.vehicle.getCO2Emission(testVehicle)
%      vehCOEmission = traci.vehicle.getCOEmission(testVehicle)
%      vehPmxEmission = traci.vehicle.getPMxEmission(testVehicle)
%      vehNOxEmission = traci.vehicle.getNOxEmission(testVehicle)
%      vehFuelConsumption = traci.vehicle.getFuelConsumption(testVehicle)
%      vehNoiseEmission = traci.vehicle.getNoiseEmission(testVehicle)
%      vehElectricityConsumption = traci.vehicle.getElectricityConsumption(testVehicle)
%      vehPersonNumber = traci.vehicle.getPersonNumber(testVehicle)
%      vehAdaptedTraveltime = traci.vehicle.getAdaptedTraveltime(testVehicle,10,'1i')
%      vehEffort = traci.vehicle.getEffort(testVehicle,10,'1i')
%      vehValidRoute = traci.vehicle.isRouteValid(testVehicle)
%      vehSignals = traci.vehicle.getSignals(testVehicle)
%      vehLength = traci.vehicle.getLength(testVehicle)
%      vehMaxSpeed = traci.vehicle.getMaxSpeed(testVehicle)
%      vehLateralLanePos = traci.vehicle.getLateralLanePosition(testVehicle)
%      vehMaxSpeedLat = traci.vehicle.getMaxSpeedLat(testVehicle)
%      vehLatAlignment = traci.vehicle.getLateralAlignment(testVehicle)
%      vehMinGapLat = traci.vehicle.getMinGapLat(testVehicle)
%      vehAllowedSpeed = traci.vehicle.getAllowedSpeed(testVehicle)
%      vehClass = traci.vehicle.getVehicleClass(testVehicle)
%      vehSpeedFactor = traci.vehicle.getSpeedFactor(testVehicle)
%      vehSpeedDeviation = traci.vehicle.getSpeedDeviation(testVehicle)
%      vehEmissionClass = traci.vehicle.getEmissionClass(testVehicle)
%      vehWaitingTime = traci.vehicle.getWaitingTime(testVehicle)
%      vehAccumWaitingTime = traci.vehicle.getAccumulatedWaitingTime(testVehicle)
%      vehSpeedMode = traci.vehicle.getSpeedMode(testVehicle)
%      vehSlope = traci.vehicle.getSlope(testVehicle)
%      vehWidth = traci.vehicle.getWidth(testVehicle)
%      vehHeight = traci.vehicle.getHeight(testVehicle)
%      vehLine = traci.vehicle.getLine(testVehicle)
%      vehVia = traci.vehicle.getVia(testVehicle)
%      vehMinGap = traci.vehicle.getMinGap(testVehicle)
%      vehShapeClass = traci.vehicle.getShapeClass(testVehicle)
%      vehAccel = traci.vehicle.getAccel(testVehicle)
%      vehDecel = traci.vehicle.getDecel(testVehicle)
%      vehEmergencyDecel = traci.vehicle.getEmergencyDecel(testVehicle)
%      vehApparentDecel = traci.vehicle.getApparentDecel(testVehicle)
%      vehImperfection = traci.vehicle.getImperfection(testVehicle)
%      vehTau = traci.vehicle.getTau(testVehicle)
%      vehLeader = traci.vehicle.getLeader(testVehicle, 1)
%      vehNextTLS = traci.vehicle.getNextTLS(testVehicle)
%      vehBestLanes = traci.vehicle.getBestLanes(testVehicle)
%      vehDrivingDistance = traci.vehicle.getDrivingDistance(testVehicle,'2o',30)
%      vehDrivingDistance2D = traci.vehicle.getDrivingDistance2D(testVehicle,620,510)
%      vehDistance = traci.vehicle.getDistance(testVehicle)
%      vehStopState = traci.vehicle.getStopState(testVehicle)
     
%      
%      %% VEHICLE SET COMMANDS
%      traci.vehicle.add('myvehicle', 'down');
%      traci.gui.trackVehicle('View #0', 'myvehicle');
%      traci.vehicle.remove('myvehicle');
%      traci.vehicle.setMaxSpeed(testVehicle, 5);
% %     traci.vehicle.setStop(testVehicle, '1i', 50, 0, 40000);
%      traci.vehicle.changeLane(testVehicle, 0, 40000);
%      traci.vehicle.slowDown(testVehicle, 1, 180000);
%      traci.vehicle.changeTarget(testVehicle, '2o');
% %      traci.vehicle.setRouteID(testVehicle, 'down');
% %      traci.vehicle.setAdaptedTraveltime(testVehicle, 10000, 50000, '1i', 15000);
% %      traci.vehicle.setEffort(testVehicle, 10000, 50000, '1i', 0.125); %Not online
% %      traci.vehicle.rerouteTraveltime(testVehicle); %Not online
% %      traci.vehicle.rerouteEffort(testVehicle); %Not online
%      traci.vehicle.setSignals(testVehicle, 2);
%      traci.vehicle.setSpeed(testVehicle, 13.8999);
%      traci.vehicle.setSpeed(testVehicle, 12.0001);
%      traci.vehicle.setColor(testVehicle,[0 0 255 0]);
%      traci.vehicle.setLength(testVehicle, 10);
%      traci.vehicle.setVehicleClass(testVehicle, 'passenger');
%      traci.vehicle.setSpeedFactor(testVehicle, 0.6);
%      traci.vehicle.setEmissionClass(testVehicle, 'unknown');
%      traci.vehicle.setWidth(testVehicle, 3);
%      traci.vehicle.setMinGap(testVehicle, 10);
%      traci.vehicle.setShapeClass(testVehicle, '');
%      traci.vehicle.setAccel(testVehicle, 2);
%      traci.vehicle.setImperfection(testVehicle, 1);
%      traci.vehicle.setTau(testVehicle, 1);
%      traci.vehicle.moveToVTD('right_10','2o',0,608,509,0);
% %    traci.vehicle.moveTo(testVehicle,'1i_0',20);  % TODO
%        
%      if step == 100
%        testVehDecel = traci.vehicle.getDecel(testVehicle)
%        traci.vehicle.setDecel(testVehicle, 5);
%      end
%        
%      if step > 100
%        testVehDecel = traci.vehicle.getDecel(testVehicle)
%      end
       
%      vehicleCommandsTested = 1;
       
  % Subscribe to the vehicle with the id contained in the variable "testVehicle" 
	% when it is loaded in the network       
%    if ~subscribedToTestVeh
%      traci.vehicle.subscribe(testVehicle);
%      subscribedToTestVeh = 1;
%    end
%        
%    if ~contextSubsToTestVeh
%      traci.vehicle.subscribeContext(testVehicle,...
%        traci.constants.CMD_GET_VEHICLE_VARIABLE, 20,...
%        {'0x40', '0x42','0x43','0x5b','0x51','0x56','0x7a'});
%      traci.vehicle.subscribeContext(testVehicle,...
%        traci.constants.CMD_GET_VEHICLE_VARIABLE, 10,...
%        {traci.constants.VAR_WAITING_TIME});
%      contextSubsToTestVeh = 1;
%    end
%           
%    testVehicleRoad = char(traci.vehicle.getSubscriptionResults(testVehicle).get(...
%      traci.constants.VAR_ROAD_ID))
%    testVehiclePos = traci.vehicle.getSubscriptionResults(testVehicle).get(...
%      traci.constants.VAR_LANEPOSITION)
%        
%    if ~trackingTestVeh
%      traci.gui.trackVehicle('View #0', testVehicle);
%      trackingTestVeh = 1;
%    end
%  end
    
  %% GETSUBSCRIPTIONRESULTS COMMANDS: Note that you have to create the required detectors in the cross.det.xml file
%  if ~getSubscriptionResultsTested && step == 100
%    edge1iVehNumber = traci.edge.getSubscriptionResults('1i').get(...
%      traci.constants.LAST_STEP_VEHICLE_NUMBER)
%    edge1iVehIDs = char(traci.edge.getSubscriptionResults('1i').get(...
%      traci.constants.LAST_STEP_VEHICLE_ID_LIST))
%    offset = traci.gui.getSubscriptionResults('View #0').get(...
%      traci.constants.VAR_VIEW_OFFSET)
%    indloopVehNumber = traci.inductionloop.getSubscriptionResults('0').get(...
%      traci.constants.LAST_STEP_VEHICLE_NUMBER)
%    junctionPosition = traci.junction.getSubscriptionResults('0').get(...
%      traci.constants.VAR_POSITION)
%    lane1WaitingTime = traci.lane.getSubscriptionResults('1i_0').get(...
%      traci.constants.VAR_WAITING_TIME)
%    lane1JamLength = traci.lanearea.getSubscriptionResults('0').get(...
%      traci.constants.JAM_LENGTH_METERS)
%    meanSpeedLane2 = traci.multientryexit.getSubscriptionResults('0').get(...
%      traci.constants.LAST_STEP_MEAN_SPEED)
%    poiPosition = traci.poi.getSubscriptionResults('mypoi').get(traci.constants.VAR_POSITION);
%    polygonPosition = traci.polygon.getSubscriptionResults('mypolygon').get(...
%      traci.constants.VAR_SHAPE)
%    routeList = char(traci.route.getSubscriptionResults('down').get(...
%      traci.constants.ID_LIST))
%    departedVehicleIDs = char(traci.simulation.getSubscriptionResults().get(...
%      traci.constants.VAR_DEPARTED_VEHICLES_IDS))
%    tlsCurrentPhase = traci.trafficlights.getSubscriptionResults('0').get(...
%      (traci.constants.TL_RED_YELLOW_GREEN_STATE))
%    maxSpeedWE = traci.vehicletype.getSubscriptionResults('typeWE').get(...
%      traci.constants.VAR_MAXSPEED)
%    getSubscriptionResultsTested = 1;
%  end
    
    %% GET CONTEXT SUBSCRIPTION RESULTS COMMANDS
    % edge4i0ContextResults = traci.edge.getContextSubscriptionResults('4i');
    % occupancy4i0Handle1 = edge4i0ContextResults('4i_0');
    % occupancy4i0 = occupancy4i0Handle1(traci.constants.LAST_STEP_VEHICLE_NUMBER);
    % fprintf('%d\n',occupancy4i0);
    
    % loop0ContextResults = traci.inductionloop.getContextSubscriptionResults('0');
    % priorityVehiclesPassedHandle = loop0ContextResults('4i_0');
    % priorityVehiclesPassed = priorityVehiclesPassedHandle(traci.constants.LAST_STEP_VEHICLE_NUMBER);
    % fprintf('%d\n',priorityVehiclesPassed);
    
    % junctionContextResults = traci.junction.getContextSubscriptionResults('0');
    
    % laneContextSubscriptionResults = traci.lane.getContextSubscriptionResults('4i_0');
    % poiContextSubscriptionResults = traci.poi.getContextSubscriptionResults('mypoi');
    % polygonContextSubscriptionResults = traci.polygon.getContextSubscriptionResults('mypolygon');
%    if contextSubsToTestVeh
%        vehicleContextSubscriptionResults = traci.vehicle.getContextSubscriptionResults(testVehicle);
%        if ~strcmp(vehicleContextSubscriptionResults,'None')
%            testVehicleSubsResults = vehicleContextSubscriptionResults(testVehicle);
%            testVehicleWaitingTime = testVehicleSubsResults(traci.constants.VAR_WAITING_TIME)
%        end
%    end
    
	%% LANE AREA DETECTOR COMMANDS: Note that you have to create the detector in the cross.det.xml file
%  if ~laneareaCommandsTested
%    laneareaDetectorIDCount = traci.lanearea.getIDCount();
%    fprintf('Number of lanearea detectors in the simulation: %d\n',...
%      laneareaDetectorIDCount);
%    
%    JamLengthVehicle = traci.lanearea.getJamLengthVehicle('0');
%    fprintf('Jam lenght in vehicles in the lanearea detector 0: %d\n',...
%      JamLengthVehicle);
%    
%    JamLengthMeters = traci.lanearea.getJamLengthMeters('0');
%    fprintf('Jam lenght in meters in the lanearea detector 0: %d\n',...
%      JamLengthMeters);
%    
%    vehicleMeanSpeedlanearea0 = traci.lanearea.getLastStepMeanSpeed('0');
%    fprintf('Average speed in the lanearea detector 0: %d\n',...
%      vehicleMeanSpeedlanearea0);
%    
%    vehicleOccupancylanearea0 = traci.lanearea.getLastStepOccupancy('0');
%    fprintf('Occupancy in the lanearea detector 0 1i: %d\n',...
%      vehicleOccupancylanearea0);
%    
%    laneareaCommandsTested = 1;
%  end
	
 
    
  %% GUI COMMANDS
%  if ~guiCommandsTested
%    guizoom = traci.gui.getZoom()
%    offset = traci.gui.getOffset()
%    schema = traci.gui.getSchema()
%    boundary = traci.gui.getBoundary()
%    guiCommandsTested = 1;
%  end
    
  %% INDUCTION LOOP COMMANDS
%  if ~indLoopCommandsTested
%    loop0position = traci.inductionloop.getPosition('0');
%    loop0LaneID = traci.inductionloop.getLaneID('0')
%    loop0MeanSpeed = traci.inductionloop.getLastStepMeanSpeed('0')
%    loop0VehicleIDs = traci.inductionloop.getLastStepVehicleIDs('0')
%    loop0Occupancy = traci.inductionloop.getLastStepOccupancy('0')
%    loop0MeanLength = traci.inductionloop.getLastStepMeanLength('0')
%    loop0TimeSinceDetection = traci.inductionloop.getTimeSinceDetection('0')
%    indLoopCommandsTested = 1;
%  end
    
  %% JUNCTION COMMANDS
%  if ~junctionCommandsTested
%    junctionPosition = traci.junction.getPosition('0')
%    junctionCommandsTested = 1;
%  end
    
  %% LANE GET COMMANDS
%  if ~laneCommandsTested
%    lane1i0Length = traci.lane.getLength('1i_0')
%    lane1i0MaxSpeed = traci.lane.getMaxSpeed('1i_0')
%    lane1i0Width = traci.lane.getWidth('1i_0')
%    lane1i0AllowedVehicles = traci.lane.getAllowed('1i_0')
%    lane1i0DisallowedVehicles = traci.lane.getDisallowed('1i_0')
%    lane1i0LinkNumber = traci.lane.getLinkNumber('1i_0')
%    lane1i0Links = traci.lane.getLinks('1i_0')
%    lane1i0Shape = traci.lane.getShape('1i_0')
%    lane1i0EdgeID = traci.lane.getEdgeID('1i_0')
%    lane1i0CO2Emmision = traci.lane.getCO2Emission('1i_0')
%    lane1i0COEmmision = traci.lane.getCOEmission('1i_0')
%    lane1i0HCEmmision = traci.lane.getHCEmission('1i_0')
%    lane1i0PMxEmmision = traci.lane.getPMxEmission('1i_0')
%    lane1i0NOxEmmision = traci.lane.getNOxEmission('1i_0')
%    lane1i0FuelConsumption = traci.lane.getFuelConsumption('1i_0')
%    lane1i0NoiseEmission = traci.lane.getNoiseEmission('1i_0')
%    lane1i0MeanSpeed = traci.lane.getLastStepMeanSpeed('1i_0')
%    lane1i0Occupancy = traci.lane.getLastStepOccupancy('1i_0')
%    lane1i0MeanVehicleLength = traci.lane.getLastStepLength('1i_0')
%    lane1i0TravelTime = traci.lane.getTraveltime('1i_0')
%    lane1i0HalringNumber = traci.lane.getLastStepHaltingNumber('1i_0')
%    lane1i0VehicleIDs = traci.lane.getLastStepVehicleIDs('1i_0')
%    lane1i0HaltingNumber = traci.lane.getLastStepHaltingNumber('1i_0')
%    laneCommandsTested = 1;
%  end
   
  %% MULTIENTRY=EXIT COMMANDS: Note that you have to create the detector in the cross.det.xml file  
%  if ~muiCommandsTested
%    muiVehicleNumber = traci.multientryexit.getLastStepVehicleNumber('0')
%    muiMeanSpeed = traci.multientryexit.getLastStepMeanSpeed('0')
%    muiVehIDs = traci.multientryexit.getLastStepVehicleIDs('0')
%    muiHaltingVehicles = traci.multientryexit.getLastStepHaltingNumber('0')
%    muiCommandsTested = 1;
%  end
    
  %% SIMULATION COMMANDS
%  if ~simCommandsTested
%    currentTime = traci.simulation.getCurrentTime()
%    loadedNumber = traci.simulation.getLoadedNumber()
%    loadedIDList = traci.simulation.getLoadedIDList();
%    departedNumber = traci.simulation.getDepartedNumber()
%    departedIDList = traci.simulation.getDepartedIDList();
%    arrivedNumber = traci.simulation.getArrivedNumber()
%    arrivedIDList = traci.simulation.getArrivedIDList()
%    startingTeleportNumber = traci.simulation.getStartingTeleportNumber()
%    startingTeleportIDList = traci.simulation.getStartingTeleportIDList()
%    endingTeleportNumber = traci.simulation.getEndingTeleportNumber()
%    deltaT = traci.simulation.getDeltaT()
%    netBoundary = traci.simulation.getNetBoundary()
%    [x y] = traci.simulation.convert2D('1i',10)
%    [roadID pos laneID] = traci.simulation.convertRoad(20, 508.35)
%    [longitude latitude] = traci.simulation.convertGeo(20, 508.35)
%    distance2D = traci.simulation.getDistance2D(20, 508.35, 30, 508.35)
%    distanceRoad = traci.simulation.getDistanceRoad('1i', 10, '1i', 20)
%    simCommandsTested = 1;
%  end
    
  %% TRAFFIC LIGHTS COMMANDS   
%  if ~tlsCommandsTested
%    tlsRYGState = traci.trafficlights.getRedYellowGreenState('0')
%    tlsRYGDefinition = traci.trafficlights.getCompleteRedYellowGreenDefinition('0')
%    tlscontrolledLanes = traci.trafficlights.getControlledLanes('0')
%    tlscontrolledLinks = traci.trafficlights.getControlledLinks('0')
%    tlsProgram = traci.trafficlights.getProgram('0')
%    tlsPhase = traci.trafficlights.getPhase('0')
%    traci.trafficlights.setPhase('0',0);
%    traci.trafficlights.setProgram('0','0');
%    traci.trafficlights.setPhaseDuration('0',5);
%    myRYGDefinition = traci.trafficlights.Logic('0',0,0,0,...
%      {traci.trafficlights.Phase(31000,31000,31000,'GrGr'),...
%       traci.trafficlights.Phase(31000,31000,31000,'rGrG'),...
%       traci.trafficlights.Phase(6000,6000,6000,'ryry')});
%   traci.trafficlights.setCompleteRedYellowGreenDefinition('0',tlsRYGDefinition{1});
%   tlsRYGDefinition = traci.trafficlights.getCompleteRedYellowGreenDefinition('0');
%   tlsCommandsTested = 1;
% end
    
  % Change the phase of the traffic light if a vehicle passed through the
  % induction loop
 % if numPriorityVehicles > 0
%    traci.gui.screenshot('View #0','passedvehicle.bmp')
%    
%    if step == 100
%      loop0VehicleData = traci.inductionloop.getVehicleData('0')
%    end
    
 %   if programPointer == length(PROGRAM)
  %    programPointer = 1;
	%  elseif ~strcmp(PROGRAM(programPointer), WEYELLOW)
     % programPointer = 4;
    %end
  %end
  %traci.trafficlights.setRedYellowGreenState('0', PROGRAM{programPointer});
    
  % AN ADDITIONAL EVIDENCE OF THE TRAFFIC LIGHTS SUBSCRIPTION, DON'T
  % FORGET TO SET THE SUBSCRIPTION BEFORE EXECUTING IT.
  % if no > 0
  %   tlsCurrentPhaseHandle = traci.trafficlights.getSubscriptionResults('0');
  %   tlsCurrentPhase = tlsCurrentPhaseHandle(traci.constants.TL_RED_YELLOW_GREEN_STATE);
  %   fprintf('The traffic lights'' phase changed to: %s\n', tlsCurrentPhase)
  % end
    
  % AN ADDITIONAL EVIDENCE OF THE LANE SUBSCRIPTIONS, ENABLE THE PLOTTING
  % FUNCTIONS BELOW TO VISUALIZE IT.
%  WElaneoccupancy(i) = traci.lane.getLastStepVehicleNumber('1i_0')+...
%  traci.lane.getLastStepVehicleNumber('2i_0');
%  NSlaneoccupancy(i) = traci.lane.getLastStepVehicleNumber('3i_0')+...
%    traci.lane.getLastStepVehicleNumber('4i_0');
     
%  steps(i) = i;
  step = step + 1;
end

%P = poisspdf(X,y);
V = {'v1','v2','v3','v4','v5','v6','v7','v8','v9','v10','v11','v12','v13','v14','v15'} ;
%rslt{1}={'v1'};
x=15;
%veh= randsample(V,4);
i=1;
while x~=0
k=randi(x);
veh= randsample(V,k);
V=setdiff(V,veh);
x=x-k;
rslt{i}=veh;
i=i+1;
end
traci.close();
fprintf('SUMO version: %s\nTraCI version: %d\n',sumoVersion,traciVersion);


eof=length(rslt);
fileID = fopen('C:\Users\SOFIENE\Downloads\SUMO\tuto_par3\test.rouX.xml','w');
%fprintf(fileID,'%6s %12s\r\n','x','exp(x)');
fprintf(fileID,'%6s \r\n','<?xml version="1.0" encoding="UTF-8"?>');
fprintf(fileID,'%6s \r\n','<routes xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:noNamespaceSchemaLocation="http://sumo.dlr.de/xsd/routes_file.xsd">');
 periode=0;
for i=1:eof
    temp=rslt{1,i};
    for j=1:length(temp)
    fprintf(fileID,'%6s%6s%6s%5d%6s\n','<vehicle id="',temp{j},'" depart="',periode,'" color="0,1,1">');
	fprintf(fileID,'%6s%6s%6s%6s%6s%6s%6s%6s%6s%6s%6s%6s \r\n','<route edges="','f','e','-a','a','b','-d','f','e','b','c','"/>');
    fprintf(fileID,'%6s \r\n','<param key="has.rerouting.device" value="true"/>');
	fprintf(fileID,'%6s \r\n','</vehicle>');
    end
    periode=periode+30;
end
%temp=rslt{1,1};
fprintf(fileID,'%6s \r\n','</routes>');

fclose(fileID);