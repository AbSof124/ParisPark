       1             : /****************************************************************************/
       2             : // Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
       3             : // Copyright (C) 2015-2017 German Aerospace Center (DLR) and others.
       4             : /****************************************************************************/
       5             : //
       6             : //   This program and the accompanying materials
       7             : //   are made available under the terms of the Eclipse Public License v2.0
       8             : //   which accompanies this distribution, and is available at
       9             : //   http://www.eclipse.org/legal/epl-v20.html
      10             : //
      11             : /****************************************************************************/
      12             : /// @file    MSParkingArea.cpp
      13             : /// @author  Mirco Sturari
      14             : /// @author  Jakob Erdmann
      15             : /// @date    Tue, 19.01.2016
      16             : /// @version $Id$
      17             : ///
      18             : // A area where vehicles can park next to the road
      19             : /****************************************************************************/
      20             : 
      21             : 
      22             : // ===========================================================================
      23             : // included modules
      24             : // ===========================================================================
      25             : #ifdef _MSC_VER
      26             : #include <windows_config.h>
      27             : #else
      28             : #include <config.h>
      29             : #endif
      30             : 
      31             : #include <cassert>
      32             : #include <utils/vehicle/SUMOVehicle.h>
      33             : #include <utils/geom/Position.h>
      34             : #include <utils/geom/GeomHelper.h>
      35             : #include <microsim/MSNet.h>
      36             : #include <microsim/MSVehicleType.h>
      37             : #include "MSLane.h"
      38             : #include "MSTransportable.h"
      39             : #include "MSParkingArea.h"
      40             : 
      41             : 
      42             : // ===========================================================================
      43             : // method definitions
      44             : // ===========================================================================
      45          22 : MSParkingArea::MSParkingArea(const std::string& id,
      46             :                              const std::vector<std::string>& lines,
      47             :                              MSLane& lane,
      48             :                              double begPos, double endPos,
      49             :                              unsigned int capacity,
      50             :                              double width, double length, double angle) :
      51             :     MSStoppingPlace(id, lines, lane, begPos, endPos),
      52             :     myCapacity(capacity),
      53             :     myWidth(width),
      54             :     myLength(length),
      55          22 :     myAngle(angle) {
      56             :     // initialize unspecified defaults
      57          22 :     if (myWidth == 0) {
      58          10 :         myWidth = SUMO_const_laneWidth;
      59             :     }
      60          22 :     if (myLength == 0) {
      61           2 :         myLength = getSpaceDim();
      62             :     }
      63             : 
      64          22 :     const double offset = MSNet::getInstance()->lefthand() ? -1 : 1;
      65          44 :     myShape = lane.getShape().getSubpart(
      66             :                   lane.interpolateLanePosToGeometryPos(begPos),
      67          22 :                   lane.interpolateLanePosToGeometryPos(endPos));
      68          22 :     myShape.move2side((lane.getWidth() / 2. + myWidth / 2.) * offset);
      69             :     // Initialize space occupancies if there is a road-side capacity
      70             :     // The overall number of lots is fixed and each lot accepts one vehicle regardless of size
      71          22 :     if (myCapacity > 0) {
      72          54 :         for (int i = 1; i <= myCapacity; ++i) {
      73          44 :             mySpaceOccupancies[i] = LotSpaceDefinition();
      74          44 :             mySpaceOccupancies[i].index = i;
      75          44 :             mySpaceOccupancies[i].vehicle = 0;
      76          44 :             mySpaceOccupancies[i].myWidth = myWidth;
      77          44 :             mySpaceOccupancies[i].myLength = myLength;
      78          44 :             mySpaceOccupancies[i].myEndPos = myBegPos + getSpaceDim() * i;
      79             : 
      80          44 :             const Position& f = myShape.positionAtOffset(getSpaceDim() * (i - 1));
      81          88 :             const Position& s = myShape.positionAtOffset(getSpaceDim() * (i));
      82          44 :             double lot_angle = ((double) atan2((s.x() - f.x()), (f.y() - s.y())) * (double) 180.0 / (double) M_PI) + myAngle;
      83          44 :             mySpaceOccupancies[i].myRotation = lot_angle;
      84          44 :             if (myAngle == 0) {
      85             :                 // parking parallel to the road
      86          24 :                 mySpaceOccupancies[i].myPosition = s;
      87             :             } else {
      88             :                 // angled parking
      89          20 :                 mySpaceOccupancies[i].myPosition = (f + s) * 0.5;
      90             :             }
      91             : 
      92          44 :         }
      93             :     }
      94          22 :     computeLastFreePos();
      95          22 : }
      96             : 
      97          39 : MSParkingArea::~MSParkingArea() {}
      98             : 
      99             : double
     100       20002 : MSParkingArea::getLastFreePos(const SUMOVehicle& /* forVehicle */) const {
     101       20002 :     return myLastFreePos;
     102             : }
     103             : 
     104             : Position
     105          40 : MSParkingArea::getVehiclePosition(const SUMOVehicle& forVehicle) {
     106          40 :     std::map<unsigned int, LotSpaceDefinition >::iterator i;
     107          64 :     for (i = mySpaceOccupancies.begin(); i != mySpaceOccupancies.end(); i++) {
     108          64 :         if ((*i).second.vehicle == &forVehicle) {
     109          40 :             return (*i).second.myPosition;
     110             :         }
     111             :     }
     112           0 :     return Position::INVALID;
     113             : }
     114             : 
     115             : double
     116         332 : MSParkingArea::getVehicleAngle(const SUMOVehicle& forVehicle) {
     117         332 :     std::map<unsigned int, LotSpaceDefinition >::iterator i;
     118         988 :     for (i = mySpaceOccupancies.begin(); i != mySpaceOccupancies.end(); i++) {
     119         988 :         if ((*i).second.vehicle == &forVehicle) {
     120         332 :             return (((*i).second.myRotation - 90.) * (double) M_PI / (double) 180.0);
     121             :         }
     122             :     }
     123           0 :     return 0.;
     124             : }
     125             : 
     126             : 
     127             : double
     128         134 : MSParkingArea::getSpaceDim() const {
     129         134 :     return (myEndPos - myBegPos) / myCapacity;
     130             : }
     131             : 
     132             : 
     133             : void
     134          92 : MSParkingArea::addLotEntry(double x, double y, double z,
     135             :                            double width, double length, double angle) {
     136             : 
     137          92 :     const int i = (int)mySpaceOccupancies.size() + 1;
     138             : 
     139          92 :     mySpaceOccupancies[i] = LotSpaceDefinition();
     140          92 :     mySpaceOccupancies[i].index = i;
     141          92 :     mySpaceOccupancies[i].vehicle = 0;
     142          92 :     mySpaceOccupancies[i].myPosition = Position(x, y, z);
     143          92 :     mySpaceOccupancies[i].myWidth = width;
     144          92 :     mySpaceOccupancies[i].myLength = length;
     145          92 :     mySpaceOccupancies[i].myRotation = angle;
     146          92 :     mySpaceOccupancies[i].myEndPos = myEndPos;
     147          92 :     myCapacity = (int)mySpaceOccupancies.size();
     148          92 :     computeLastFreePos();
     149          92 : }
     150             : 
     151             : 
     152             : void
     153         166 : MSParkingArea::enter(SUMOVehicle* what, double beg, double end) {
     154         166 :     if (myLastFreeLot >= 1 && myLastFreeLot <= (int)mySpaceOccupancies.size()) {
     155         166 :         mySpaceOccupancies[myLastFreeLot].vehicle = what;
     156         166 :         myEndPositions[what] = std::pair<double, double>(beg, end);
     157         166 :         computeLastFreePos();
     158             :     }
     159         166 : }
     160             : 
     161             : 
     162             : void
     163         166 : MSParkingArea::leaveFrom(SUMOVehicle* what) {
     164             :     assert(myEndPositions.find(what) != myEndPositions.end());
     165         166 :     std::map<unsigned int, LotSpaceDefinition >::iterator i;
     166         494 :     for (i = mySpaceOccupancies.begin(); i != mySpaceOccupancies.end(); i++) {
     167         494 :         if ((*i).second.vehicle == what) {
     168         166 :             (*i).second.vehicle = 0;
     169         166 :             break;
     170             :         }
     171             :     }
     172         166 :     myEndPositions.erase(myEndPositions.find(what));
     173         166 :     computeLastFreePos();
     174         166 : }
     175             : 
     176             : 
     177             : void
     178         446 : MSParkingArea::computeLastFreePos() {
     179         446 :     myLastFreeLot = 0;
     180         446 :     myLastFreePos = myBegPos;
     181         446 :     std::map<unsigned int, LotSpaceDefinition >::iterator i;
     182        1140 :     for (i = mySpaceOccupancies.begin(); i != mySpaceOccupancies.end(); i++) {
     183        1048 :         if ((*i).second.vehicle == 0) {
     184         354 :             myLastFreeLot = (*i).first;
     185         354 :             myLastFreePos = (*i).second.myEndPos;
     186         354 :             break;
     187             :         }
     188             :     }
     189         446 : }
     190             : 
     191             : 
     192             : double
     193          92 : MSParkingArea::getWidth() const {
     194          92 :     return myWidth;
     195             : }
     196             : 
     197             : 
     198             : double
     199          92 : MSParkingArea::getLength() const {
     200          92 :     return myLength;
     201             : }
     202             : 
     203             : 
     204             : double
     205          92 : MSParkingArea::getAngle() const {
     206          92 :     return myAngle;
     207             : }
     208             : 
     209             : 
     210             : int
     211        8732 : MSParkingArea::getCapacity() const {
     212        8732 :     return myCapacity;
     213             : }
     214             : 
     215             : 
     216             : int
     217        8680 : MSParkingArea::getOccupancy() const {
     218        8680 :     return (int)myEndPositions.size();
     219       43554 : }
     220             : 
     221             : 
     222             : /****************************************************************************/