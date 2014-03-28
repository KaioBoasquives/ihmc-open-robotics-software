package us.ihmc.darpaRoboticsChallenge.drcRobot;

import java.util.Set;

import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LimbName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.darpaRoboticsChallenge.DRCRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.Pair;

public abstract class DRCRobotJointMap implements SDFJointNameMap
{
   public abstract double getPelvisToFoot();
   
   public abstract Set<String> getLastSimulatedJoints();

   public abstract String[] getIMUSensorsToUse();

   public abstract String getRightCameraName();

   public abstract String getLidarSensorName();

   public abstract String getLeftCameraName();

   public abstract boolean isTorqueVelocityLimitsEnabled();

   public abstract String getModelName();

   public abstract String getLidarJointName();

   public abstract DRCRobotModel getSelectedModel();

   public abstract double getAnkleHeight();

   public abstract String getJointBeforeFootName(RobotSide robotSide);

   public abstract NeckJointName[] getNeckJointNames();

   public abstract ArmJointName[] getArmJointNames();

   public abstract LegJointName[] getLegJointNames();

   public abstract String getHeadName();

   public abstract String getChestName();

   public abstract String getPelvisName();

   public abstract SpineJointName getSpineJointName(String jointName);

   public abstract NeckJointName getNeckJointName(String jointName);

   public abstract JointRole getJointRole(String jointName);

   public abstract Pair<RobotSide, LimbName> getLimbName(String limbName);

   public abstract Pair<RobotSide, ArmJointName> getArmJointName(String jointName);

   public abstract Pair<RobotSide, LegJointName> getLegJointName(String jointName);

   public abstract String getNameOfJointBeforeChest();

   public abstract String getNameOfJointBeforeThigh(RobotSide robotSide);

   public abstract String getNameOfJointBeforeHand(RobotSide robotSide);

   public abstract SpineJointName[] getSpineJointNames();
   
   public abstract SideDependentList<String> getFeetForceSensorNames();

   public abstract SideDependentList<String> getJointBeforeThighNames();
   
   public abstract String[] getOrderedJointNames();
   
   public abstract String getHighestNeckPitchJointName();

//   public abstract Transform3D getPelvisContactPointTransform();
//
//   public abstract List<Point2d> getPelvisContactPoints();
//
//   public abstract Transform3D getPelvisBackContactPointTransform();
//
//   public abstract List<Point2d> getPelvisBackContactPoints();
//
//   public abstract Transform3D getChestBackContactPointTransform();
//
//   public abstract List<Point2d> getChestBackContactPoints();
//
//   public abstract SideDependentList<Transform3D> getThighContactPointTransforms();
//
//   public abstract SideDependentList<List<Point2d>> getThighContactPoints();
   
}