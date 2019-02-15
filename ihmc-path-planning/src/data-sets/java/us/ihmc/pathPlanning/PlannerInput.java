package us.ihmc.pathPlanning;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlannerInput
{
   private Point3D startPosition = new Point3D();
   private Point3D goalPosition = new Point3D();
   private double startYaw = Double.NaN;
   private double goalYaw = Double.NaN;
   private HashMap<String, List<String>> additionalData = new HashMap<>();

   public Point3D getStartPosition()
   {
      return startPosition;
   }

   public Point3D getGoalPosition()
   {
      return goalPosition;
   }

   public double getStartYaw()
   {
      return startYaw;
   }

   public double getGoalYaw()
   {
      return goalYaw;
   }

   public boolean getVisGraphIsTestable()
   {
      return flagEquals(DataSetIOTools.VIS_GRAPH_TAG, DataSetIOTools.TESTABLE_FLAG);
   }

   public boolean getStepPlannerIsTestable()
   {
      return flagEquals(DataSetIOTools.STEP_PLANNERS_TAG, DataSetIOTools.TESTABLE_FLAG);
   }

   public boolean getQuadrupedPlannerIsTestable()
   {
      return flagEquals(DataSetIOTools.QUADRUPED_PLANNER_TAG, DataSetIOTools.TESTABLE_FLAG);
   }

   public boolean getVisGraphIsInDevelopment()
   {
      return flagEquals(DataSetIOTools.VIS_GRAPH_TAG, DataSetIOTools.IN_DEVELOPMENT_FLAG);
   }

   public boolean getStepPlannerIsInDevelopment()
   {
      return flagEquals(DataSetIOTools.STEP_PLANNERS_TAG, DataSetIOTools.IN_DEVELOPMENT_FLAG);
   }

   public boolean getQuadrupedPlannerIsInDevelopment()
   {
      return flagEquals(DataSetIOTools.QUADRUPED_PLANNER_TAG, DataSetIOTools.IN_DEVELOPMENT_FLAG);
   }

   public boolean containsTimeoutFlag(String prefix)
   {
      return containsFlag(prefix + DataSetIOTools.TIMEOUT_SUFFIX);
   }

   public double getQuadrupedTimeout()
   {
      return getDoubleFlag(DataSetIOTools.QUADRUPED_TIMEOUT_TAG);
   }

   public double getTimeoutFlag(String prefix)
   {
      return getDoubleFlag(prefix + DataSetIOTools.TIMEOUT_SUFFIX);
   }

   public List<String> getAdditionalData(String key)
   {
      return additionalData.get(key);
   }

   public void setStartPosition(Point3DReadOnly startPosition)
   {
      this.startPosition.set(startPosition);
   }

   public void setStartPosition(double x, double y, double z)
   {
      this.startPosition.set(x, y, z);
   }

   public void setGoalPosition(Point3DReadOnly goalPosition)
   {
      this.goalPosition.set(goalPosition);
   }

   public void setGoalPosition(double x, double y, double z)
   {
      this.goalPosition.set(x, y, z);
   }

   public void setStartYaw(double yaw)
   {
      this.startYaw = yaw;
   }

   public void setGoalYaw(double yaw)
   {
      this.goalYaw = yaw;
   }

   public void addAdditionalData(String key, String value)
   {
      additionalData.computeIfAbsent(key, k -> new ArrayList<>()).add(value);
   }

   public boolean containsFlag(String key)
   {
      return additionalData.containsKey(key);
   }

   public boolean flagEquals(String key, String value)
   {
      return containsFlag(key) && additionalData.get(key).get(0).equals(value);
   }

   private double getDoubleFlag(String key)
   {
      if(!additionalData.containsKey(key))
         return Double.NaN;
      return Double.parseDouble(additionalData.get(key).get(0));
   }

   public boolean hasStartOrientation()
   {
      return !Double.isNaN(startYaw);
   }

   public boolean hasGoalOrientation()
   {
      return !Double.isNaN(goalYaw);
   }

   /* package-private */
   HashMap<String, List<String>> getAdditionDataMap()
   {
      return additionalData;
   }
}
