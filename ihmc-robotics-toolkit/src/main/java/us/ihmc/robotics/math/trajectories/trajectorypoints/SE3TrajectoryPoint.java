package us.ihmc.robotics.math.trajectories.trajectorypoints;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.math.trajectories.trajectorypoints.interfaces.SE3TrajectoryPointBasics;
import us.ihmc.robotics.math.trajectories.waypoints.SE3Waypoint;
import us.ihmc.robotics.math.trajectories.waypoints.tools.WaypointToStringTools;

public class SE3TrajectoryPoint implements SE3TrajectoryPointBasics
{
   private final SE3Waypoint se3Waypoint = new SE3Waypoint();
   private double time;

   public SE3TrajectoryPoint()
   {
   }

   public SE3TrajectoryPoint(SE3TrajectoryPointBasics other)
   {
      set(other);
   }

   public SE3TrajectoryPoint(double time, Point3DReadOnly position, QuaternionReadOnly orientation, Vector3DReadOnly linearVelocity,
                                   Vector3DReadOnly angularVelocity)
   {
      set(time, position, orientation, linearVelocity, angularVelocity);
   }

   @Override
   public Point3DReadOnly getPosition()
   {
      return se3Waypoint.getPosition();
   }

   @Override
   public void setPosition(double x, double y, double z)
   {
      se3Waypoint.setPosition(x, y, z);
   }

   @Override
   public Vector3DReadOnly getLinearVelocity()
   {
      return se3Waypoint.getLinearVelocity();
   }

   @Override
   public void setLinearVelocity(double x, double y, double z)
   {
      se3Waypoint.setLinearVelocity(x, y, z);
   }

   @Override
   public void applyTransform(Transform transform)
   {
      se3Waypoint.applyTransform(transform);
   }

   @Override
   public void applyInverseTransform(Transform transform)
   {
      se3Waypoint.applyInverseTransform(transform);
   }

   @Override
   public QuaternionReadOnly getOrientation()
   {
      return se3Waypoint.getOrientation();
   }

   @Override
   public void setOrientation(double x, double y, double z, double s)
   {
      se3Waypoint.setOrientation(x, y, z, s);
   }

   @Override
   public Vector3DReadOnly getAngularVelocity()
   {
      return se3Waypoint.getAngularVelocity();
   }

   @Override
   public void setAngularVelocity(double x, double y, double z)
   {
      se3Waypoint.setAngularVelocity(x, y, z);
   }

   @Override
   public void setTime(double time)
   {
      this.time = time;
   }

   @Override
   public double getTime()
   {
      return time;
   }

   @Override
   public String toString()
   {
      return "SE3 trajectory point: (time = " + WaypointToStringTools.format(getTime()) + ", " + WaypointToStringTools.waypointToString(se3Waypoint) + ")";
   }
}
