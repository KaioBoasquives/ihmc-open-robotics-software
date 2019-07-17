package us.ihmc.robotEnvironmentAwareness.filter;

import us.ihmc.euclid.tuple3D.Point3D;

public interface PointCloudFilterInterface
{
   abstract void initialize();
   abstract void updateInput(Point3D[] points);
   abstract void calculate();
   abstract Point3D[] getOutput();
}
