package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class CliffDetectionTools
{

   public static boolean isNearCliff(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, double footYaw, double cliffHeightToAvoid,
                                     double forward, double backward, double left, double right)
   {
      if (!Double.isFinite(cliffHeightToAvoid))
         return true;

      Point3D highestNearbyPoint = new Point3D();
      double maximumCliffZInSoleFrame = findHighestNearbyPoint(planarRegionsList, footInWorld, footYaw, highestNearbyPoint, forward, backward, left, right);

      return maximumCliffZInSoleFrame > cliffHeightToAvoid;
   }

   public static PlanarRegion findHighestNearbyRegion(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, Point3DBasics highestNearbyPointToPack,
                                                      double minimumDistanceFromCliffBottoms)
   {
      PlanarRegion highestRegion = null;
      highestNearbyPointToPack.setZ( Double.NEGATIVE_INFINITY);

      List<PlanarRegion> intersectingRegionsToPack = PlanarRegionTools
            .filterPlanarRegionsWithBoundingCircle(new Point2D(footInWorld), minimumDistanceFromCliffBottoms, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegionsToPack)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();
         double distanceToPoint = footInWorld.distanceXY(closestPointInWorld);

         if (distanceToPoint < minimumDistanceFromCliffBottoms && heightOfPointFromFoot > highestNearbyPointToPack.getZ())
         {
            highestRegion = intersectingRegion;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return highestRegion;
   }

   public static double findHighestNearbyPoint(PlanarRegionsList planarRegionsList, Point3DReadOnly footInWorld, double footYaw,
                                               Point3DBasics highestNearbyPointToPack, double forward, double backward, double left, double right)
   {
      double maxZInSoleFrame = Double.NEGATIVE_INFINITY;

      RigidBodyTransform transformToRegion = new RigidBodyTransform();
      transformToRegion.setRotationYaw(footYaw);


      ConvexPolygon2D tempPolygon = new ConvexPolygon2D();
      tempPolygon.addVertex(forward, left);
      tempPolygon.addVertex(forward, right);
      tempPolygon.addVertex(backward, left);
      tempPolygon.addVertex(backward, right);
      tempPolygon.update();
      tempPolygon.applyTransform(transformToRegion);
      tempPolygon.translate(footInWorld.getX(), footInWorld.getY());

      List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsIntersectingPolygon(tempPolygon, planarRegionsList.getPlanarRegionsAsList());

      for (PlanarRegion intersectingRegion : intersectingRegions)
      {
         Point3DReadOnly closestPointInWorld = PlanarRegionTools.closestPointOnPlane(footInWorld, intersectingRegion);

         double heightOfPointFromFoot = closestPointInWorld.getZ() - footInWorld.getZ();

         if (tempPolygon.isPointInside(closestPointInWorld.getX(), closestPointInWorld.getY()) && heightOfPointFromFoot > maxZInSoleFrame)
         {
            maxZInSoleFrame = heightOfPointFromFoot;
            highestNearbyPointToPack.set(closestPointInWorld);
         }
      }

      return maxZInSoleFrame;
   }
}
