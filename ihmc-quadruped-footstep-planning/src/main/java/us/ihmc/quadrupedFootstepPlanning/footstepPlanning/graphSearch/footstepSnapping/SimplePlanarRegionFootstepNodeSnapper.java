package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;

public class SimplePlanarRegionFootstepNodeSnapper extends FootstepNodeSnapper
{
   protected final Point2D footPosition = new Point2D();

   private final DoubleProvider projectionInsideDelta;
   private final BooleanProvider projectInsideUsingConvexHull;
   protected final PlanarRegionConstraintDataHolder constraintDataHolder = new PlanarRegionConstraintDataHolder();
   protected final PlanarRegionConstraintDataParameters constraintDataParameters = new PlanarRegionConstraintDataParameters();

   public SimplePlanarRegionFootstepNodeSnapper(FootstepPlannerParameters parameters, DoubleProvider projectionInsideDelta,
                                                BooleanProvider projectInsideUsingConvexHull, boolean enforceTranslationLessThanGridCell)
   {
      super(parameters);

      this.projectionInsideDelta = projectionInsideDelta;
      this.projectInsideUsingConvexHull = projectInsideUsingConvexHull;

      constraintDataParameters.enforceTranslationLessThanGridCell = enforceTranslationLessThanGridCell;
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      super.setPlanarRegions(planarRegionsList);
      constraintDataHolder.clear();
      constraintDataParameters.projectionInsideDelta = projectionInsideDelta.getValue();
      constraintDataParameters.projectInsideUsingConvexHull = projectInsideUsingConvexHull.getValue();
   }

   @Override
   public FootstepNodeSnapData snapInternal(RobotQuadrant robotQuadrant, int xIndex, int yIndex)
   {
      FootstepNodeTools.getFootPosition(xIndex, yIndex, footPosition);
      Vector2D projectionTranslation = new Vector2D();
      PlanarRegion highestRegion = PlanarRegionSnapTools
            .findHighestRegionWithProjection(footPosition, projectionTranslation, constraintDataHolder, planarRegionsList.getPlanarRegionsAsList(),
                                             constraintDataParameters);

      if (highestRegion == null || projectionTranslation.containsNaN() || isTranslationBiggerThanGridCell(projectionTranslation))
      {
         return FootstepNodeSnapData.emptyData();
      }
      else
      {
         RigidBodyTransform snapTransform = getSnapTransformIncludingTranslation(footPosition, projectionTranslation, highestRegion);
         return new FootstepNodeSnapData(snapTransform);
      }
   }

   protected RigidBodyTransform getSnapTransformIncludingTranslation(Point2DReadOnly position, Vector2DReadOnly translation, PlanarRegion containingRegion)
   {
      double x = position.getX();
      double xTranslated = x + translation.getX();
      double y = position.getY();
      double yTranslated = y + translation.getY();
      double z = containingRegion.getPlaneZGivenXY(xTranslated, yTranslated);


      Vector3D surfaceNormal = new Vector3D();
      containingRegion.getNormal(surfaceNormal);

      RigidBodyTransform snapTransform = PlanarRegionSnapTools.createTransformToMatchSurfaceNormalPreserveX(surfaceNormal);
      PlanarRegionSnapTools.setTranslationSettingZAndPreservingXAndY(x, y, xTranslated, yTranslated, z, snapTransform);

      return snapTransform;
   }

   protected boolean isTranslationBiggerThanGridCell(Vector2D translation)
   {
      if (!constraintDataParameters.enforceTranslationLessThanGridCell)
         return false;

      double maximumTranslationPerAxis = 0.5 * FootstepNode.gridSizeXY;
      return Math.abs(translation.getX()) > maximumTranslationPerAxis || Math.abs(translation.getY()) > maximumTranslationPerAxis;
   }

}