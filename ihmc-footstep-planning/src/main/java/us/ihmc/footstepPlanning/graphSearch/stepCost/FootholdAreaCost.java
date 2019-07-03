package us.ihmc.footstepPlanning.graphSearch.stepCost;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerCostParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootholdAreaCost implements FootstepCost
{
   private static final double weight = 1.0;

   private final FootstepPlannerCostParameters costParameters;
   private final SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons;
   private final FootstepNodeSnapperReadOnly snapper;

   public FootholdAreaCost(FootstepPlannerParameters parameters, SideDependentList<? extends ConvexPolygon2DReadOnly> footPolygons, FootstepNodeSnapperReadOnly snapper)
   {
      this.costParameters = parameters.getCostParameters();
      this.footPolygons = footPolygons;
      this.snapper = snapper;
   }

   @Override
   public double compute(FootstepNode startNode, FootstepNode endNode)
   {
      FootstepNodeSnapData snapData = snapper.getSnapData(endNode);
      ConvexPolygon2D footholdAfterSnap = snapData.getCroppedFoothold();
      double area = footholdAfterSnap.getArea();
      double footArea = footPolygons.get(endNode.getRobotSide()).getArea();

      if (footholdAfterSnap.isEmpty())
         return 0.0;

      double percentAreaUnoccupied = 1.0 - area / footArea;
      return percentAreaUnoccupied * costParameters.getFootholdAreaWeight();
   }
}
