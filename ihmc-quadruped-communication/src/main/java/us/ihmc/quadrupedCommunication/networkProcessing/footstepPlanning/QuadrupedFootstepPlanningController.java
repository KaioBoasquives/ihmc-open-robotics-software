package us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.trajectories.QuadrupedPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.VisibilityGraphWithAStarPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn.QuadrupedSplineWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn.QuadrupedVisGraphWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForQuadrupedFootstepPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedFootstepPlanningController extends QuadrupedToolboxController
{
   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);
   private final EnumMap<FootstepPlannerType, QuadrupedBodyPathAndFootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicReference<QuadrupedFootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private QuadrupedFootstepPlannerStart start;

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoVisibilityGraphParameters visibilityGraphParameters;
   private final YoFootstepPlannerParameters footstepPlannerParameters;
   private final AtomicLong robotTimestampNanos = new AtomicLong();
   private final YoDouble robotTimestamp = new YoDouble("robotTimestamp", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoDouble timeout = new YoDouble("toolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private final QuadrupedPlanarRegionsTrajectoryExpander swingOverPlanarRegionsTrajectoryExpander;


   public QuadrupedFootstepPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, VisibilityGraphsParameters defaultVisibilityGraphParameters,
                                              FootstepPlannerParameters defaultFootstepPlannerParameters, PointFootSnapperParameters pointFootSnapperParameters,
                                              OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      visibilityGraphParameters = new YoVisibilityGraphParameters(defaultVisibilityGraphParameters, registry);
      footstepPlannerParameters = new YoFootstepPlannerParameters(defaultFootstepPlannerParameters, registry);

      swingOverPlanarRegionsTrajectoryExpander = new QuadrupedPlanarRegionsTrajectoryExpander(registry, graphicsListRegistry);

      if (robotDataReceiver != null)
      {
         plannerMap.put(FootstepPlannerType.SIMPLE_PATH_TURN_WALK_TURN,
                        new QuadrupedSplineWithTurnWalkTurnPlanner(xGaitSettings, robotTimestamp, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(), null, registry));
         plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_TURN_WALK_TURN,
                        new QuadrupedVisGraphWithTurnWalkTurnPlanner(xGaitSettings, visibilityGraphParameters, robotTimestamp, pointFootSnapperParameters,
                                                                     robotDataReceiver.getReferenceFrames(), null, registry));
      }
      plannerMap.put(FootstepPlannerType.A_STAR,
                     QuadrupedAStarFootstepPlanner.createPlanner(footstepPlannerParameters, xGaitSettings, null, registry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR, new VisibilityGraphWithAStarPlanner(footstepPlannerParameters, xGaitSettings,
                                                                                                    visibilityGraphParameters, graphicsListRegistry, registry));
      activePlanner.set(FootstepPlannerType.SIMPLE_PATH_TURN_WALK_TURN);

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      for (FootstepPlannerType plannerKey : plannerMap.keySet())
      {
         QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(plannerKey);
         planner.setGroundPlane(message);
      }
   }

   public void processSupportRegionParameters(QuadrupedSupportPlanarRegionParametersMessage message)
   {
      for (FootstepPlannerType plannerKey : plannerMap.keySet())
      {
         WaypointsForQuadrupedFootstepPlanner planner = plannerMap.get(plannerKey).getWaypointPathPlanner();
         if (planner != null)
            planner.setFallbackRegionSize(message.getInsideSupportRegionSize());
      }
   }

   public void processFootstepPlannerParametersPacket(QuadrupedFootstepPlannerParametersPacket packet)
   {
      footstepPlannerParameters.set(packet);
   }

   public void processVisibilityGraphParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      visibilityGraphParameters.set(packet);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processRobotTimestamp(long timestampInNanos)
   {
      this.robotTimestampNanos.set(timestampInNanos);
   }

   public void processFootstepPlanningRequest(QuadrupedFootstepPlanningRequestPacket footstepPlanningRequestPacket)
   {
      latestRequestReference.set(footstepPlanningRequestPacket);
   }

   @Override
   public boolean initializeInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      isDone.set(false);

      QuadrupedFootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      planId.set(request.getPlannerRequestId());
      if (request.getRequestedFootstepPlannerType() >= 0)
         activePlanner.set(FootstepPlannerType.fromByte(request.getRequestedFootstepPlannerType()));

      PlanarRegionsListMessage planarRegionsListMessage = request.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         if (planarRegionsList.isEmpty())
            this.planarRegionsList = Optional.empty();
         else
            this.planarRegionsList = Optional.of(planarRegionsList);
      }

      QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegionsList(planarRegionsList.get());
      }
      else
      {
         planner.setPlanarRegionsList(null);
      }

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getGoalPositionInWorld(), request.getGoalOrientationInWorld());

      start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();

      FootstepPlannerTargetType targetType = FootstepPlannerTargetType.fromByte(request.getStartTargetType());
      if (targetType == FootstepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getBodyPositionInWorld(), request.getBodyOrientationInWorld());
         start.setStartPose(initialPose);
         start.setStartType(targetType);
      }
      else
      {
         start.setFootStartPosition(RobotQuadrant.FRONT_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontLeftPositionInWorld()));
         start.setFootStartPosition(RobotQuadrant.FRONT_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontRightPositionInWorld()));
         start.setFootStartPosition(RobotQuadrant.HIND_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindLeftPositionInWorld()));
         start.setFootStartPosition(RobotQuadrant.HIND_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindRightPositionInWorld()));
         start.setStartType(FootstepPlannerTargetType.FOOTSTEPS);
      }

      start.setInitialQuadrant(RobotQuadrant.fromByte(request.getInitialStepRobotQuadrant()));
      goal.setGoalPose(goalPose);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0.0 && Double.isFinite(horizonLength))
         planner.setPlanningHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         this.timeout.set(timeout);
         planner.setTimeout(timeout);
      }
      else
      {
         planner.setTimeout(Double.POSITIVE_INFINITY);
      }

      planner.setStart(start);
      planner.setGoal(goal);

      return true;
   }

   @Override
   public void updateInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      reportMessage(packStatus(FootstepPlannerStatus.PLANNING_PATH));

      FootstepPlanningResult status = planner.planPath();

      BodyPathPlan bodyPathPlan = null;
      if (status.validForExecution())
      {
         bodyPathPlan = planner.getPathPlan();
         reportMessage(packStatus(FootstepPlannerStatus.PLANNING_STEPS));
         reportMessage(packPathResult(bodyPathPlan, status));

         status = planner.plan();
      }

      FootstepPlan footstepPlan;
      if (status.validForExecution())
         footstepPlan = planner.getPlan();
      else
         footstepPlan = null;

      reportMessage(packStepResult(footstepPlan, bodyPathPlan, status));

      finishUp();
   }

   public void finishUp()
   {
      if (DEBUG)
         PrintTools.info("Finishing up the planner");
      plannerMap.get(activePlanner.getEnumValue()).cancelPlanning();
      reportMessage(packStatus(FootstepPlannerStatus.IDLE));
      isDone.set(true);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlannerStatusMessage packStatus(FootstepPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
   }

   private BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, FootstepPlanningResult status)
   {
      if (DEBUG)
      {
         PrintTools.info("Finished planning path. Result: " + status);
      }

      BodyPathPlanMessage result = new BodyPathPlanMessage();
      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));

         result.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
         result.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      return result;
   }

   private QuadrupedFootstepPlanningToolboxOutputStatus packStepResult(FootstepPlan footstepPlan, BodyPathPlan bodyPathPlan, FootstepPlanningResult status)
   {
      QuadrupedFootstepPlanningToolboxOutputStatus result = new QuadrupedFootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new QuadrupedTimedStepListMessage());
      }
      else
      {
         result.getFootstepDataList().set(convertToTimedStepListMessage(footstepPlan));
         result.getLowLevelPlannerGoal().set(footstepPlan.getLowLevelPlanGoal());
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      result.setTimeTaken(plannerMap.get(activePlanner.getEnumValue()).getPlanningDuration());

      addObstacleAvoidanceWaypointsWhenNecessary(result.getFootstepDataList());

      return result;
   }

   private void addObstacleAvoidanceWaypointsWhenNecessary(QuadrupedTimedStepListMessage stepListMessage)
   {
      if (!planarRegionsList.isPresent())
         return;

      QuadrantDependentList<FramePoint3D> footPositions = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FramePoint3D footPosition = new FramePoint3D(start.getFootGoalPosition(robotQuadrant));
         footPositions.put(robotQuadrant, footPosition);
      }

      FramePoint3D swingStartPosition = new FramePoint3D();
      FramePoint3D swingEndPosition = new FramePoint3D();


      for (int stepIndex = 1; stepIndex < stepListMessage.getQuadrupedStepList().size(); stepIndex++)
      {
         QuadrupedStepMessage stepMessage = stepListMessage.getQuadrupedStepList().get(stepIndex).getQuadrupedStepMessage();
         RobotQuadrant movingQuadrant = RobotQuadrant.fromByte(stepMessage.getRobotQuadrant());
         swingStartPosition.set(footPositions.get(movingQuadrant));
         swingEndPosition.set(stepMessage.getGoalPosition());


         if (swingOverPlanarRegionsTrajectoryExpander.expandTrajectoryOverPlanarRegions(swingStartPosition, swingEndPosition, 0.08, planarRegionsList.get(), null))
         {
            List<FramePoint3D> expandedWaypoints = swingOverPlanarRegionsTrajectoryExpander.getExpandedWaypoints();
            for (int waypointIndex = 0; waypointIndex < expandedWaypoints.size(); waypointIndex++)
            {
               stepMessage.getCustomPositionWaypoints().add().set(expandedWaypoints.get(waypointIndex));
            }
         }



         footPositions.get(movingQuadrant).set(swingEndPosition);
      }

   }

   private static QuadrupedTimedStepListMessage convertToTimedStepListMessage(FootstepPlan footstepPlan)
   {
      if (footstepPlan == null)
         return null;

      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++)
      {
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(footstepPlan.getFootstep(i)));
      }

      return QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, false);
   }
}
