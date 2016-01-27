package us.ihmc.aware.controller;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.aware.controller.common.*;
import us.ihmc.aware.planning.QuadrupedTimedStep;
import us.ihmc.aware.vmc.QuadrupedContactForceLimits;
import us.ihmc.aware.vmc.QuadrupedJointLimits;
import us.ihmc.aware.vmc.QuadrupedVirtualModelController;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.aware.parameters.QuadrupedRobotParameters;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.aware.parameters.QuadrupedVirtualModelBasedStepParameters;
import us.ihmc.quadrupedRobotics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.supportPolygon.QuadrupedSupportPolygon;
import us.ihmc.quadrupedRobotics.util.HeterogeneousMemoryPool;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.aware.util.PreallocatedQueue;
import us.ihmc.robotics.controllers.AxisAngleOrientationController;
import us.ihmc.robotics.controllers.EuclideanPositionController;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.stateMachines.State;
import us.ihmc.robotics.stateMachines.StateMachine;
import us.ihmc.robotics.stateMachines.StateTransition;
import us.ihmc.robotics.stateMachines.StateTransitionCondition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.ArtifactList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

import java.awt.*;

public class QuadrupedVirtualModelBasedStepController implements QuadrupedController
{
   // parameters
   private final SDFFullRobotModel fullRobotModel;
   private final DoubleYoVariable robotTimestamp;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final QuadrupedVirtualModelBasedStepParameters parameters;
   private final QuadrupedJointNameMap jointNameMap;
   private final double controlDT;
   private final double gravity;
   private final double mass;

   // utilities
   private final QuadrupedJointLimits jointLimits;
   private final QuadrupedContactForceLimits contactForceLimits;
   private final QuadrupedReferenceFrames referenceFrames;
   private final ReferenceFrame worldFrame;
   private final ReferenceFrame bodyFrame;
   private final ReferenceFrame comFrame;
   private final PoseReferenceFrame supportFrame;
   private final QuadrantDependentList<ReferenceFrame> soleFrame;
   private final QuadrantDependentList<RigidBody> footRigidBody;
   private final CenterOfMassJacobian comJacobian;
   private final TwistCalculator twistCalculator;

   // controllers
   private final QuadrupedVirtualModelController virtualModelController;
   private final PIDController comHeightController;
   private final AxisAngleOrientationController bodyOrientationController;
   private final DivergentComponentOfMotionController dcmPositionController;
   private final QuadrantDependentList<EuclideanPositionController> swingPositionController;

   // state machines
   public enum FootState
   {
      SWING_STATE, SUPPORT_STATE
   }

   private final QuadrantDependentList<StateMachine<FootState>> footStateMachine;

   // provider inputs
   private static int STEP_QUEUE_CAPACITY = 30;
   private final PreallocatedQueue<QuadrupedTimedStep> stepQueue;
   private final QuadrantDependentList<QuadrupedTimedStep> stepCache;
   private final FrameOrientation bodyOrientationDesired;
   private double comHeightDesired;

   // setpoints
   private final QuadrantDependentList<FramePoint> solePositionSetpoint;
   private final QuadrantDependentList<FrameVector> soleLinearVelocitySetpoint;
   private final QuadrantDependentList<FrameVector> soleForceFeedforwardSetpoint;
   private final QuadrantDependentList<FrameVector> soleForceSetpoint;
   private final FrameOrientation bodyOrientationSetpoint;
   private final FrameVector bodyAngularVelocitySetpoint;
   private final FrameVector bodyTorqueFeedforwardSetpoint;
   private final FrameVector bodyTorqueSetpoint;
   private final FramePoint dcmPositionSetpoint;
   private final FrameVector dcmVelocitySetpoint;
   private final FramePoint icpPositionSetpoint;
   private final FrameVector icpVelocitySetpoint;
   private final FramePoint cmpPositionSetpoint;
   private final FramePoint vrpPositionSetpoint;
   private final FrameVector comForceSetpoint;
   private double comHeightSetpoint;

   // estimates
   private final QuadrantDependentList<FrameOrientation> soleOrientationEstimate;
   private final QuadrantDependentList<FramePoint> solePositionEstimate;
   private final QuadrantDependentList<FrameVector> soleAngularVelocityEstimate;
   private final QuadrantDependentList<FrameVector> soleLinearVelocityEstimate;
   private final QuadrupedSupportPolygon supportPolygonEstimate;
   private final FramePoint supportCentroidEstimate;
   private final FrameOrientation supportOrientationEstimate;
   private final FrameOrientation bodyOrientationEstimate;
   private final FramePoint bodyPositionEstimate;
   private final FrameVector bodyLinearVelocityEstimate;
   private final FrameVector bodyAngularVelocityEstimate;
   private final FramePoint comPositionEstimate;
   private final FrameVector comVelocityEstimate;
   private final FramePoint dcmPositionEstimate;
   private final FramePoint icpPositionEstimate;
   private double comHeightEstimate;

   // YoVariables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoFrameOrientation yoBodyOrientationDesired;
   private final DoubleYoVariable yoComHeightDesired;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocitySetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceFeedforwardSetpoint;
   private final QuadrantDependentList<YoFrameVector> yoSoleForceSetpoint;
   private final YoFrameOrientation yoBodyOrientationSetpoint;
   private final YoFrameVector yoBodyAngularVelocitySetpoint;
   private final YoFrameVector yoBodyTorqueFeedforwardSetpoint;
   private final YoFrameVector yoBodyTorqueSetpoint;
   private final YoFramePoint yoIcpPositionSetpoint;
   private final YoFramePoint yoCmpPositionSetpoint;
   private final YoFramePoint yoVrpPositionSetpoint;
   private final YoFrameVector yoComForceSetpoint;
   private final DoubleYoVariable yoComHeightSetpoint;
   private final QuadrantDependentList<YoFrameOrientation> yoSoleOrientationEstimate;
   private final QuadrantDependentList<YoFramePoint> yoSolePositionEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleAngularVelocityEstimate;
   private final QuadrantDependentList<YoFrameVector> yoSoleLinearVelocityEstimate;
   private final YoFrameConvexPolygon2d yoSupportPolygonEstimate;
   private final YoFramePoint yoSupportCentroidEstimate;
   private final YoFrameOrientation yoSupportOrientationEstimate;
   private final YoFrameOrientation yoBodyOrientationEstimate;
   private final YoFramePoint yoBodyPositionEstimate;
   private final YoFrameVector yoBodyAngularVelocityEstimate;
   private final YoFrameVector yoBodyLinearVelocityEstimate;
   private final YoFramePoint yoComPositionEstimate;
   private final YoFrameVector yoComVelocityEstimate;
   private final YoFramePoint yoIcpPositionEstimate;
   private final DoubleYoVariable yoComHeightEstimate;

   // YoGraphics
   private final YoGraphicsList yoGraphicsList;
   private final ArtifactList artifactList;

   // temporary
   private final HeterogeneousMemoryPool pool = new HeterogeneousMemoryPool();

   public QuadrupedVirtualModelBasedStepController(QuadrupedRuntimeEnvironment runtimeEnvironment, QuadrupedRobotParameters robotParameters, QuadrupedVirtualModelController virtualModelController)
   {
      // parameters
      this.fullRobotModel = runtimeEnvironment.getFullRobotModel();
      this.robotTimestamp = runtimeEnvironment.getRobotTimestamp();
      this.yoGraphicsListRegistry = runtimeEnvironment.getGraphicsListRegistry();
      this.parameters = robotParameters.getQuadrupedVirtualModelBasedStepParameters();
      this.jointNameMap = robotParameters.getJointMap();
      this.controlDT = runtimeEnvironment.getControlDT();
      this.gravity = 9.81;
      this.mass = fullRobotModel.getTotalMass();

      // utilities
      jointLimits = new QuadrupedJointLimits(robotParameters.getQuadrupedJointLimits());
      contactForceLimits = new QuadrupedContactForceLimits(robotParameters.getQuadrupedContactForceLimits());
      referenceFrames = virtualModelController.getReferenceFrames();
      comFrame = referenceFrames.getCenterOfMassZUpFrame();
      bodyFrame = referenceFrames.getBodyFrame();
      worldFrame = referenceFrames.getWorldFrame();
      supportFrame = new PoseReferenceFrame("SupportFrame", referenceFrames.getWorldFrame());
      soleFrame = referenceFrames.getFootReferenceFrames();
      footRigidBody = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String jointBeforeFootName = robotParameters.getJointMap().getJointBeforeFootName(robotQuadrant);
         OneDoFJoint jointBeforeFoot = fullRobotModel.getOneDoFJointByName(jointBeforeFootName);
         footRigidBody.set(robotQuadrant, jointBeforeFoot.getSuccessor());
      }
      comJacobian = new CenterOfMassJacobian(fullRobotModel.getElevator());
      twistCalculator = new TwistCalculator(worldFrame, fullRobotModel.getElevator());

      // controllers
      this.virtualModelController = virtualModelController;
      comHeightController = new PIDController("bodyHeight", registry);
      bodyOrientationController = new AxisAngleOrientationController("bodyOrientation", bodyFrame, controlDT, registry);
      dcmPositionController = new DivergentComponentOfMotionController("dcm", comFrame, controlDT, mass, gravity, parameters.getComHeightNominal(), registry);
      swingPositionController = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression() + "SwingState";
         swingPositionController.set(robotQuadrant, new EuclideanPositionController(prefix, soleFrame.get(robotQuadrant), controlDT, registry));
      }

      // state machines
      footStateMachine = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         SwingState swingState = new SwingState(FootState.SWING_STATE, robotQuadrant);
         SupportState supportState = new SupportState(FootState.SUPPORT_STATE, robotQuadrant);
         String prefix = getClass().getSimpleName() + robotQuadrant.getCamelCaseNameForMiddleOfExpression();
         footStateMachine.set(robotQuadrant, new StateMachine<>(prefix, prefix + "TransitionTime", FootState.class, robotTimestamp, registry));
         footStateMachine.get(robotQuadrant).addState(swingState);
         footStateMachine.get(robotQuadrant).addState(supportState);
         footStateMachine.get(robotQuadrant).setCurrentState(FootState.SUPPORT_STATE);
      }

      // provider inputs
      stepQueue = new PreallocatedQueue<>(QuadrupedTimedStep.class, STEP_QUEUE_CAPACITY);
      stepCache = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         stepCache.set(robotQuadrant, new QuadrupedTimedStep(robotQuadrant));
      }
      bodyOrientationDesired = new FrameOrientation(worldFrame);
      comHeightDesired = parameters.getComHeightNominal();

      // setpoints
      solePositionSetpoint = new QuadrantDependentList<>();
      soleLinearVelocitySetpoint = new QuadrantDependentList<>();
      soleForceFeedforwardSetpoint = new QuadrantDependentList<>();
      soleForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.set(robotQuadrant, new FramePoint(worldFrame));
         soleLinearVelocitySetpoint.set(robotQuadrant, new FrameVector(worldFrame));
         soleForceFeedforwardSetpoint.set(robotQuadrant, new FrameVector(worldFrame));
         soleForceSetpoint.set(robotQuadrant, new FrameVector(worldFrame));
      }
      bodyOrientationSetpoint = new FrameOrientation(worldFrame);
      bodyAngularVelocitySetpoint = new FrameVector(worldFrame);
      bodyTorqueFeedforwardSetpoint = new FrameVector(worldFrame);
      bodyTorqueSetpoint = new FrameVector(worldFrame);
      dcmPositionSetpoint = new FramePoint(worldFrame);
      dcmVelocitySetpoint = new FrameVector(worldFrame);
      icpPositionSetpoint = new FramePoint(worldFrame);
      icpVelocitySetpoint = new FrameVector(worldFrame);
      cmpPositionSetpoint = new FramePoint(worldFrame);
      vrpPositionSetpoint = new FramePoint(worldFrame);
      comForceSetpoint = new FrameVector(worldFrame);
      comHeightSetpoint = parameters.getComHeightNominal();

      // estimates
      soleOrientationEstimate = new QuadrantDependentList<>();
      solePositionEstimate = new QuadrantDependentList<>();
      soleAngularVelocityEstimate = new QuadrantDependentList<>();
      soleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientationEstimate.set(robotQuadrant, new FrameOrientation(worldFrame));
         solePositionEstimate.set(robotQuadrant, new FramePoint(worldFrame));
         soleAngularVelocityEstimate.set(robotQuadrant, new FrameVector(worldFrame));
         soleLinearVelocityEstimate.set(robotQuadrant, new FrameVector(worldFrame));
      }
      supportPolygonEstimate = new QuadrupedSupportPolygon();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         supportPolygonEstimate.setFootstep(robotQuadrant, new FramePoint(worldFrame));
      }
      supportCentroidEstimate = new FramePoint(worldFrame);
      supportOrientationEstimate = new FrameOrientation(worldFrame);
      bodyOrientationEstimate = new FrameOrientation(worldFrame);
      bodyPositionEstimate = new FramePoint(worldFrame);
      bodyAngularVelocityEstimate = new FrameVector(worldFrame);
      bodyLinearVelocityEstimate = new FrameVector(worldFrame);
      comPositionEstimate = new FramePoint(worldFrame);
      comVelocityEstimate = new FrameVector(worldFrame);
      dcmPositionEstimate = new FramePoint(worldFrame);
      icpPositionEstimate = new FramePoint(worldFrame);
      comHeightEstimate = parameters.getComHeightNominal();

      // YoVariables
      yoBodyOrientationDesired = new YoFrameOrientation("bodyOrientationDesired", supportFrame, registry);
      yoComHeightDesired = new DoubleYoVariable("comHeightDesired", registry);
      yoSolePositionSetpoint = new QuadrantDependentList<>();
      yoSoleLinearVelocitySetpoint = new QuadrantDependentList<>();
      yoSoleForceFeedforwardSetpoint = new QuadrantDependentList<>();
      yoSoleForceSetpoint = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSolePositionSetpoint.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionSetpoint", worldFrame, registry));
         yoSoleLinearVelocitySetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocitySetpoint", worldFrame, registry));
         yoSoleForceFeedforwardSetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleForceFeedforwardSetpoint", worldFrame, registry));
         yoSoleForceSetpoint.set(robotQuadrant, new YoFrameVector(prefix + "SoleForceSetpoint", worldFrame, registry));
      }
      yoBodyOrientationSetpoint = new YoFrameOrientation("bodyOrientationSetpoint", worldFrame, registry);
      yoBodyAngularVelocitySetpoint = new YoFrameVector("bodyAngularVelocitySetpoint", worldFrame, registry);
      yoBodyTorqueFeedforwardSetpoint = new YoFrameVector("bodyTorqueFeedforwardSetpoint", worldFrame, registry);
      yoBodyTorqueSetpoint = new YoFrameVector("bodyTorqueSetpoint", worldFrame, registry);
      yoIcpPositionSetpoint = new YoFramePoint("icpPositionSetpoint", worldFrame, registry);
      yoCmpPositionSetpoint = new YoFramePoint("cmpPositionSetpoint", worldFrame, registry);
      yoVrpPositionSetpoint = new YoFramePoint("vrpPositionSetpoint", worldFrame, registry);
      yoComForceSetpoint = new YoFrameVector("comForceSetpoint", worldFrame, registry);
      yoComHeightSetpoint = new DoubleYoVariable("comHeightSetpoint", registry);
      yoSoleOrientationEstimate = new QuadrantDependentList<>();
      yoSolePositionEstimate = new QuadrantDependentList<>();
      yoSoleAngularVelocityEstimate = new QuadrantDependentList<>();
      yoSoleLinearVelocityEstimate = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         String prefix = robotQuadrant.getCamelCaseNameForStartOfExpression();
         yoSoleOrientationEstimate.set(robotQuadrant, new YoFrameOrientation(prefix + "SoleOrientationEstimate", worldFrame, registry));
         yoSolePositionEstimate.set(robotQuadrant, new YoFramePoint(prefix + "SolePositionEstimate", worldFrame, registry));
         yoSoleAngularVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleAngularVelocityEstimate", worldFrame, registry));
         yoSoleLinearVelocityEstimate.set(robotQuadrant, new YoFrameVector(prefix + "SoleLinearVelocityEstimate", worldFrame, registry));
      }
      yoSupportPolygonEstimate = new YoFrameConvexPolygon2d("supportPolygon", "", worldFrame, 4, registry);
      yoSupportCentroidEstimate = new YoFramePoint("supportCentroidEstimate", worldFrame, registry);
      yoSupportOrientationEstimate = new YoFrameOrientation("supportOrientationEstimate", worldFrame, registry);
      yoBodyOrientationEstimate = new YoFrameOrientation("bodyOrientationEstimate", worldFrame, registry);
      yoBodyPositionEstimate = new YoFramePoint("bodyPositionEstimate", worldFrame, registry);
      yoBodyAngularVelocityEstimate = new YoFrameVector("bodyAngularVelocityEstimate", worldFrame, registry);
      yoBodyLinearVelocityEstimate = new YoFrameVector("bodyLinearVelocityEstimate", worldFrame, registry);
      yoComPositionEstimate = new YoFramePoint("comPositionEstimate", worldFrame, registry);
      yoComVelocityEstimate = new YoFrameVector("comVelocityEstimate", worldFrame, registry);
      yoIcpPositionEstimate = new YoFramePoint("icpPositionEstimate", worldFrame, registry);
      yoComHeightEstimate = new DoubleYoVariable("comHeightEstimate", registry);

      // YoGraphics
      yoGraphicsList = new YoGraphicsList(getClass().getSimpleName() + "Graphics");
      artifactList = new ArtifactList(getClass().getSimpleName() + "Artifacts");
      registerGraphics();

      runtimeEnvironment.getParentRegistry().addChild(registry);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public YoGraphicsList getYoGraphicsList()
   {
      return yoGraphicsList;
   }

   public ArtifactList getArtifactList()
   {
      return artifactList;
   }

   public boolean addStep(QuadrupedTimedStep quadrupedTimedStep)
   {
      if (quadrupedTimedStep.getTimeInterval().getStartTime() > robotTimestamp.getDoubleValue() && stepQueue.enqueue())
      {

         stepQueue.getTail().set(quadrupedTimedStep);
         return true;
      }
      return false;
   }

   public void removeSteps()
   {
      while(stepQueue.dequeue())
      {
      }
   }

   public int getStepQueueSize()
   {
      return stepQueue.size();
   }


   private void registerGraphics()
   {
      YoGraphicPosition yoComPositionEstimateViz = new YoGraphicPosition("comPositionEstimate", yoComPositionEstimate, 0.025, YoAppearance.Black(), GraphicType.BALL_WITH_CROSS);
      YoGraphicPosition yoIcpPositionEstimateViz = new YoGraphicPosition("icpPositionEstimate", yoIcpPositionEstimate, 0.025, YoAppearance.Chartreuse());
      YoGraphicPosition yoIcpPositionSetpointViz = new YoGraphicPosition("icpPositionSetpoint", yoIcpPositionSetpoint, 0.025, YoAppearance.Blue());
      YoGraphicPosition yoCmpPositionSetpointViz = new YoGraphicPosition("cmpPositionSetpoint", yoCmpPositionSetpoint, 0.025, YoAppearance.Magenta());
      yoGraphicsList.add(yoComPositionEstimateViz);
      yoGraphicsList.add(yoIcpPositionEstimateViz);
      yoGraphicsList.add(yoIcpPositionSetpointViz);
      yoGraphicsList.add(yoCmpPositionSetpointViz);
      yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

      YoArtifactPolygon yoSupportPolygonArtifact = new YoArtifactPolygon("supportPolygon", yoSupportPolygonEstimate, Color.BLACK, false);
      artifactList.add(yoComPositionEstimateViz.createArtifact());
      artifactList.add(yoIcpPositionEstimateViz.createArtifact());
      artifactList.add(yoIcpPositionSetpointViz.createArtifact());
      artifactList.add(yoCmpPositionSetpointViz.createArtifact());
      artifactList.add(yoSupportPolygonArtifact);
      yoGraphicsListRegistry.registerArtifactList(artifactList);
   }

   private void handleStepEvents()
   {
      if (stepQueue.size() > 0)
      {
         QuadrupedTimedStep currentStep = stepQueue.getHead();
         double currentTime = robotTimestamp.getDoubleValue();
         double currentStepStartTime = currentStep.getTimeInterval().getStartTime();
         RobotQuadrant robotQuadrant = currentStep.getRobotQuadrant();
         if (currentTime > currentStepStartTime)
         {
            // dequeue step
            stepCache.get(robotQuadrant).set(currentStep);
            if (footStateMachine.get(robotQuadrant).getCurrentStateEnum() == FootState.SUPPORT_STATE)
            {
               footStateMachine.get(robotQuadrant).setCurrentState(FootState.SWING_STATE);
            }
            stepQueue.dequeue();
         }
      }
   }

   private void updateEstimates()
   {
      Twist twist = pool.lease(Twist.class);

      // update frames and twists
      referenceFrames.updateFrames();
      twistCalculator.compute();
      comJacobian.compute();

      // compute sole poses and twists
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         twistCalculator.packTwistOfBody(twist, footRigidBody.get(robotQuadrant));
         twist.changeFrame(soleFrame.get(robotQuadrant));
         twist.packAngularPart(soleAngularVelocityEstimate.get(robotQuadrant));
         twist.packLinearPart(soleLinearVelocityEstimate.get(robotQuadrant));
         soleOrientationEstimate.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         solePositionEstimate.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
      }

      // compute support polygon
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionEstimate.get(robotQuadrant).changeFrame(supportPolygonEstimate.getReferenceFrame());
         supportPolygonEstimate.setFootstep(robotQuadrant, solePositionEstimate.get(robotQuadrant));
      }

      // compute support frame (centroid and nominal orientation)
      supportCentroidEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportOrientationEstimate.changeFrame(supportPolygonEstimate.getReferenceFrame());
      supportPolygonEstimate.getCentroid2d(supportCentroidEstimate);
      supportOrientationEstimate.setYawPitchRoll(supportPolygonEstimate.getNominalYaw(), 0, 0);
      supportFrame.setPoseAndUpdate(supportCentroidEstimate, supportOrientationEstimate);

      // compute body pose and twist
      twistCalculator.packTwistOfBody(twist, fullRobotModel.getPelvis());
      twist.changeFrame(bodyFrame);
      twist.packAngularPart(bodyAngularVelocityEstimate);
      twist.packLinearPart(bodyLinearVelocityEstimate);
      bodyOrientationEstimate.setToZero(bodyFrame);
      bodyPositionEstimate.setToZero(bodyFrame);

      // compute center of mass position and velocity
      comPositionEstimate.setToZero(comFrame);
      comJacobian.packCenterOfMassVelocity(comVelocityEstimate);

      // compute divergent component of motion and instantaneous capture point
      comPositionEstimate.changeFrame(worldFrame);
      comVelocityEstimate.changeFrame(worldFrame);
      dcmPositionEstimate.changeFrame(worldFrame);
      double omega = dcmPositionController.getNaturalFrequency();
      dcmPositionEstimate.setX(comPositionEstimate.getX() + comVelocityEstimate.getX() / omega);
      dcmPositionEstimate.setY(comPositionEstimate.getY() + comVelocityEstimate.getY() / omega);
      dcmPositionEstimate.setZ(comPositionEstimate.getZ() + comVelocityEstimate.getZ() / omega);
      icpPositionEstimate.setIncludingFrame(dcmPositionEstimate);
      icpPositionEstimate.add(0, 0, -dcmPositionController.getComHeight());

      // compute center of mass height
      comPositionEstimate.changeFrame(worldFrame);
      supportCentroidEstimate.changeFrame(worldFrame);
      comHeightEstimate = comPositionEstimate.getZ() - supportCentroidEstimate.getZ();
   }

   private void updateSetpoints()
   {
      // compute capture point natural frequency
      double comHeight = Math.max(comHeightSetpoint, parameters.getComHeightNominal() / 5);
      dcmPositionController.setComHeight(comHeight);

      // compute body torque setpoints to track desired body orientation
      bodyTorqueSetpoint.changeFrame(bodyFrame);
      bodyOrientationSetpoint.setIncludingFrame(bodyOrientationDesired);
      bodyOrientationSetpoint.changeFrame(bodyFrame);
      bodyAngularVelocitySetpoint.setToZero(bodyFrame);
      bodyAngularVelocityEstimate.changeFrame(bodyFrame);
      bodyTorqueFeedforwardSetpoint.setToZero(bodyFrame);
      bodyOrientationController.compute(bodyTorqueSetpoint, bodyOrientationSetpoint, bodyAngularVelocitySetpoint, bodyAngularVelocityEstimate, bodyTorqueFeedforwardSetpoint);

      // compute horizontal forces to track desired instantaneous capture point
      icpPositionSetpoint.setIncludingFrame(supportCentroidEstimate);
      icpVelocitySetpoint.setToZero(supportFrame);
      dcmPositionSetpoint.setIncludingFrame(icpPositionSetpoint);
      dcmPositionSetpoint.add(0, 0, dcmPositionController.getComHeight());
      dcmVelocitySetpoint.setIncludingFrame(icpVelocitySetpoint);
      dcmPositionController.compute(comForceSetpoint, vrpPositionSetpoint, cmpPositionSetpoint, dcmPositionSetpoint, dcmVelocitySetpoint, dcmPositionEstimate);

      // compute vertical force to track desired center of mass height
      comHeightSetpoint = comHeightDesired;
      double comForceZ = parameters.getComHeightGravityFeedforwardConstant() * mass * gravity + comHeightController.compute(comHeightEstimate, comHeightSetpoint, comVelocityEstimate.getZ(), 0, controlDT);
      comForceSetpoint.changeFrame(worldFrame);
      comForceSetpoint.setZ(comForceZ);

      // compute virtual forces to track swing foot trajectories
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).checkTransitionConditions();
         footStateMachine.get(robotQuadrant).doAction();
      }

      // compute joint torques using virtual model control
      virtualModelController.setComForceCommand(comForceSetpoint);
      virtualModelController.setComTorqueCommand(bodyTorqueSetpoint);
      virtualModelController.compute(jointLimits, contactForceLimits);
   }

   private void readYoVariables()
   {
      yoBodyOrientationDesired.getFrameOrientationIncludingFrame(bodyOrientationDesired);
      comHeightDesired = yoComHeightDesired.getDoubleValue();
   }

   private void writeYoVariables()
   {
      // update frames
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.get(robotQuadrant).changeFrame(yoSolePositionSetpoint.get(robotQuadrant).getReferenceFrame());
         soleLinearVelocitySetpoint.get(robotQuadrant).changeFrame(yoSoleLinearVelocitySetpoint.get(robotQuadrant).getReferenceFrame());
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(yoSoleForceFeedforwardSetpoint.get(robotQuadrant).getReferenceFrame());
         soleForceSetpoint.get(robotQuadrant).changeFrame(yoSoleForceSetpoint.get(robotQuadrant).getReferenceFrame());
      }
      bodyOrientationSetpoint.changeFrame(yoBodyOrientationSetpoint.getReferenceFrame());
      bodyAngularVelocitySetpoint.changeFrame(yoBodyAngularVelocitySetpoint.getReferenceFrame());
      bodyTorqueFeedforwardSetpoint.changeFrame(yoBodyTorqueFeedforwardSetpoint.getReferenceFrame());
      bodyTorqueSetpoint.changeFrame(yoBodyTorqueSetpoint.getReferenceFrame());
      icpPositionSetpoint.changeFrame(yoIcpPositionSetpoint.getReferenceFrame());
      cmpPositionSetpoint.changeFrame(yoCmpPositionSetpoint.getReferenceFrame());
      vrpPositionSetpoint.changeFrame(yoVrpPositionSetpoint.getReferenceFrame());
      comForceSetpoint.changeFrame(yoComForceSetpoint.getReferenceFrame());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleOrientationEstimate.get(robotQuadrant).changeFrame(yoSoleOrientationEstimate.get(robotQuadrant).getReferenceFrame());
         solePositionEstimate.get(robotQuadrant).changeFrame(yoSolePositionEstimate.get(robotQuadrant).getReferenceFrame());
         soleAngularVelocityEstimate.get(robotQuadrant).changeFrame(yoSoleAngularVelocityEstimate.get(robotQuadrant).getReferenceFrame());
         soleLinearVelocityEstimate.get(robotQuadrant).changeFrame(yoSoleLinearVelocityEstimate.get(robotQuadrant).getReferenceFrame());
      }
      supportCentroidEstimate.changeFrame(yoSupportCentroidEstimate.getReferenceFrame());
      supportOrientationEstimate.changeFrame(yoSupportOrientationEstimate.getReferenceFrame());
      bodyOrientationEstimate.changeFrame(yoBodyOrientationEstimate.getReferenceFrame());
      bodyPositionEstimate.changeFrame(yoBodyPositionEstimate.getReferenceFrame());
      bodyAngularVelocityEstimate.changeFrame(yoBodyAngularVelocityEstimate.getReferenceFrame());
      bodyLinearVelocityEstimate.changeFrame(yoBodyLinearVelocityEstimate.getReferenceFrame());
      comPositionEstimate.changeFrame(yoComPositionEstimate.getReferenceFrame());
      comVelocityEstimate.changeFrame(yoComVelocityEstimate.getReferenceFrame());
      icpPositionEstimate.changeFrame(yoIcpPositionEstimate.getReferenceFrame());

      // copy variables
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionSetpoint.get(robotQuadrant).changeFrame(yoSolePositionSetpoint.get(robotQuadrant).getReferenceFrame());
         soleLinearVelocitySetpoint.get(robotQuadrant).changeFrame(yoSoleLinearVelocitySetpoint.get(robotQuadrant).getReferenceFrame());
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(yoSoleForceFeedforwardSetpoint.get(robotQuadrant).getReferenceFrame());
         soleForceSetpoint.get(robotQuadrant).changeFrame(yoSoleForceSetpoint.get(robotQuadrant).getReferenceFrame());
      }
      yoBodyOrientationSetpoint.set(bodyOrientationSetpoint);
      yoBodyAngularVelocitySetpoint.set(bodyAngularVelocitySetpoint);
      yoBodyTorqueFeedforwardSetpoint.set(bodyTorqueFeedforwardSetpoint);
      yoBodyTorqueSetpoint.set(bodyTorqueSetpoint);
      yoIcpPositionSetpoint.set(icpPositionSetpoint);
      yoCmpPositionSetpoint.set(cmpPositionSetpoint);
      yoVrpPositionSetpoint.set(vrpPositionSetpoint);
      yoComForceSetpoint.set(comForceSetpoint);
      yoComHeightSetpoint.set(comHeightSetpoint);

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         yoSoleOrientationEstimate.get(robotQuadrant).set(soleOrientationEstimate.get(robotQuadrant));
         yoSolePositionEstimate.get(robotQuadrant).set(solePositionEstimate.get(robotQuadrant));
         yoSoleAngularVelocityEstimate.get(robotQuadrant).set(soleAngularVelocityEstimate.get(robotQuadrant));
         yoSoleLinearVelocityEstimate.get(robotQuadrant).set(soleLinearVelocityEstimate.get(robotQuadrant));
      }
      supportPolygonEstimate.packYoFrameConvexPolygon2d(yoSupportPolygonEstimate);
      yoSupportCentroidEstimate.set(supportCentroidEstimate);
      yoSupportOrientationEstimate.set(supportOrientationEstimate);
      yoBodyOrientationEstimate.set(bodyOrientationEstimate);
      yoBodyAngularVelocityEstimate.set(bodyAngularVelocityEstimate);
      yoBodyLinearVelocityEstimate.set(bodyLinearVelocityEstimate);
      yoComPositionEstimate.set(comPositionEstimate);
      yoComVelocityEstimate.set(comVelocityEstimate);
      yoIcpPositionEstimate.set(icpPositionEstimate);
      yoComHeightEstimate.set(comHeightEstimate);
   }

   @Override public QuadrupedControllerEvent process()
   {
      pool.evict();
      readYoVariables();
      handleStepEvents();
      updateEstimates();
      updateSetpoints();
      writeYoVariables();
      return null;
   }

   @Override public void onEntry()
   {
      // initialize desired values (provider inputs)
      yoBodyOrientationDesired.setYawPitchRoll(0.0, 0.0, 0.0);
      yoComHeightDesired.set(parameters.getComHeightNominal());

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         for (int i = 0; i < jointNameMap.getLegJointNames().length; i++)
         {
            // initialize leg joint mode to force control
            LegJointName legJointName = jointNameMap.getLegJointNames()[i];
            String jointName = jointNameMap.getLegJointName(robotQuadrant, legJointName);
            OneDoFJoint joint = fullRobotModel.getOneDoFJointByName(jointName);
            joint.setUnderPositionControl(false);
         }
      }

      // initialize controllers and state machines
      virtualModelController.reinitialize();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         virtualModelController.setContactState(robotQuadrant, true);
      }
      bodyOrientationController.reset();
      bodyOrientationController.setProportionalGains(parameters.getBodyOrientationProportionalGains());
      bodyOrientationController.setIntegralGains(parameters.getBodyOrientationIntegralGains(), parameters.getBodyOrientationMaxIntegralError());
      bodyOrientationController.setDerivativeGains(parameters.getBodyOrientationDerivativeGains());
      comHeightController.resetIntegrator();
      comHeightController.setProportionalGain(parameters.getComHeightProportionalGain());
      comHeightController.setIntegralGain(parameters.getComHeightIntegralGain());
      comHeightController.setMaxIntegralError(parameters.getComHeightMaxIntegralError());
      comHeightController.setDerivativeGain(parameters.getComHeightDerivativeGain());
      dcmPositionController.reset();
      dcmPositionController.setProportionalGains(parameters.getDcmProportionalGains());
      dcmPositionController.setIntegralGains(parameters.getDcmIntegralGains(), parameters.getDcmMaxIntegralError());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footStateMachine.get(robotQuadrant).setCurrentState(FootState.SUPPORT_STATE);
         swingPositionController.get(robotQuadrant).setProportionalGains(parameters.getSwingPositionProportionalGains());
         swingPositionController.get(robotQuadrant).setIntegralGains(parameters.getSwingPositionIntegralGains(), parameters.getSwingPositionMaxIntegralError());
         swingPositionController.get(robotQuadrant).setDerivativeGains(parameters.getSwingPositionDerivativeGains());
      }

      // show graphics
      yoGraphicsListRegistry.hideYoGraphics();
      yoGraphicsListRegistry.hideArtifacts();
      yoGraphicsList.setVisible(true);
      artifactList.setVisible(true);
      virtualModelController.setVisible(true);
   }

   @Override public void onExit()
   {
      // hide graphics
      yoGraphicsList.setVisible(false);
      artifactList.setVisible(false);
      virtualModelController.setVisible(false);
   }

   private class SwingState extends State<FootState>
   {
      private final RobotQuadrant robotQuadrant;

      public SwingState(FootState stateEnum, RobotQuadrant robotQuadrant)
      {
         super(stateEnum);
         this.robotQuadrant = robotQuadrant;

         // configure state transitions
         addStateTransition(new StateTransition<>(FootState.SUPPORT_STATE, new SwingToSupportCondition()));
      }

      @Override public void doAction()
      {
         // compute sole force setpoints to track swing trajectory
         soleForceSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         solePositionSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         soleLinearVelocitySetpoint.get(robotQuadrant).setToZero(soleFrame.get(robotQuadrant));
         soleLinearVelocityEstimate.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(worldFrame);
         soleForceFeedforwardSetpoint.get(robotQuadrant).set(0, 0, parameters.getSwingPositionGravityFeedforwardForce());
         soleForceFeedforwardSetpoint.get(robotQuadrant).changeFrame(soleFrame.get(robotQuadrant));
         swingPositionController.get(robotQuadrant)
               .compute(soleForceSetpoint.get(robotQuadrant), solePositionSetpoint.get(robotQuadrant), soleLinearVelocitySetpoint.get(robotQuadrant), soleLinearVelocityEstimate.get(robotQuadrant), soleForceFeedforwardSetpoint.get(robotQuadrant));
         virtualModelController.setSwingForceCommand(robotQuadrant, soleForceSetpoint.get(robotQuadrant));
      }

      @Override public void doTransitionIntoAction()
      {
         // initialize controllers
         swingPositionController.get(robotQuadrant).reset();

         // initialize contact state
         virtualModelController.setContactState(robotQuadrant, false);
      }

      @Override public void doTransitionOutOfAction()
      {
         virtualModelController.setContactState(robotQuadrant, true);
      }

      private class SwingToSupportCondition implements StateTransitionCondition
      {
         @Override public boolean checkCondition()
         {
            // detect if contact has occurred
            return false;
         }
      }
   }

   private class SupportState extends State<FootState>
   {
      private final RobotQuadrant robotQuadrant;

      public SupportState(FootState stateEnum, RobotQuadrant robotQuadrant)
      {
         super(stateEnum);
         this.robotQuadrant = robotQuadrant;
      }

      @Override public void doAction()
      {
      }

      @Override public void doTransitionIntoAction()
      {
      }

      @Override public void doTransitionOutOfAction()
      {

      }
   }

}
