package us.ihmc.commonWalkingControlModules.capturePoint;

import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * Command that holds input for the {@link LinearMomentumRateControlModule} coming from the walking controller state
 * machine that might be running at a slower rate then the ICP feedback.
 *
 * @author Georg Wiedebach
 */
public class LinearMomentumRateControlModuleInput
{
   /**
    * The time constant for the LIPM model. This might be changed during a run so it is a parameter. Typically the value
    * is {@code omega0 = sqrt(gravity / comZ)}.
    */
   private double omega0;

   /**
    * The desired capture point that the ICP controller should track.
    */
   private final FramePoint2D desiredCapturePoint = new FramePoint2D();

   /**
    * The desired capture point velocity that the ICP controller should track.
    */
   private final FrameVector2D desiredCapturePointVelocity = new FrameVector2D();

   /**
    * Assuming to tracking error for the ICP this is the location that the CMP should be placed at according to the ICP
    * plan.
    */
   private final FixedFramePoint2DBasics perfectCMP = new FramePoint2D();

   /**
    * Assuming to tracking error for the ICP this is the location that the CoP should be placed at according to the ICP
    * plan.
    */
   private final FixedFramePoint2DBasics perfectCoP = new FramePoint2D();

   /**
    * Is a flag that enables the z-selection in the linear momentum rate command if {@code true}.
    */
   private boolean controlHeightWithMomentum;

   @Deprecated // The CoM height control should be moved to the fast thread or this should use the achieved value from the last tick.
   private double desiredCoMHeightAcceleration = 0.0;

   /**
    * Indicates which foot will be in support when stepping. Note, that this will only be used if
    * {@link #initializeForSingleSupport} is set to {@code true}.
    */
   private RobotSide supportSide = null;

   /**
    * Indicates which foot the robot shifts its weight to when performing a transfer. This will usually be the upcoming
    * support side. Note, that this will only be used if {@link #initializeForSingleSupport} is set to {@code true}.
    */
   private RobotSide transferToSide = null;

   /**
    * Flag that indicates the ICP planner has just transitioned to a standing state. This causes the ICP controller to
    * to initialize itself for standing.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForStanding;

   /**
    * Flag that indicates the ICP planner has just transitioned to a single support state. This causes the ICP
    * controller to to initialize itself for single support.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForSingleSupport;

   /**
    * Flag that indicates the ICP planner has just transitioned to a transfer state. This causes the ICP controller to
    * to initialize itself for transfer.
    * <p>
    * Note, that this should only be true for one tick. Also, only one of the flags {@link #initializeForStanding},
    * {@link #initializeForSingleSupport}, and {@link #initializeForTransfer} can be true at a time.
    */
   private boolean initializeForTransfer;

   /**
    * Flag that indicates to the ICP controller that the desired feedback CoP should stay within the bounds of the
    * support polygon. This should generally be true but can be set to false in certain cases where the support polygon
    * might not be accurate e.g. when using handholds.
    */
   private boolean keepCoPInsideSupportPolygon;

   /**
    * Is a flag that enables the z-selection in the angular momentum rate command if {@code true}. The desired angular
    * momentum will generally be zero.
    */
   private boolean minimizeAngularMomentumRateZ;

   /**
    * List of upcoming footsteps that are being executed by the controller. This is of interest to the ICP controller
    * because it might consider n-step capturability or adjust the locations of the upcoming footsteps when needed.
    * <p>
    * The fields {@link #footsteps}, {@link #footstepTimings}, and {@link #finalTransferDuration} shopuld always be set
    * together. Note, that they will only be used if one of the initialize flags is set to {@code true}
    */
   private final RecyclingArrayList<Footstep> footsteps = new RecyclingArrayList<>(Footstep.class);

   /**
    * List of upcoming footstep timings that are being executed by the controller. This is of interest to the ICP
    * controller because it might consider n-step capturability or adjust the locations of the upcoming footsteps when
    * needed.
    * <p>
    * The fields {@link #footsteps}, {@link #footstepTimings}, and {@link #finalTransferDuration} shopuld always be set
    * together. Note, that they will only be used if one of the initialize flags is set to {@code true}
    */
   private final RecyclingArrayList<FootstepTiming> footstepTimings = new RecyclingArrayList<>(FootstepTiming.class);

   /**
    * The final transfer duration for a footstep plan. This is a separate field since footsteps only hold the time to
    * transfer to and the time to swing. Hence, the time for the final transfer back to standing is not contained in the
    * list of {@link #footstepTimings}.
    * <p>
    * The fields {@link #footsteps}, {@link #footstepTimings}, and {@link #finalTransferDuration} shopuld always be set
    * together. Note, that they will only be used if one of the initialize flags is set to {@code true}
    */
   private double finalTransferDuration;

   /**
    * This field can be used to inform the ICP controller that the walking state machine has sped up the swing of the
    * robot. This can happen under disturbances.
    * <p>
    * This value can be set to {@code NaN} or to {@code <= 0.0} to indicate that no swing speed up has been performed.
    */
   private double remainingTimeInSwingUnderDisturbance;

   /**
    * The contact state of the robot. Effectively updates the support polygon for the ICP feedback controller.
    */
   private final SideDependentList<PlaneContactStateCommand> contactStateCommands = new SideDependentList<>(new PlaneContactStateCommand(),
                                                                                                            new PlaneContactStateCommand());

   public void setOmega0(double omega0)
   {
      this.omega0 = omega0;
   }

   public double getOmega0()
   {
      return omega0;
   }

   public void setDesiredCapturePoint(FramePoint2DReadOnly desiredCapturePoint)
   {
      this.desiredCapturePoint.setIncludingFrame(desiredCapturePoint);
   }

   public FramePoint2DReadOnly getDesiredCapturePoint()
   {
      return desiredCapturePoint;
   }

   public void setDesiredCapturePointVelocity(FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      this.desiredCapturePointVelocity.setIncludingFrame(desiredCapturePointVelocity);
   }

   public FrameVector2DReadOnly getDesiredCapturePointVelocity()
   {
      return desiredCapturePointVelocity;
   }

   @Deprecated // TODO: This should not be coming from the walking controller.
   public void setDesiredCenterOfMassHeightAcceleration(double desiredCoMHeightAcceleration)
   {
      this.desiredCoMHeightAcceleration = desiredCoMHeightAcceleration;
   }

   public double getDesiredCoMHeightAcceleration()
   {
      return desiredCoMHeightAcceleration;
   }

   public void setMinimizeAngularMomentumRateZ(boolean minimizeAngularMomentumRateZ)
   {
      this.minimizeAngularMomentumRateZ = minimizeAngularMomentumRateZ;
   }

   public boolean getMinimizeAngularMomentumRateZ()
   {
      return minimizeAngularMomentumRateZ;
   }

   public void setPerfectCMP(FramePoint2DReadOnly perfectCMP)
   {
      this.perfectCMP.setMatchingFrame(perfectCMP);
   }

   public FramePoint2DReadOnly getPerfectCMP()
   {
      return perfectCMP;
   }

   public void setPerfectCoP(FramePoint2DReadOnly perfectCoP)
   {
      this.perfectCoP.setMatchingFrame(perfectCoP);
   }

   public FramePoint2DReadOnly getPerfectCoP()
   {
      return perfectCoP;
   }

   public void setControlHeightWithMomentum(boolean controlHeightWithMomentum)
   {
      this.controlHeightWithMomentum = controlHeightWithMomentum;
   }

   public boolean getControlHeightWithMomentum()
   {
      return controlHeightWithMomentum;
   }

   public void setSupportSide(RobotSide supportSide)
   {
      this.supportSide = supportSide;
   }

   public RobotSide getSupportSide()
   {
      return supportSide;
   }

   public void setTransferToSide(RobotSide transferToSide)
   {
      this.transferToSide = transferToSide;
   }

   public RobotSide getTransferToSide()
   {
      return transferToSide;
   }

   public void setFootsteps(List<Footstep> footsteps)
   {
      this.footsteps.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footsteps.add().set(footsteps.get(i));
      }
   }

   public List<Footstep> getFootsteps()
   {
      return footsteps;
   }

   public void setFootstepTimings(List<FootstepTiming> footstepTimings)
   {
      this.footstepTimings.clear();
      for (int i = 0; i < footsteps.size(); i++)
      {
         this.footstepTimings.add().set(footstepTimings.get(i));
      }
   }

   public List<FootstepTiming> getFootstepTimings()
   {
      return footstepTimings;
   }

   public void setFinalTransferDuration(double finalTransferDuration)
   {
      this.finalTransferDuration = finalTransferDuration;
   }

   public double getFinalTransferDuration()
   {
      return finalTransferDuration;
   }

   public void setInitializeForStanding(boolean initializeForStanding)
   {
      this.initializeForStanding = initializeForStanding;
   }

   public boolean getInitializeForStanding()
   {
      return initializeForStanding;
   }

   public void setInitializeForSingleSupport(boolean initializeForSingleSupport)
   {
      this.initializeForSingleSupport = initializeForSingleSupport;
   }

   public boolean getInitializeForSingleSupport()
   {
      return initializeForSingleSupport;
   }

   public void setInitializeForTransfer(boolean initializeForTransfer)
   {
      this.initializeForTransfer = initializeForTransfer;
   }

   public boolean getInitializeForTransfer()
   {
      return initializeForTransfer;
   }

   public void setRemainingTimeInSwingUnderDisturbance(double remainingTimeInSwingUnderDisturbance)
   {
      this.remainingTimeInSwingUnderDisturbance = remainingTimeInSwingUnderDisturbance;
   }

   public double getRemainingTimeInSwingUnderDisturbance()
   {
      return remainingTimeInSwingUnderDisturbance;
   }

   public void setKeepCoPInsideSupportPolygon(boolean keepCoPInsideSupportPolygon)
   {
      this.keepCoPInsideSupportPolygon = keepCoPInsideSupportPolygon;
   }

   public boolean getKeepCoPInsideSupportPolygon()
   {
      return keepCoPInsideSupportPolygon;
   }

   public void setContactStateCommand(SideDependentList<PlaneContactStateCommand> contactStateCommands)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         this.contactStateCommands.get(robotSide).set(contactStateCommands.get(robotSide));
      }
   }

   public SideDependentList<PlaneContactStateCommand> getContactStateCommands()
   {
      return contactStateCommands;
   }
}