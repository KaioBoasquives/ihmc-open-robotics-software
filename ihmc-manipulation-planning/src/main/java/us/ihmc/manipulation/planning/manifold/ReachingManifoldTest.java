package us.ihmc.manipulation.planning.manifold;

import java.util.Random;

import controller_msgs.msg.dds.ReachingManifoldMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.rotationConversion.YawPitchRollConversion;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ReachingManifoldTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double trajectoryTime = 0.05;
   private final double dt = 0.001;
   private final int recordFrequency = 1;
   private final int bufferSize = (int) (trajectoryTime / dt / recordFrequency + 2);

   private final YoDouble currentTrajectoryTime = new YoDouble("CurrentTime", registry);

   private final Random random = new Random(1);

   // define manifold
   private final Point3D manifoldOriginPosition = new Point3D(0.7, -0.2, 1.0);
   private final RotationMatrix manifoldOriginOrientation = new RotationMatrix();

   // define initial hand configuration
   private final Point3D initialHandPosition = new Point3D(0.1, -0.4, 0.8);
   private final double[] handLowerLimits = new double[] {1.0, 0.5, 0.4};
   private final double[] handUpperLimits = new double[] {-0.2, -0.3, -0.2};

   public ReachingManifoldTest()
   {
      // scs
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setCreateGUI(true);
      parameters.setDataBufferSize(bufferSize);
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("voidrobot"), parameters);

      // hand frame
      RigidBody hand = new RigidBody("hand", new RigidBodyTransform(), worldFrame);

      // create manifold message

      ReachingManifoldMessage reachingManifoldMessage = ReachingManifoldTools.createSphereManifoldMessage(hand, manifoldOriginPosition, 0.2);
      // ReachingManifoldMessage reachingManifoldMessage = ReachingManifoldTools.createCylinderManifoldMessage(hand, manifoldOriginPosition, manifoldOriginOrientation, 0.2, 0.3);
      // ReachingManifoldMessage reachingManifoldMessage = ReachingManifoldTools.createBoxManifoldMessage(hand, manifoldOriginPosition, manifoldOriginOrientation, 0.1, 0.2, 0.05);
      // ReachingManifoldMessage reachingManifoldMessage = ReachingManifoldTools.createTorusManifoldMessage(hand, manifoldOriginPosition, manifoldOriginOrientation, 0.4, 0.1);

      // yographics 
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      // graphics for manifold message
      scs.addStaticLinkGraphics(ReachingManifoldTools.createManifoldMessageStaticGraphic(reachingManifoldMessage, 0.005, 20));

      // graphics for hand frame
      YoDouble yoPXHand = new YoDouble("yoPXHand", registry);
      YoDouble yoPYHand = new YoDouble("yoPYHand", registry);
      YoDouble yoPZHand = new YoDouble("yoPZHand", registry);
      YoDouble yoYawHand = new YoDouble("yoYawHand", registry);
      YoDouble yoPitchHand = new YoDouble("yoPitchHand", registry);
      YoDouble yoRollHand = new YoDouble("yoRollHand", registry);

      yoGraphicsListRegistry.registerYoGraphic("handViz",
                                               new YoGraphicCoordinateSystem("handViz", yoPXHand, yoPYHand, yoPZHand, yoYawHand, yoPitchHand, yoRollHand, 0.2));

      // graphics for closest frame
      YoDouble yoPX = new YoDouble("yoPX", registry);
      YoDouble yoPY = new YoDouble("yoPY", registry);
      YoDouble yoPZ = new YoDouble("yoPZ", registry);
      YoDouble yoYaw = new YoDouble("yoYaw", registry);
      YoDouble yoPitch = new YoDouble("yoPitch", registry);
      YoDouble yoRoll = new YoDouble("yoRoll", registry);

      yoGraphicsListRegistry.registerYoGraphic("handViz", new YoGraphicCoordinateSystem("closestPointViz", yoPX, yoPY, yoPZ, yoYaw, yoPitch, yoRoll, 0.2));

      // run scs
      scs.addYoVariableRegistry(registry);
      scs.setDT(dt, recordFrequency);
      Graphics3DObject worldFrameGraphics = new Graphics3DObject();
      worldFrameGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(worldFrameGraphics);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry, true);

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         currentTrajectoryTime.set(t);

         // random hand position
         yoPXHand.set(initialHandPosition.getX() + random(handLowerLimits[0], handUpperLimits[0]));
         yoPYHand.set(initialHandPosition.getY() + random(handLowerLimits[1], handUpperLimits[1]));
         yoPZHand.set(initialHandPosition.getZ() + random(handLowerLimits[2], handUpperLimits[2]));

         // random hand orientation
         double[] randomYPR = new double[3];
         Quaternion randomQuat = new Quaternion();

         double s = random(0, 1);
         double s1 = Math.sqrt(1 - s);
         double s2 = Math.sqrt(s);

         double theta1 = Math.PI * 2 * random(0, 1);
         double theta2 = Math.PI * 2 * random(0, 1);

         randomQuat.set(Math.sin(theta1) * s1, Math.cos(theta1) * s1, Math.sin(theta2) * s2, Math.cos(theta2) * s2);
         randomQuat.norm();

         YawPitchRollConversion.convertQuaternionToYawPitchRoll(randomQuat, randomYPR);

         yoYawHand.set(randomYPR[0]);
         yoPitchHand.set(randomYPR[1]);
         yoRollHand.set(randomYPR[2]);

         // find closest frame
         RigidBodyTransform handTransform = new RigidBodyTransform();
         handTransform.appendTranslation(yoPXHand.getDoubleValue(), yoPYHand.getDoubleValue(), yoPZHand.getDoubleValue());
         handTransform.setRotationYawPitchRoll(yoYawHand.getDoubleValue(), yoPitchHand.getDoubleValue(), yoRollHand.getDoubleValue());

         RigidBodyTransform closestTransform = new RigidBodyTransform();
         ReachingManifoldTools.packClosestRigidBodyTransformOnManifold(reachingManifoldMessage, handTransform, closestTransform);

         yoPX.set(closestTransform.getTranslationX());
         yoPY.set(closestTransform.getTranslationY());
         yoPZ.set(closestTransform.getTranslationZ());

         double[] closestYPR = new double[3];
         YawPitchRollConversion.convertMatrixToYawPitchRoll(closestTransform.getRotationMatrix(), closestYPR);

         yoYaw.set(closestYPR[0]);
         yoPitch.set(closestYPR[1]);
         yoRoll.set(closestYPR[2]);

         // update scs
         scs.tickAndUpdate();
      }

      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new ReachingManifoldTest();
   }

   private double random(double lowerLimit, double upperLimit)
   {
      return random.nextDouble() * (upperLimit - lowerLimit) + lowerLimit;
   }
}