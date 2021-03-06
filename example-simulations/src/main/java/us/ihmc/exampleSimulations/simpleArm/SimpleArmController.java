package us.ihmc.exampleSimulations.simpleArm;


import java.util.Random;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.algorithms.InverseDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Random controller to test state estimation.
 */
public class SimpleArmController extends SimpleRobotController
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final SimpleRobotInputOutputMap robot;
   private final YoDouble time;

   private static final Random random = new Random(94929438248L);
   private YoDouble noiseMagnitude = new YoDouble("NoiseMagnitude", registry);
   private YoDouble decay = new YoDouble("Decay", registry);
   private YoDouble frequency = new YoDouble("Frequency", registry);
   private YoDouble magnitude = new YoDouble("Magnitude", registry);
   private YoDouble offset = new YoDouble("Offset", registry);

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private RigidBodyBasics endEffectorBody;

   public SimpleArmController(SimpleRobotInputOutputMap robot, RigidBodyBasics endEffectorBody, YoDouble time)
   {
      this.endEffectorBody = endEffectorBody;
      this.time = time;
      this.robot = robot;

      noiseMagnitude.set(0.5);
      decay.set(0.05);
      frequency.set(0.5);

      inverseDynamicsCalculator = new InverseDynamicsCalculator(endEffectorBody);
      inverseDynamicsCalculator.setGravitionalAcceleration(-SimpleArmRobot.gravity);
   }

   @Override
   public void doControl()
   {
      robot.readFromSimulation();

      magnitude.add(noiseMagnitude.getDoubleValue() * (random.nextDouble() - 0.5));
      offset.add(noiseMagnitude.getDoubleValue() * (random.nextDouble() - 0.5));

      double angle = 2.0 * Math.PI * frequency.getDoubleValue() * time.getDoubleValue();
      double randomTorque = magnitude.getDoubleValue() * Math.sin(angle) + offset.getDoubleValue();

      inverseDynamicsCalculator.compute();
      endEffectorBody.childrenSubtreeIterable().forEach(inverseDynamicsCalculator::writeComputedJointWrench);

      robot.addYawTorque(randomTorque);
      robot.addPitch1Torque(randomTorque);
      robot.addPitch2Torque(randomTorque);

      magnitude.mul(1.0 - decay.getDoubleValue());
      offset.mul(1.0 - decay.getDoubleValue());

      robot.writeToSimulation();
   }

}
