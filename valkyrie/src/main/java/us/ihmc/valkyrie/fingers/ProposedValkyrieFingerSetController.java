package us.ihmc.valkyrie.fingers;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.math.trajectories.waypoints.MultipleWaypointsTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.waypoints.SimpleTrajectoryPoint1D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class ProposedValkyrieFingerSetController<T extends Enum<T>> implements RobotController
{
   private final String name;
   private final YoVariableRegistry registry;

   private final RobotSide robotSide;

   private final EnumMap<T, SettableDoubleProvider> fingerControlSpace;
   private final EnumMap<T, MultipleWaypointsTrajectoryGenerator> trajectoryGenerators;

   private final List<T> listOfKeySet;

   private final StateMachine<TrajectoryGeneratorMode, State> stateMachine;
   private final YoEnum<TrajectoryGeneratorMode> requestedState;

   enum TrajectoryGeneratorMode
   {
      JOINTSPACE
   }

   public ProposedValkyrieFingerSetController(Class<T> enumType, RobotSide robotSide, YoDouble yoTime, EnumMap<T, YoDouble> fingerControlSpaceMap,
                                              YoVariableRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.name = robotSide.getLowerCaseName() + enumType.getSimpleName();

      this.registry = new YoVariableRegistry(name);

      this.fingerControlSpace = new EnumMap<>(enumType);
      this.trajectoryGenerators = new EnumMap<>(enumType);
      this.listOfKeySet = new ArrayList<T>();
      T[] enumConstants = enumType.getEnumConstants();
      for (int i = 0; i < enumConstants.length; i++)
      {
         T key = enumConstants[i];
         double initialValue = fingerControlSpaceMap.get(key).getDoubleValue();
         fingerControlSpace.put(key, new SettableDoubleProvider(initialValue));

         MultipleWaypointsTrajectoryGenerator trajectoryGenerator = new MultipleWaypointsTrajectoryGenerator(robotSide + key.name() + "_traj", parentRegistry);
         trajectoryGenerator.clear();
         trajectoryGenerator.appendWaypoint(yoTime.getDoubleValue(), initialValue, 0.0);
         trajectoryGenerator.initialize();
         trajectoryGenerators.put(key, trajectoryGenerator);

         listOfKeySet.add(key);
      }

      requestedState = new YoEnum<>(name + "requestedState", registry, TrajectoryGeneratorMode.class, true);
      requestedState.set(null);
      StateMachineFactory<TrajectoryGeneratorMode, State> factory = new StateMachineFactory<>(TrajectoryGeneratorMode.class);

      factory.setNamePrefix(name).setRegistry(registry).buildYoClock(yoTime);

      JointSpaceState stateWorking = new JointSpaceState();

      factory.addState(TrajectoryGeneratorMode.JOINTSPACE, stateWorking);
      factory.addRequestedTransition(TrajectoryGeneratorMode.JOINTSPACE, TrajectoryGeneratorMode.JOINTSPACE, requestedState, false);

      stateMachine = factory.build(TrajectoryGeneratorMode.JOINTSPACE);

      parentRegistry.addChild(registry);
   }

   private class JointSpaceState implements State
   {
      @Override
      public void onEntry()
      {
         for (int i = 0; i < listOfKeySet.size(); i++)
         {
            T key = listOfKeySet.get(i);
            trajectoryGenerators.get(key).initialize();
         }
      }

      @Override
      public void doAction(double timeInState)
      {
//         for (int i = 0; i < listOfKeySet.size(); i++)
//         {
//            T key = listOfKeySet.get(i);
//            MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectoryGenerator = trajectoryGenerators.get(key);
//            double value = multipleWaypointsTrajectoryGenerator.getValue();
//
//            if (multipleWaypointsTrajectoryGenerator.getLastWaypointTime() > stateMachine.getTimeInCurrentState())
//            {
//               multipleWaypointsTrajectoryGenerator.compute(stateMachine.getTimeInCurrentState());
//               value = multipleWaypointsTrajectoryGenerator.getValue();
//            }
//            else
//            {
//               SimpleTrajectoryPoint1D lastPoint = new SimpleTrajectoryPoint1D();
//               multipleWaypointsTrajectoryGenerator.getLastWaypoint(lastPoint);
//               value = lastPoint.getPosition();
//            }
//
//            fingerControlSpace.get(key).setValue(value);
//         }
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return false;
      }

      @Override
      public void onExit()
      {

      }
   }

   @Override
   public void initialize()
   {

   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public void doControl()
   {
      stateMachine.doActionAndTransition();
      
      for (int i = 0; i < listOfKeySet.size(); i++)
      {
         T key = listOfKeySet.get(i);
         MultipleWaypointsTrajectoryGenerator multipleWaypointsTrajectoryGenerator = trajectoryGenerators.get(key);
         double value = multipleWaypointsTrajectoryGenerator.getValue();

         if (multipleWaypointsTrajectoryGenerator.getLastWaypointTime() > stateMachine.getTimeInCurrentState())
         {
            multipleWaypointsTrajectoryGenerator.compute(stateMachine.getTimeInCurrentState());
            value = multipleWaypointsTrajectoryGenerator.getValue();
         }
         else
         {
            SimpleTrajectoryPoint1D lastPoint = new SimpleTrajectoryPoint1D();
            multipleWaypointsTrajectoryGenerator.getLastWaypoint(lastPoint);
            value = lastPoint.getPosition();
         }

         fingerControlSpace.get(key).setValue(value);
      }
   }
   
   public void executeTrajectories()
   {
      requestedState.set(TrajectoryGeneratorMode.JOINTSPACE);
   }

   public void setDesired(T controlSpace, double time, double delayTime, double goal)
   {
      trajectoryGenerators.get(controlSpace).clear();
      trajectoryGenerators.get(controlSpace).appendWaypoint(delayTime, fingerControlSpace.get(controlSpace).getValue(), 0.0);
      trajectoryGenerators.get(controlSpace).appendWaypoint(delayTime + time, goal, 0.0);
      trajectoryGenerators.get(controlSpace).initialize();
   }

   public void setStop(T controlSpace, double current)
   {
      trajectoryGenerators.get(controlSpace).clear();
      trajectoryGenerators.get(controlSpace).appendWaypoint(0.0, fingerControlSpace.get(controlSpace).getValue(), 0.0);
      trajectoryGenerators.get(controlSpace).initialize();
   }

   public double getDesired(T controlSpace)
   {
      return fingerControlSpace.get(controlSpace).getValue();
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }
}
