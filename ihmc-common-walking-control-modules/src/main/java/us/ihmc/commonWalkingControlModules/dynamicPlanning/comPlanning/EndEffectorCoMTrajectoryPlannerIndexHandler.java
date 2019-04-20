package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import gnu.trove.map.hash.TIntIntHashMap;

import java.util.List;

public class EndEffectorCoMTrajectoryPlannerIndexHandler
{
   private static final int sequenceSize = 6;
   private static final int vrpWaypointSize = 4;

   private final List< ? extends ContactStateProvider> contactSequence;

   private int size;
   private int numberOfVRPWaypoints;
   private int numberOfEndEffectorVRPWaypoints;
   private final TIntIntHashMap vrpWaypointIndices = new TIntIntHashMap();

   public EndEffectorCoMTrajectoryPlannerIndexHandler(List<? extends ContactStateProvider> contactSequence)
   {
      this.contactSequence = contactSequence;
   }

   public void update()
   {
      vrpWaypointIndices.clear();
      size = 0;
      numberOfVRPWaypoints = 0;
      numberOfEndEffectorVRPWaypoints = 0;
      if (contactSequence.get(0).getContactState() == ContactState.IN_CONTACT)
      {
         vrpWaypointIndices.put(0, numberOfVRPWaypoints);
         numberOfVRPWaypoints += vrpWaypointSize; // start and end
         numberOfEndEffectorVRPWaypoints += contactSequence.get(0).getNumberOfBodiesInContact() * vrpWaypointSize; // start and end
      }
      size += sequenceSize;
      for (int sequenceId = 1; sequenceId < contactSequence.size(); sequenceId++)
      {
         if (contactSequence.get(sequenceId).getContactState() == ContactState.IN_CONTACT)
         {
            vrpWaypointIndices.put(sequenceId, numberOfVRPWaypoints);
            numberOfVRPWaypoints += vrpWaypointSize;
            numberOfEndEffectorVRPWaypoints += contactSequence.get(0).getNumberOfBodiesInContact() * vrpWaypointSize;
         }
         size += sequenceSize;
      }
   }

   public int getTotalSize()
   {
      return size;
   }

   public int getNumberOfVRPWaypoints()
   {
      return numberOfVRPWaypoints;
   }
   

   public int getNumberOfEndEffectorVRPWaypoints()
   {
      return numberOfEndEffectorVRPWaypoints;
   }

   public int getContactSequenceStartIndex(int sequenceNumber)
   {
      return 6 * sequenceNumber;
   }

   public int getVRPWaypointStartPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber);
   }

   public int getVRPWaypointStartVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 1;
   }

   public int getVRPWaypointFinalPositionIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 2;
   }

   public int getVRPWaypointFinalVelocityIndex(int sequenceNumber)
   {
      return vrpWaypointIndices.get(sequenceNumber) + 3;
   }
}