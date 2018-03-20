package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message commands the controller to move the spine in jointspace to the desired joint angles while going through the specified trajectory points.
 * A third order polynomial function is used to interpolate between trajectory points.
 */
public class SpineTrajectoryMessage extends Packet<SpineTrajectoryMessage>
      implements Settable<SpineTrajectoryMessage>, EpsilonComparable<SpineTrajectoryMessage>
{
   /**
    * The trajectories for each joint in order from the one closest to the pelvis to the one the closest to the chest.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_;

   public SpineTrajectoryMessage()
   {
      jointspace_trajectory_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();
   }

   public SpineTrajectoryMessage(SpineTrajectoryMessage other)
   {
      set(other);
   }

   public void set(SpineTrajectoryMessage other)
   {
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_, jointspace_trajectory_);
   }

   /**
    * The trajectories for each joint in order from the one closest to the pelvis to the one the closest to the chest.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectory()
   {
      return jointspace_trajectory_;
   }

   @Override
   public boolean epsilonEquals(SpineTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.jointspace_trajectory_.epsilonEquals(other.jointspace_trajectory_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof SpineTrajectoryMessage))
         return false;

      SpineTrajectoryMessage otherMyClass = (SpineTrajectoryMessage) other;

      if (!this.jointspace_trajectory_.equals(otherMyClass.jointspace_trajectory_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("SpineTrajectoryMessage {");
      builder.append("jointspace_trajectory=");
      builder.append(this.jointspace_trajectory_);

      builder.append("}");
      return builder.toString();
   }
}