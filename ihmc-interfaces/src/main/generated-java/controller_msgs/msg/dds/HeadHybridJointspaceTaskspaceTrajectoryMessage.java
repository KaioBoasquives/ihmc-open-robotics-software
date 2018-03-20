package controller_msgs.msg.dds;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

/**
 * This message is part of the IHMC whole-body controller API.
 * This message commands the controller to move the head in both taskspace and jointspace
 * to the desired orientation and joint angles while going through the specified trajectory points.
 */
public class HeadHybridJointspaceTaskspaceTrajectoryMessage extends Packet<HeadHybridJointspaceTaskspaceTrajectoryMessage>
      implements Settable<HeadHybridJointspaceTaskspaceTrajectoryMessage>, EpsilonComparable<HeadHybridJointspaceTaskspaceTrajectoryMessage>
{
   /**
    * The taskspace trajectory information.
    */
   public controller_msgs.msg.dds.SE3TrajectoryMessage taskspace_trajectory_message_;
   /**
    * The jointspace trajectory information.
    * The indexing for the joints goes increasingly from the joint the closest to the chest to the joint the closest to the head.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage jointspace_trajectory_message_;

   public HeadHybridJointspaceTaskspaceTrajectoryMessage()
   {
      taskspace_trajectory_message_ = new controller_msgs.msg.dds.SE3TrajectoryMessage();
      jointspace_trajectory_message_ = new controller_msgs.msg.dds.JointspaceTrajectoryMessage();
   }

   public HeadHybridJointspaceTaskspaceTrajectoryMessage(HeadHybridJointspaceTaskspaceTrajectoryMessage other)
   {
      set(other);
   }

   public void set(HeadHybridJointspaceTaskspaceTrajectoryMessage other)
   {
      controller_msgs.msg.dds.SE3TrajectoryMessagePubSubType.staticCopy(other.taskspace_trajectory_message_, taskspace_trajectory_message_);
      controller_msgs.msg.dds.JointspaceTrajectoryMessagePubSubType.staticCopy(other.jointspace_trajectory_message_, jointspace_trajectory_message_);
   }

   /**
    * The taskspace trajectory information.
    */
   public controller_msgs.msg.dds.SE3TrajectoryMessage getTaskspaceTrajectoryMessage()
   {
      return taskspace_trajectory_message_;
   }

   /**
    * The jointspace trajectory information.
    * The indexing for the joints goes increasingly from the joint the closest to the chest to the joint the closest to the head.
    */
   public controller_msgs.msg.dds.JointspaceTrajectoryMessage getJointspaceTrajectoryMessage()
   {
      return jointspace_trajectory_message_;
   }

   @Override
   public boolean epsilonEquals(HeadHybridJointspaceTaskspaceTrajectoryMessage other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!this.taskspace_trajectory_message_.epsilonEquals(other.taskspace_trajectory_message_, epsilon))
         return false;

      if (!this.jointspace_trajectory_message_.epsilonEquals(other.jointspace_trajectory_message_, epsilon))
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
      if (!(other instanceof HeadHybridJointspaceTaskspaceTrajectoryMessage))
         return false;

      HeadHybridJointspaceTaskspaceTrajectoryMessage otherMyClass = (HeadHybridJointspaceTaskspaceTrajectoryMessage) other;

      if (!this.taskspace_trajectory_message_.equals(otherMyClass.taskspace_trajectory_message_))
         return false;

      if (!this.jointspace_trajectory_message_.equals(otherMyClass.jointspace_trajectory_message_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("HeadHybridJointspaceTaskspaceTrajectoryMessage {");
      builder.append("taskspace_trajectory_message=");
      builder.append(this.taskspace_trajectory_message_);

      builder.append(", ");
      builder.append("jointspace_trajectory_message=");
      builder.append(this.jointspace_trajectory_message_);

      builder.append("}");
      return builder.toString();
   }
}