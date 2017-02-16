package us.ihmc.tools.thread;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;

public class ThreadToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 3.1, categoriesOverride = IntegrationCategory.FLAKY)
   @Test(timeout = 30000)
   public void testTimeLimitScheduler()
   {
      final int ITERATIONS = 100;
      final double EPSILON = 5;

      TimeUnit timeUnit = TimeUnit.MILLISECONDS;
      long initialDelay = 0;
      long delay = 3;
      long timeLimit = 30;

      final Runnable runnable = new Runnable()

      {
         @Override
         public void run()
         {
            //Do some calculation
            Math.sqrt(Math.PI);
         }
      };

      for (int i = 0; i < ITERATIONS; i++)
      {
         long startTime = System.currentTimeMillis();
         ScheduledFuture<?> future = ThreadTools.scheduleWithFixeDelayAndTimeLimit(getClass().getSimpleName(), runnable, initialDelay, delay, timeUnit, timeLimit);
         while(!future.isDone())
         {
            // do nothing
         }
         long endTime = System.currentTimeMillis();
         assertEquals(timeLimit, endTime - startTime, EPSILON);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testIterationLimitScheduler()
   {
      TimeUnit timeUnit = TimeUnit.MILLISECONDS;
      long initialDelay = 0;
      long delay = 10;
      final int iterations = 10;

      final AtomicInteger counter = new AtomicInteger();

      final Runnable runnable = new Runnable()
      {
         @Override
         public void run()
         {
            counter.incrementAndGet();
         }
      };

      ScheduledFuture<?> future = ThreadTools.scheduleWithFixedDelayAndIterationLimit(getClass().getSimpleName(), runnable, initialDelay, delay, timeUnit, iterations);
      
      while(!future.isDone())
      {
         // do nothing
      }
      
      assertEquals(iterations, counter.get());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.FLAKY)
   @Test(timeout = 30000)
   public void testExecuteWithTimeout()
   {
      final StateHolder holder = new StateHolder();
      for (int i = 0; i < 10; i++)
      {
         System.out.println("i = " + i);
         holder.state = State.DIDNT_RUN;
         ThreadTools.executeWithTimeout("timeoutTest1", new Runnable()
         {
            @Override
            public void run()
            {
               holder.state = State.TIMED_OUT;

               ThreadTools.sleep(10);

               holder.state = State.RAN_WITHOUT_TIMING_OUT;
            }
         }, 5, TimeUnit.MILLISECONDS);
         assertFalse("Didn't run. Should timeout.", holder.state.equals(State.DIDNT_RUN));
         assertTrue("Did not timeout.", holder.state.equals(State.TIMED_OUT));

         holder.state = State.DIDNT_RUN;
         ThreadTools.executeWithTimeout("timeoutTest2", new Runnable()
         {
            @Override
            public void run()
            {
               holder.state = State.TIMED_OUT;

               ThreadTools.sleep(5);

               holder.state = State.RAN_WITHOUT_TIMING_OUT;
            }
         }, 10, TimeUnit.MILLISECONDS);
         assertFalse("Didn't run. Shouldn't timeout.", holder.state.equals(State.DIDNT_RUN));
         assertTrue("Timed out early.", holder.state.equals(State.RAN_WITHOUT_TIMING_OUT));
      }
   }

   private enum State
   {
      DIDNT_RUN, TIMED_OUT, RAN_WITHOUT_TIMING_OUT;
   }

   private class StateHolder
   {
      public State state = State.DIDNT_RUN;
   }
}
