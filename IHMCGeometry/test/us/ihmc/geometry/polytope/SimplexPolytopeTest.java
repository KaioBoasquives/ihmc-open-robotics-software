package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class SimplexPolytopeTest
{

   @DeployableTestMethod(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestPointToOriginOnConvexHull()
   {
      SimplexPolytope simplex = new SimplexPolytope();

      // Single Point
      Point3d pointOne = new Point3d(1.0, 2.0, 3.0);
      simplex.setPoints(pointOne);

      Point3d closestPointToOrigin = new Point3d();
      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertEquals(1, simplex.getNumberOfPoints());

      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);

      // Two Points
      Point3d pointTwo = new Point3d(1.0, 2.0, 4.0);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, 2.5);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 2.5), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertFalse(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, -12.9);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, 4.0);
      simplex.setPoints(pointOne, pointTwo);

      // Three Points. Start at (1, 2, 3) and (1, 2, 4). Add (4, 2, 3)
      Point3d pointThree = new Point3d(4.0, 2.0, 3.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));

      pointTwo = new Point3d(1.0, 2.0, -12.9);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));

      pointOne = new Point3d(-20.0, 2.0, 3.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(3, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertTrue(simplex.containsPoint(pointThree));

      // Four Points. Start at (1, 1, 1), (1, 2, 1), (2, 1, 1), and (1, 1, 2)
      simplex = new SimplexPolytope();

      pointOne = new Point3d(1.0, 1.0, 1.0);
      pointTwo = new Point3d(1.0, 2.0, 1.0);
      pointThree = new Point3d(2.0, 1.0, 1.0);
      Point3d pointFour = new Point3d(1.0, 1.0, 2.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 1.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertFalse(simplex.containsPoint(pointFour));

      pointOne = new Point3d(1.0, 1.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));

      pointOne = new Point3d(1.0, -20.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 0.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(3, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));

      pointOne = new Point3d(-20.0, -20.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(4, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertTrue(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));
   }

}
