package us.ihmc.robotEnvironmentAwareness.filter;

import java.util.ArrayList;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.fusion.MultisenseInformation;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.PointCloudProjectionHelper;

public class StereoPointCloudFilter implements PointCloudFilterInterface
{
   private Point3D[] inputPoints = null;
   private Point3D[] outputPoints = null;
   private Point3D[] noisePoints = null;

   private static final int DEFAULT_DOWN_SAMPLING_SIZE = 1;

   private final int imageWidth;
   private final int imageHeight;
   private final int downSamplingSize;
   private final int downSamplingWidth;
   private final int downSamplingHeight;

   private final ArrayList<Point3D>[] pointsHolder;
   private final boolean[] pixelOccupancyHolder;

   private final Point3D cameraPosition = new Point3D();
   private final Quaternion cameraOrientation = new Quaternion();
   private IntrinsicParameters intrinsicParameters = MultisenseInformation.CART.getIntrinsicParameters();

   public StereoPointCloudFilter(int imageWidth, int imageHeight)
   {
      this(imageWidth, imageHeight, DEFAULT_DOWN_SAMPLING_SIZE);
   }

   public StereoPointCloudFilter(int imageWidth, int imageHeight, int downSamplingSize)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.downSamplingSize = downSamplingSize;
      downSamplingWidth = imageWidth / downSamplingSize;
      downSamplingHeight = imageHeight / downSamplingSize;

      LogTools.info("imageWidth " + imageWidth);
      LogTools.info("imageHeight " + imageHeight);
      LogTools.info("downSamplingWidth " + downSamplingWidth);
      LogTools.info("downSamplingHeight " + downSamplingHeight);

      pointsHolder = new ArrayList[downSamplingWidth * downSamplingHeight];
      pixelOccupancyHolder = new boolean[downSamplingWidth * downSamplingHeight];

      for (int i = 0; i < downSamplingWidth * downSamplingHeight; i++)
      {
         pointsHolder[i] = new ArrayList<Point3D>();
         pixelOccupancyHolder[i] = false;
      }
   }

   public void setCameraPosition(Point3DReadOnly sensorPosition)
   {
      this.cameraPosition.set(sensorPosition);
   }

   public void setCameraOrientation(QuaternionReadOnly cameraOrientation)
   {
      this.cameraOrientation.set(cameraOrientation);
   }

   public void setIntrinsicParameters(IntrinsicParameters intrinsicParameters)
   {
      this.intrinsicParameters.set(intrinsicParameters);
   }

   @Override
   public void initialize()
   {
      for (int i = 0; i < downSamplingWidth * downSamplingHeight; i++)
      {
         pointsHolder[i].clear();
         pixelOccupancyHolder[i] = false;
      }
   }

   @Override
   public void updateInput(Point3D[] points)
   {
      inputPoints = points;
   }

   @Override
   public void calculate()
   {
      // project.
      for (int i = 0; i < inputPoints.length; i++)
      {
         Point3D point = inputPoints[i];
         if (point == null)
            break;
         int[] pixel = PointCloudProjectionHelper.projectMultisensePointCloudOnImage(point, intrinsicParameters, cameraPosition, cameraOrientation);

         if (pixel[0] < 0 || pixel[0] >= imageWidth || pixel[1] < 0 || pixel[1] >= imageHeight)
            continue;

         int[] downSamplingPixel = new int[2];
         downSamplingPixel[0] = pixel[0] / downSamplingSize;
         downSamplingPixel[1] = pixel[1] / downSamplingSize;
         int arrayIndex = getArrayIndex(downSamplingPixel[0], downSamplingPixel[1]);
         pointsHolder[arrayIndex].add(new Point3D(point));
         pixelOccupancyHolder[i] = true;
      }

      // filter in pixel level.
      int numberOfOccupiedPixel = 0;
      for (int i = 0; i < downSamplingWidth * downSamplingHeight; i++)
      {
         if (pixelOccupancyHolder[i])
         {
            numberOfOccupiedPixel++;
         }
      }
      outputPoints = new Point3D[numberOfOccupiedPixel];
      for (int i = 0; i < numberOfOccupiedPixel; i++)
         outputPoints[i] = new Point3D();

      for (int i = 0; i < downSamplingWidth * downSamplingHeight; i++)
      {
         if (pixelOccupancyHolder[i])
         {
            int numberOfPoints = pointsHolder[i].size();
            double minimumDistance = Double.POSITIVE_INFINITY;
            Point3D representativePoint = new Point3D();
            for (int j = 0; j < numberOfPoints; j++)
            {
               double distance = cameraPosition.distance(pointsHolder[i].get(j));
               if (distance < minimumDistance)
               {
                  minimumDistance = distance;
                  representativePoint.set(pointsHolder[i].get(j));
               }
            }
            outputPoints[i].set(representativePoint);
         }
      }
   }

   @Override
   public Point3D[] getOutput()
   {
      return outputPoints;
   }

   public Point3D[] getNoise()
   {
      return noisePoints;
   }

   private int getArrayIndex(int u, int v)
   {
      return u + v * downSamplingWidth;
   }
}
