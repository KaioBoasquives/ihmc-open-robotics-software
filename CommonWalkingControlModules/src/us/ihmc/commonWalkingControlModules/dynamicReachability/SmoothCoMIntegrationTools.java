package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoFrameTrajectory3D;
import us.ihmc.commonWalkingControlModules.angularMomentumTrajectoryGenerator.YoTrajectory;
import us.ihmc.robotics.geometry.Direction;
import us.ihmc.robotics.geometry.FrameTuple;

public class SmoothCoMIntegrationTools extends CoMIntegrationTools
{
   private static final int defaultSize = 1000;
   
   private static final DenseMatrix64F tPowersDerivativeVector = new DenseMatrix64F(defaultSize, 1);
   private static final DenseMatrix64F tPowersDerivativeVectorTranspose = new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedBetaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   private static final DenseMatrix64F generalizedDeltaCoMPrimeRow = new DenseMatrix64F(1, defaultSize);
   
   private static final DenseMatrix64F polynomialCoefficientCombinedVector= new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F polynomialCoefficientVector= new DenseMatrix64F(defaultSize, 1);
   
   private static final DenseMatrix64F generalizedAlphaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedBetaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedGammaCoMPrimeMatrix = new DenseMatrix64F(1, 1);
   private static final DenseMatrix64F generalizedDeltaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix = new DenseMatrix64F(defaultSize, defaultSize);
   
   private static final DenseMatrix64F M1 = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F M2 = new DenseMatrix64F(defaultSize, defaultSize);
   private static final DenseMatrix64F M3 = new DenseMatrix64F(defaultSize, defaultSize);
   
   
   public static void calculateCoMQuantityFromCorrespondingCMPPolynomial3D(double omega0, double time, int comDerivativeOrder, 
                                                                           YoFrameTrajectory3D cmpPolynomial3D, 
                                                                           FrameTuple<?, ?> icpPositionDesiredFinal, 
                                                                           FrameTuple<?, ?> comPositionDesiredFinal, 
                                                                           FrameTuple<?, ?> icpQuantityDesired)
   {        
      int numberOfCoefficients = cmpPolynomial3D.getNumberOfCoefficients();
      if(numberOfCoefficients == -1)
      {
         icpQuantityDesired.setToNaN();
         return;
      }
      
      initializeMatrices3D(numberOfCoefficients);
      setPolynomialCoefficientVector3D(polynomialCoefficientCombinedVector, cmpPolynomial3D);

      calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(omega0, time, generalizedAlphaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(omega0, time, generalizedBetaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(omega0, time, generalizedGammaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(omega0, time, generalizedDeltaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial3D);
      CommonOps.subtract(generalizedAlphaCoMPrimeMatrix, generalizedBetaCoMPrimeMatrix, generalizedAlphaBetaCoMPrimeMatrix);

      calculateCoMQuantity3D(generalizedAlphaBetaCoMPrimeMatrix, generalizedGammaCoMPrimeMatrix, generalizedDeltaCoMPrimeMatrix, polynomialCoefficientCombinedVector, 
                             icpPositionDesiredFinal, comPositionDesiredFinal, icpQuantityDesired);
      
//      PrintTools.debug("A: " + generalizedAlphaCoMPrimeMatrix.toString());
//      PrintTools.debug("B: " + generalizedBetaCoMPrimeMatrix.toString());
//      PrintTools.debug("C: " + generalizedGammaCoMPrimeMatrix.toString());
//      PrintTools.debug("AB: " + generalizedAlphaBetaCoMPrimeMatrix.toString());
//      PrintTools.debug("P: " + polynomialCoefficientCombinedVector.toString());
   }
   
   public static double calculateCoMQuantityFromCorrespondingCMPPolynomial1D(double omega0, double time, int comDerivativeOrder, YoTrajectory cmpPolynomial, double icpPositionDesiredFinal, double comPositionDesiredFinal)
   {      
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
   
      initializeMatrices1D(numberOfCoefficients);
      setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial);
   
      calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(omega0, time, generalizedAlphaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(omega0, time, generalizedBetaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedGammaCoMPrimeOnCMPSegment1D(omega0, time, generalizedGammaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(omega0, time, generalizedDeltaCoMPrimeMatrix, comDerivativeOrder, cmpPolynomial);
      CommonOps.subtract(generalizedAlphaCoMPrimeMatrix, generalizedBetaCoMPrimeMatrix, generalizedAlphaBetaCoMPrimeMatrix);
   
      return calculateCoMQuantity1D(generalizedAlphaBetaCoMPrimeMatrix, generalizedGammaCoMPrimeMatrix, generalizedDeltaCoMPrimeMatrix, polynomialCoefficientVector, icpPositionDesiredFinal, comPositionDesiredFinal);
   }
   
   public static void calculateCoMQuantity3D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, DenseMatrix64F generalizedGammaCoMPrimeMatrix, DenseMatrix64F generalizedDeltaCoMPrimeMatrix,
                                             DenseMatrix64F polynomialCoefficientCombinedVector, FrameTuple<?, ?> icpPositionDesiredFinal, FrameTuple<?, ?> comPositionDesiredFinal,
                                             FrameTuple<?, ?> icpQuantityDesired)
   {
      M1.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientCombinedVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientCombinedVector, M1);

      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredFinal.getX());
      M2.set(1, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredFinal.getY());
      M2.set(2, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredFinal.getZ());
      
      double scalar = 0.5 * (generalizedGammaCoMPrimeMatrix.get(0, 0) - 1.0 / generalizedGammaCoMPrimeMatrix.get(0, 0));
      M3.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientCombinedVector.getNumCols());
      M3.zero();

      CommonOps.mult(generalizedDeltaCoMPrimeMatrix, polynomialCoefficientCombinedVector, M3);
      M3.set(0, 0, M3.get(0, 0) - icpPositionDesiredFinal.getX());
      M3.set(1, 0, M3.get(1, 0) - icpPositionDesiredFinal.getY());
      M3.set(2, 0, M3.get(2, 0) - icpPositionDesiredFinal.getZ());
      CommonOps.scale(scalar, M3);
            
      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, M3);
      
      icpQuantityDesired.set(M1.get(0, 0), M1.get(1, 0), M1.get(2, 0));
   }
   
   public static double calculateCoMQuantity1D(DenseMatrix64F generalizedAlphaBetaCoMPrimeMatrix, DenseMatrix64F generalizedGammaCoMPrimeMatrix, DenseMatrix64F generalizedDeltaCoMPrimeMatrix,
                                               DenseMatrix64F polynomialCoefficientVector, double icpPositionDesiredFinal, double comPositionDesiredFinal)
   {
      M1.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M1.zero();

      CommonOps.mult(generalizedAlphaBetaCoMPrimeMatrix, polynomialCoefficientVector, M1);

      M2.reshape(M1.getNumRows(),  M1.getNumCols());
      M2.set(0, 0, generalizedGammaCoMPrimeMatrix.get(0, 0) * comPositionDesiredFinal);
      
      double scalar = 0.5 * (generalizedGammaCoMPrimeMatrix.get(0, 0) - 1.0 / generalizedGammaCoMPrimeMatrix.get(0, 0));
      M3.reshape(generalizedAlphaBetaCoMPrimeMatrix.getNumRows(), polynomialCoefficientVector.getNumCols());
      M3.zero();

      CommonOps.mult(generalizedDeltaCoMPrimeMatrix, polynomialCoefficientVector, M3);
      M3.set(0, 0, M3.get(0, 0) - icpPositionDesiredFinal);
      CommonOps.scale(scalar, M3);
            
      CommonOps.addEquals(M1, M2);
      CommonOps.addEquals(M1, M3);
      
      return M1.get(0, 0);
   }
   
   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrime, 
                                                                      int alphaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(omega0, time, generalizedAlphaCoMPrimeRow, alphaCoMDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedAlphaCoMPrimeRow, generalizedAlphaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedAlphaCoMPrimeRow.numCols);
      }
   }
   
   public static void calculateGeneralizedAlphaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedAlphaCoMPrimeRow,
                                                                      int alphaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedAlphaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedAlphaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = 0; j < numberOfCoefficients - i; i++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + j + alphaCoMDerivativeOrder, time));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar = Math.pow(-1.0, j) * Math.pow(omega0, -(i + j));
            CommonOps.addEquals(generalizedAlphaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrime, 
                                                                     int betaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(omega0, time, generalizedBetaCoMPrimeRow, betaCoMDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedBetaCoMPrimeRow, generalizedBetaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedBetaCoMPrimeRow.numCols);
      }
   }
   
   public static void calculateGeneralizedBetaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedBetaCoMPrimeRow, 
                                                                     int betaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedBetaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedBetaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         for(int j = 0; j < numberOfCoefficients - i; i++)
         {
            tPowersDerivativeVector.zero();
            tPowersDerivativeVectorTranspose.zero();
            
            tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + j, timeSegmentTotal));
            CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
            
            double scalar =  Math.pow(-1.0, j + betaCoMDerivativeOrder) * Math.pow(omega0,  -(i + j) + betaCoMDerivativeOrder) * Math.exp(omega0*(timeSegmentTotal-time));
            CommonOps.addEquals(generalizedBetaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);            
         }
      }
   }
   
   public static void calculateGeneralizedGammaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedGammaCoMPrime, 
                                                                      int gammaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {      
      double timeSegmentTotal = cmpPolynomial3D.getFinalTime();
      double ddGamaPrimeValue = Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder)*Math.exp(omega0 * (timeSegmentTotal - time));
      generalizedGammaCoMPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public static void calculateGeneralizedGammaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedGammaCoMPrime, 
                                                                      int gammaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {      
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      double ddGamaPrimeValue = Math.pow(-1.0, gammaCoMDerivativeOrder) * Math.pow(omega0, gammaCoMDerivativeOrder) * Math.exp(omega0 * (timeSegmentTotal - time));
      generalizedGammaCoMPrime.set(0, 0, ddGamaPrimeValue);
   }
   
   public static void calculateGeneralizedDeltaCoMPrimeOnCMPSegment3D(double omega0, double time, DenseMatrix64F generalizedDeltaCoMPrime, 
                                                                      int deltaCoMDerivativeOrder, YoFrameTrajectory3D cmpPolynomial3D)
   {                  
      for(Direction dir : Direction.values())
      {
         YoTrajectory cmpPolynomial = cmpPolynomial3D.getYoTrajectory(dir);
         
         calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(omega0, time, generalizedDeltaCoMPrimeRow, deltaCoMDerivativeOrder, cmpPolynomial);
         
         CommonOps.insert(generalizedDeltaCoMPrimeRow, generalizedDeltaCoMPrime, dir.ordinal(), dir.ordinal() * generalizedDeltaCoMPrimeRow.numCols);
      }
   }
   
   public static void calculateGeneralizedDeltaCoMPrimeOnCMPSegment1D(double omega0, double time, DenseMatrix64F generalizedDeltaCoMPrimeRow, 
                                                                      int deltaCoMDerivativeOrder, YoTrajectory cmpPolynomial)
   {                  
      int numberOfCoefficients = cmpPolynomial.getNumberOfCoefficients();
      double timeSegmentTotal = cmpPolynomial.getFinalTime();
      
      tPowersDerivativeVector.reshape(numberOfCoefficients, 1);
      tPowersDerivativeVectorTranspose.reshape(1, numberOfCoefficients);
      
      generalizedDeltaCoMPrimeRow.reshape(1, numberOfCoefficients);
      generalizedDeltaCoMPrimeRow.zero();
      
      for(int i = 0; i < numberOfCoefficients; i++)
      {
         tPowersDerivativeVector.zero();
         tPowersDerivativeVectorTranspose.zero();
         
         tPowersDerivativeVector.set(cmpPolynomial.getXPowersDerivativeVector(i + deltaCoMDerivativeOrder, timeSegmentTotal));
         CommonOps.transpose(tPowersDerivativeVector, tPowersDerivativeVectorTranspose);
                  
         double scalar = Math.pow(omega0, -i);
         CommonOps.addEquals(generalizedDeltaCoMPrimeRow, scalar, tPowersDerivativeVectorTranspose);
      }
   }
   
   public static void initializeMatrices3D(int numberOfCoefficients)
   {
      initializeMatrices(3, numberOfCoefficients);
   }
   
   public static void initializeMatrices1D(int numberOfCoefficients)
   {
      initializeMatrices(1, numberOfCoefficients);
   }
   
   public static void initializeMatrices(int dimension, int numberOfCoefficients)
   {
      polynomialCoefficientCombinedVector.reshape(dimension * numberOfCoefficients, 1);
      polynomialCoefficientCombinedVector.zero();
      
      generalizedAlphaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaCoMPrimeMatrix.zero();
      
      generalizedBetaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedBetaCoMPrimeMatrix.zero();
      
      generalizedAlphaBetaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedAlphaBetaCoMPrimeMatrix.zero();
      
      generalizedGammaCoMPrimeMatrix.reshape(1, 1);
      generalizedGammaCoMPrimeMatrix.zero();
      
      generalizedDeltaCoMPrimeMatrix.reshape(dimension, dimension * numberOfCoefficients);
      generalizedDeltaCoMPrimeMatrix.zero();
   }
   
   public static void setPolynomialCoefficientVector3D(DenseMatrix64F polynomialCoefficientCombinedVector, YoFrameTrajectory3D cmpPolynomial3D)
   {
      for(Direction dir : Direction.values())
      {
         setPolynomialCoefficientVector1D(polynomialCoefficientVector, cmpPolynomial3D.getYoTrajectory(dir));
         
         CommonOps.insert(polynomialCoefficientVector, polynomialCoefficientCombinedVector, dir.ordinal() * polynomialCoefficientVector.numRows, 0);
      }
   }
   
   public static void setPolynomialCoefficientVector1D(DenseMatrix64F polynomialCoefficientVector, YoTrajectory cmpPolynomial)
   {
      double[] polynomialCoefficients = cmpPolynomial.getCoefficients();
      
      polynomialCoefficientVector.reshape(cmpPolynomial.getNumberOfCoefficients(), 1);
      polynomialCoefficientVector.zero();
      
      polynomialCoefficientVector.setData(polynomialCoefficients);
   }
}
