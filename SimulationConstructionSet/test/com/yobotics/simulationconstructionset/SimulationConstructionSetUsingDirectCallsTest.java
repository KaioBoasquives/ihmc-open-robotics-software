package com.yobotics.simulationconstructionset;

import static org.junit.Assert.*;

import java.awt.AWTException;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;
import java.util.ArrayList;

import org.junit.After;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.utilities.ThreadTools;

import com.yobotics.simulationconstructionset.examples.FallingBrickRobot;
import com.yobotics.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import com.yobotics.simulationconstructionset.gui.StandardSimulationGUI;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObject;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicVector;

public class SimulationConstructionSetUsingDirectCallsTest
{
   // all tests assume that:
   // - the simpleRobot has been used to create the SimulationConstructionSet instance
   // - the registry "simpleRegistry" is empty
   
   private static double epsilon = 1e-10;
   
   private int recordFrequency = 10;
   private int index = 15;
   private int inputPoint = 10;
   private int outputPoint = 30;
   private int middleIndex = 22;
   private int nonMiddleIndex = 35;
   private double simulateDT = 0.1; 
   private double simulateTime = 1.0;
   private double recordDT = 0.5;
   private double recordFreq = computeRecordFreq(recordDT, simulateDT);
   private double realTimeRate = 0.75;
   private double frameRate = 0.5;
   private double recomputedSecondsPerFrameRate = recomputeTiming(simulateDT, recordFreq, realTimeRate, frameRate);
   
   private Dimension dimension = new Dimension(250, 350);
   private FallingBrickRobot simpleRobot = new FallingBrickRobot();
   private Point location = new Point(25, 50);
   private String rootRegistryName = "root";
   private String simpleRegistryName = "simpleRegistry";
   private String searchString = "d";
   private String searchStringStart = "q";
   private String[] regularExpressions = new String[] {"gc.*.fs"};
   private String simpleRobotFirstVariableName = getFirstVariableNameFromRobotRegistry(simpleRobot);
   private String simpleRobotRegistryNameSpace = getRegistryNameSpaceFromRobot(simpleRobot);
   private String cameraTrackingVarNameX = "simpleCameraTrackingVarNameX";
   private String cameraTrackingVarNameY = "simpleCameraTrackingVarNameY";
   private String cameraTrackingVarNameZ = "simpleCameraTrackingVarNameZ";
   private String cameraDollyVarNameX = "simpleCameraDollyVarNameX";
   private String cameraDollyVarNameY = "simpleCameraDollyVarNameY";
   private String cameraDollyVarNameZ = "simpleCameraDollyVarNameZ";
   private NameSpace simpleRegistryNameSpace = new NameSpace(rootRegistryName + "." + simpleRegistryName);
   private YoVariableRegistry simpleRegistry = new YoVariableRegistry(simpleRegistryName);
   private YoVariableRegistry dummyRegistry = new YoVariableRegistry("dummyRegistry");
   private Link staticLink = new Link("simpleLink");
   private Graphics3DObject staticLinkGraphics = staticLink.getLinkGraphics();
   private Graphics3DNodeType graphics3DNodeType = Graphics3DNodeType.GROUND;
   private ExternalForcePoint simpleExternalForcePoint = new ExternalForcePoint("simpleExternalForcePoint", dummyRegistry);
   private DynamicGraphicObject dynamicGraphicObject = new DynamicGraphicVector("simpleDynamicGraphicObject", simpleExternalForcePoint);
   
   SimulationConstructionSet scs = createAndStartSCSWithRobot(simpleRobot);

   @Test
   public void testSimulationConstructionSetUsingDirectCalls() throws AWTException
   {
      double startTime = scs.getTime();
      scs.simulate(simulateTime);
      while(scs.isSimulating())
      {
         ThreadTools.sleep(100L);
      }
      double endTime = scs.getTime();
      assertEquals(simulateTime, endTime-startTime, 1e-7);
      
      Robot[] robotFromSCS = scs.getRobots();
      assertEquals(simpleRobot, robotFromSCS[0]);
      
      scs.setIndex(index);
      int indexFromSCS = scs.getIndex();
      assertEquals(index, indexFromSCS, epsilon);
      
      setInputAndOutputPointsInSCS(scs, inputPoint, outputPoint);
      boolean isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(middleIndex);
      assertEquals(true, isMiddleIndexFromSCS);
      isMiddleIndexFromSCS = scs.isIndexBetweenInAndOutPoint(nonMiddleIndex);
      assertEquals(false, isMiddleIndexFromSCS);

      setInputPointInSCS(scs, inputPoint);
      scs.setIndex(outputPoint);
      StandardSimulationGUI GUIFromSCS = scs.getStandardSimulationGUI();
      GUIFromSCS.gotoInPointNow();
      int inputPointFromSCS = scs.getIndex();
      assertEquals(inputPoint, inputPointFromSCS);
      
      scs.setFrameSize(dimension);
      Dimension dimensionFromSCS = scs.getJFrame().getBounds().getSize();
      assertEquals(dimension.height, dimensionFromSCS.height, epsilon);
      assertEquals(dimension.width, dimensionFromSCS.width, epsilon);

      scs.setFrameLocation(location.x, location.y);
      Point locationFromSCS = scs.getJFrame().getLocation();
      assertEquals(location.x, locationFromSCS.x, epsilon);
      assertEquals(location.y, locationFromSCS.y, epsilon);

      scs.setFrameMaximized();
      int frameStateFromSCS = scs.getJFrame().getExtendedState();
      assertEquals(Frame.MAXIMIZED_BOTH, frameStateFromSCS);

      scs.setFrameAlwaysOnTop(true);
      boolean alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(true, alwaysOnTopFromSCS);
      
      scs.setFrameAlwaysOnTop(false);
      alwaysOnTopFromSCS = scs.getJFrame().isAlwaysOnTop();
      assertEquals(false, alwaysOnTopFromSCS);
      
      YoVariableRegistry rootRegistryFromSCS = scs.getRootRegistry();
      String  rootRegistryNameFromSCS = rootRegistryFromSCS.getName();
      assertEquals(rootRegistryName, rootRegistryNameFromSCS);
      
      scs.addYoVariableRegistry(simpleRegistry);
      YoVariableRegistry simpleRegistryFromSCS = scs.getRootRegistry().getRegistry(simpleRegistryNameSpace);
      assertEquals(simpleRegistry, simpleRegistryFromSCS);

      
//      ThreadTools.sleepForever();
//    Robot simpleRobot2 = new Robot("simpleRobot2");
//    Robot[] robots = {simpleRobot, simpleRobot2};
//    
//    SimulationConstructionSet scs2 = new SimulationConstructionSet(robots);
//    generateSimulationFromDataFile
//     JButton button = new JButton("test");
   }
   
   @Test
   public void testCameraMethods()
   {
      scs.setCameraTracking(true, false, false, false);
      boolean isCameraTracking = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ = scs.getGUI().getCamera().isTrackingZ();
      assertTrue(isCameraTracking);
      assertFalse(isCameraTrackingX);
      assertFalse(isCameraTrackingY);
      assertFalse(isCameraTrackingZ);
      
      scs.setCameraTracking(false, true, false, false);
      boolean isCameraTracking2 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX2 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY2 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ2 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking2);
      assertTrue(isCameraTrackingX2);
      assertFalse(isCameraTrackingY2);
      assertFalse(isCameraTrackingZ2);
      
      scs.setCameraTracking(false, false, true, false);
      boolean isCameraTracking3 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX3 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY3 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ3 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking3);
      assertFalse(isCameraTrackingX3);
      assertTrue(isCameraTrackingY3);
      assertFalse(isCameraTrackingZ3);
      
      scs.setCameraTracking(false, false, false, true);
      boolean isCameraTracking4 = scs.getGUI().getCamera().isTracking();
      boolean isCameraTrackingX4 = scs.getGUI().getCamera().isTrackingX();
      boolean isCameraTrackingY4 = scs.getGUI().getCamera().isTrackingY();
      boolean isCameraTrackingZ4 = scs.getGUI().getCamera().isTrackingZ();
      assertFalse(isCameraTracking4);
      assertFalse(isCameraTrackingX4);
      assertFalse(isCameraTrackingY4);
      assertTrue(isCameraTrackingZ4);
      
      scs.setCameraDolly(true, false, false, false);
      boolean isCameraDolly = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ = scs.getGUI().getCamera().isDollyZ();
      assertTrue(isCameraDolly);
      assertFalse(isCameraDollyX);
      assertFalse(isCameraDollyY);
      assertFalse(isCameraDollyZ);
      
      scs.setCameraDolly(false, true, false, false);
      boolean isCameraDolly2 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX2 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY2 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ2 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly2);
      assertTrue(isCameraDollyX2);
      assertFalse(isCameraDollyY2);
      assertFalse(isCameraDollyZ2);
      
      scs.setCameraDolly(false, false, true, false);
      boolean isCameraDolly3 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX3 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY3 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ3 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly3);
      assertFalse(isCameraDollyX3);
      assertTrue(isCameraDollyY3);
      assertFalse(isCameraDollyZ3);
      
      scs.setCameraDolly(false, false, false, true);
      boolean isCameraDolly4 = scs.getGUI().getCamera().isDolly();
      boolean isCameraDollyX4 = scs.getGUI().getCamera().isDollyX();
      boolean isCameraDollyY4 = scs.getGUI().getCamera().isDollyY();
      boolean isCameraDollyZ4 = scs.getGUI().getCamera().isDollyZ();
      assertFalse(isCameraDolly4);
      assertFalse(isCameraDollyX4);
      assertFalse(isCameraDollyY4);
      assertTrue(isCameraDollyZ4);
      
      setCameraTrackingDoubleYoVariableInSCSRegistry(cameraTrackingVarNameX, cameraTrackingVarNameY, cameraTrackingVarNameZ, scs);
      scs.setCameraTrackingVars(cameraTrackingVarNameX, cameraTrackingVarNameY, cameraTrackingVarNameZ);
      String cameraTrackingVarNameXSCS = scs.getRootRegistry().getVariable(cameraTrackingVarNameX).getName();
      String cameraTrackingVarNameYSCS = scs.getRootRegistry().getVariable(cameraTrackingVarNameY).getName();
      String cameraTrackingVarNameZSCS = scs.getRootRegistry().getVariable(cameraTrackingVarNameZ).getName();
      assertEquals(cameraTrackingVarNameX, cameraTrackingVarNameXSCS);
      assertEquals(cameraTrackingVarNameY, cameraTrackingVarNameYSCS);
      assertEquals(cameraTrackingVarNameZ, cameraTrackingVarNameZSCS);
      
      setCameraDollyDoubleYoVariableInSCSRegistry(cameraDollyVarNameX, cameraDollyVarNameY, cameraDollyVarNameZ, scs);
      scs.setCameraDollyVars(cameraDollyVarNameX, cameraDollyVarNameY, cameraDollyVarNameZ);
      String cameraDollyVarNameXSCS = scs.getRootRegistry().getVariable(cameraDollyVarNameX).getName();
      String cameraDollyVarNameYSCS = scs.getRootRegistry().getVariable(cameraDollyVarNameY).getName();
      String cameraDollyVarNameZSCS = scs.getRootRegistry().getVariable(cameraDollyVarNameZ).getName();
      assertEquals(cameraDollyVarNameX, cameraDollyVarNameXSCS);
      assertEquals(cameraDollyVarNameY, cameraDollyVarNameYSCS);
      assertEquals(cameraDollyVarNameZ, cameraDollyVarNameZSCS);
   }
   
   @Test
   public void test3DGraphicsMethods()
   {
      Graphics3DNode graphics3DNodeFromSCS = scs.addStaticLinkGraphics(staticLinkGraphics);
      assertNotNull(graphics3DNodeFromSCS);
      
      Graphics3DNode graphics3DNodeFromSCS2 = scs.addStaticLinkGraphics(staticLinkGraphics, graphics3DNodeType);
      assertNotNull(graphics3DNodeFromSCS2);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS = scs.addDynamicGraphicObject(dynamicGraphicObject);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS2 = scs.addDynamicGraphicObject(dynamicGraphicObject, true);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS2);
      
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObjectFromSCS3 = scs.addDynamicGraphicObject(dynamicGraphicObject, false);
      assertNotNull(graphicsDynamicGraphicsObjectFromSCS3);
   }
   
   @Test
   public void testGetVariableMethods() throws AWTException
   {
      ArrayList<YoVariable> allVariablesFromRobot = simpleRobot.getAllVariables();
      ArrayList<YoVariable> allVariablesFromSCS = scs.getAllVariables();
      assertEquals(allVariablesFromRobot, allVariablesFromSCS);
      
      int allVariablesArrayFromRobot = simpleRobot.getAllVariablesArray().length;
      int allVariablesArrayFromSCS = scs.getAllVariablesArray().length;
      assertEquals(allVariablesArrayFromRobot, allVariablesArrayFromSCS);
      
      YoVariable yoVariableFromSCS = scs.getVariable(simpleRobotFirstVariableName);
      String variableNameFromSCS = yoVariableFromSCS.getName();
      assertEquals(simpleRobotFirstVariableName, variableNameFromSCS);
      
      YoVariable yoVariableFromRobot = simpleRobot.getVariable(simpleRobotFirstVariableName);
      YoVariable yoVariableFromSCS2 = scs.getVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot, yoVariableFromSCS2);
      
      ArrayList<YoVariable> yoVariableArrayFromRobot =  simpleRobot.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      ArrayList<YoVariable> yoVariableArrayFromSCS =  scs.getVariables(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(yoVariableArrayFromRobot, yoVariableArrayFromSCS);
      
      ArrayList<YoVariable> yoVariableFromRobot2 = simpleRobot.getVariables(simpleRobotFirstVariableName);
      ArrayList<YoVariable> yoVariableFromSCS3 = scs.getVariables(simpleRobotFirstVariableName);
      assertEquals(yoVariableFromRobot2, yoVariableFromSCS3); 
      
      ArrayList<YoVariable> yoVariableFromRobot3 = simpleRobot.getVariables(simpleRobotRegistryNameSpace);
      ArrayList<YoVariable> yoVariableFromSCS4 = scs.getVariables(simpleRobotRegistryNameSpace);
      assertEquals(yoVariableFromRobot3, yoVariableFromSCS4); 
      
      boolean hasUniqueVariableRobot = simpleRobot.hasUniqueVariable(simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS = scs.hasUniqueVariable(simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot, hasUniqueVariableSCS);
      
      boolean hasUniqueVariableRobot2 = simpleRobot.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      boolean hasUniqueVariableSCS2 = scs.hasUniqueVariable(simpleRobotRegistryNameSpace, simpleRobotFirstVariableName);
      assertEquals(hasUniqueVariableRobot2, hasUniqueVariableSCS2);
      
      ArrayList<YoVariable> arrayOfVariablesContainingRobot = getSimpleRobotVariablesThatContain(searchString, false, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesContainingSCS = scs.getVariablesThatContain(searchString);
      assertEquals(arrayOfVariablesContainingRobot, arrayOfVariablesContainingSCS);
      
      ArrayList<YoVariable> arrayOfVariablesContainingRobot2 = getSimpleRobotVariablesThatContain(searchString, true, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesContainingSCS2 = scs.getVariablesThatContain(searchString, true);
      assertEquals(arrayOfVariablesContainingRobot2, arrayOfVariablesContainingSCS2);
      
      ArrayList<YoVariable> arrayOfVariablesStartingRobot = getSimpleRobotVariablesThatStartWith(searchStringStart, simpleRobot);
      ArrayList<YoVariable> arrayOfVariablesStartingSCS = scs.getVariablesThatStartWith(searchStringStart);
      assertEquals(arrayOfVariablesStartingRobot, arrayOfVariablesStartingSCS);
      
      String[] varNames =  getVariableNamesGivenArrayListOfYoVariables(arrayOfVariablesContainingRobot);
      ArrayList<YoVariable>  arrayOfVariablesRegExprRobot = getSimpleRobotRegExpVariables(varNames, regularExpressions, simpleRobot);
      ArrayList<YoVariable>  arrayOfVariablesRegExprSCS = scs.getVars(varNames, regularExpressions);
      assertEquals(arrayOfVariablesRegExprRobot, arrayOfVariablesRegExprSCS);
   }
   
   @Test
   public void testTimingMethods() throws AWTException
   {
      scs.setDT(simulateDT, recordFrequency);
      double simulateDTFromSCS = scs.getDT();
      assertEquals(simulateDT, simulateDTFromSCS, epsilon); 
      
      scs.setRecordDT(recordDT);
      double recordFreqFromSCS = scs.getRecordFreq();
      assertEquals(recordFreq, recordFreqFromSCS, epsilon); 
      
      scs.setPlaybackRealTimeRate(realTimeRate);
      double realTimeRateFromSCS = scs.getPlaybackRealTimeRate();
      assertEquals(realTimeRate, realTimeRateFromSCS, epsilon);
      
      scs.setPlaybackDesiredFrameRate(frameRate);
      double frameRateFromSCS = scs.getPlaybackFrameRate();
      assertEquals(recomputedSecondsPerFrameRate, frameRateFromSCS, epsilon);
      
      double ticksPerCycle = computeTicksPerPlayCycle(simulateDT, recordFreq, realTimeRate, frameRate);
      double ticksPerCycleFromSCS = scs.getTicksPerPlayCycle();
      assertEquals(ticksPerCycle, ticksPerCycleFromSCS, epsilon);
   }
   
   @After
   public void closeSCS()
   {
      scs.closeAndDispose();
      scs = null;
   }
   
   // local methods
   
   private void setCameraTrackingDoubleYoVariableInSCSRegistry(String cameraTrackingVarNameX, String cameraTrackingVarNameY, String cameraTrackingVarNameZ, SimulationConstructionSet scs)
   {
      YoVariableRegistry scsRegistry = scs.getRootRegistry();
      
      DoubleYoVariable cameraTrackingDouBleYoVariableX = new DoubleYoVariable(cameraTrackingVarNameX, scsRegistry);
      DoubleYoVariable cameraTrackingDouBleYoVariableY = new DoubleYoVariable(cameraTrackingVarNameY, scsRegistry);
      DoubleYoVariable cameraTrackingDouBleYoVariableZ = new DoubleYoVariable(cameraTrackingVarNameZ, scsRegistry);
   }
   
   private void setCameraDollyDoubleYoVariableInSCSRegistry(String cameraDollyVarNameX, String cameraDollyVarNameY, String cameraDollyVarNameZ, SimulationConstructionSet scs)
   {
      YoVariableRegistry scsRegistry = scs.getRootRegistry();
      
      DoubleYoVariable cameraDollyDouBleYoVariableX = new DoubleYoVariable(cameraDollyVarNameX, scsRegistry);
      DoubleYoVariable cameraDollyDouBleYoVariableY = new DoubleYoVariable(cameraDollyVarNameY, scsRegistry);
      DoubleYoVariable cameraDollyDouBleYoVariableZ = new DoubleYoVariable(cameraDollyVarNameZ, scsRegistry);
   }
   
   private String getRegistryNameSpaceFromRobot(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getNameSpace().getName();
   }
   
   private String getFirstVariableNameFromRobotRegistry(Robot robotModel)
   {
      return robotModel.getRobotsYoVariableRegistry().getAllVariablesArray()[0].getName();
   }
   
   private SimulationConstructionSet createAndStartSCSWithRobot(Robot robotModel)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel);
      scs.setFrameMaximized();
      scs.startOnAThread();
      return scs;
   }
   
   private void setInputAndOutputPointsInSCS(SimulationConstructionSet scs, int inputPointIndex, int outputPointIndex)
   {
      setInputPointInSCS(scs, inputPointIndex);
      setOutputPointInSCS(scs, outputPointIndex);
   }
   
   private void setInputPointInSCS(SimulationConstructionSet scs, int inputPointIndex)
   {
      scs.setIndex(inputPointIndex);
      scs.setInPoint();
   }
   
   private void setOutputPointInSCS(SimulationConstructionSet scs, int outputPointIndex)
   {
      scs.setIndex(outputPointIndex);
      scs.setOutPoint();
   }
   
   private ArrayList<YoVariable> getSimpleRobotVariablesThatContain(String searchString, boolean caseSensitive, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable> ret = null;

      if (currentlyMatched != null)
      {
         if (!caseSensitive)
         {
            searchString = searchString.toLowerCase();
         }

         for (int i = 0; i < currentlyMatched.size(); i++)
         {
            YoVariable entry = currentlyMatched.get(i);

            if (entry.getName().toLowerCase().contains((searchString)))
            {
               if (ret == null)
               {
                  ret = new ArrayList<YoVariable>();
               }

               ret.add(entry);
            }
         }
      }

      return ret;
   }
   
   private ArrayList<YoVariable> getSimpleRobotVariablesThatStartWith(String searchString, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
      ArrayList<YoVariable> ret = null;

      for (int i = 0; i < currentlyMatched.size(); i++)
      {
         YoVariable Variable = currentlyMatched.get(i);

         if (Variable.getName().startsWith(searchString))
         {
            if (ret == null)
            {
               ret = new ArrayList<YoVariable>();
            }

            ret.add(Variable);
         }
      }

      return ret;
   }
   
   private String[] getVariableNamesGivenArrayListOfYoVariables(ArrayList<YoVariable> yoVariableList)
   {
      String[] ret = null;

      for (int i = 0; i < yoVariableList.size(); i++)
      {
         String variableName = yoVariableList.get(i).getName();

         if (ret == null)
         {
            ret = new String[yoVariableList.size()];
         }

         ret[i] = variableName;
      }

      return ret;
   }
   
   private ArrayList<YoVariable> getSimpleRobotRegExpVariables(String[] varNames, String[] regularExpressions, Robot robotModel)
   {
      ArrayList<YoVariable> currentlyMatched = robotModel.getAllVariables();
      YoVariableList tempList = new YoVariableList("temp");

      for (int i = 0; i < currentlyMatched.size(); i++)
      {
         YoVariable var = currentlyMatched.get(i);

         tempList.addVariable(var);
      }

      return tempList.getMatchingVariables(varNames, regularExpressions);
   }
   
   private int computeRecordFreq(double recordDT, double simulateDT)
   {
      return (int) Math.round(recordDT / simulateDT);
   }
   
   private double recomputeTiming(double dt, double recordFreq, double realTimeRate, double frameRate)
   {
      double ticksPerCycle = computeTicksPerPlayCycle(dt, recordFreq, realTimeRate, frameRate);
      double secondsPerFrameRate = ticksPerCycle * (dt * recordFreq) / realTimeRate;
      return secondsPerFrameRate;
   }
   
   private double computeTicksPerPlayCycle(double dt, double recordFreq, double realTimeRate, double frameRate)
   {
      return Math.max((int) (frameRate * realTimeRate / (dt * recordFreq)), 1);
   }
   
   // unused
   private ArrayList<Graphics3DObject> createRandomGraphics3DArrayList(Graphics3DObject staticLinkGraphics)
   {
      ArrayList<Graphics3DObject> graphics3DArrayList = new ArrayList<>();
      for(int i = 0; i < Math.round(Math.random()*10); i++)
      {
         graphics3DArrayList.add(staticLinkGraphics);
      }
      
      return graphics3DArrayList;
   }
}