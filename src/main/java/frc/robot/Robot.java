/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.visionPosition.blueA;
import frc.robot.Constants.visionPosition.blueB;
import frc.robot.Constants.visionPosition.redA;
import frc.robot.Constants.visionPosition.redB;

import org.opencv.core.MatOfPoint;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static Trajectory testTrajectory;
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private VisionThread visionThread;
  private double maxCenterX = 0.0;
  private double centerX = 0.0;
  private double area = 0.0;
  private ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
  private ShuffleboardTab cameraTab = Shuffleboard.getTab("Camera");
  NetworkTableEntry pathChosen = mainTab.add("Path Chosen", "none").getEntry();

  private final Object imgLock = new Object();
  NetworkTableEntry s_centerX, s_frameCnt, s_area;
  int frameCnt = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    Shuffleboard.selectTab("Main");
    mainTab.add("Code Version No.", 1.0);
    mainTab.add("Branch", "main");
    m_robotContainer = new RobotContainer(() -> getMaxCenterX());
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    mainTab.add("Camera Feed", camera).withWidget(BuiltInWidgets.kCameraStream).withPosition(4, 0).withSize(3, 3);
    s_centerX = cameraTab.add("GRIP centerX", 0).getEntry();
    s_frameCnt = cameraTab.add("GRIP frame count", 0).getEntry();
    s_area = cameraTab.add("Grip area", 0).getEntry();

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
      ArrayList<MatOfPoint> contourList = pipeline.filterContoursOutput();
      if (!contourList.isEmpty()) {
        int numBalls = contourList.size();
        double maxA = 0;
        int maxIndex = 0;
        for (int i = 0; i < numBalls; i++) {
          Rect r = Imgproc.boundingRect(contourList.get(i));
          double a = r.area();
          if (a > maxA) {
            maxA = a;
            maxIndex = i;
          }
        }
        Rect r = Imgproc.boundingRect(contourList.get(maxIndex));
        synchronized (imgLock) {
          centerX = r.x + (r.width / 2);
          area = r.area();
          frameCnt++;
        }
      }
    });
    visionThread.start();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    synchronized (imgLock) {
      s_centerX.setDouble(this.centerX);
      s_frameCnt.setDouble(this.frameCnt);
      s_area.setDouble(this.area);
      maxCenterX = this.centerX;
      if (maxCenterX > blueA.left && maxCenterX < blueA.right) {
        pathChosen.setString("Blue A");
        Globals.pathChosen = 1;
      }

      else if (maxCenterX > blueB.left && maxCenterX < blueB.right) {
        pathChosen.setString("Blue B");
        Globals.pathChosen = 2;
      }

      else if (maxCenterX > redA.left && maxCenterX < redA.right) {
        pathChosen.setString("Red A");
        Globals.pathChosen = 3;
      }

      else if (maxCenterX > redB.left && maxCenterX < redB.right) {
        pathChosen.setString("Red B");
        Globals.pathChosen = 4;
      }

      else {
        pathChosen.setString("none");
        Globals.pathChosen = 0;
      }
    }

    edu.wpi.first.wpilibj.Timer.delay(1.0 / 5.0);

  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    visionThread.interrupt();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    visionThread.interrupt();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public double getMaxCenterX() {
    return maxCenterX;
  }
}
