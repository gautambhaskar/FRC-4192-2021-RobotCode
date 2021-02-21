// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;

import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.GripPipelineNew;
import org.opencv.core.MatOfPoint;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private VisionThread visionThread;
  private double maxCenterX = 0.0;
  private double centerX = 0.0;
  private double area = 0.0;

  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;

  private final Object imgLock = new Object();
  NetworkTableEntry s_centerX, s_frameCnt, s_area;
  int frameCnt = 0;

  public Vision() {
    init();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void init() {
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);
    s_centerX = Shuffleboard.getTab("Camera").add("GRIP centerX", 0).getEntry();
    s_frameCnt = Shuffleboard.getTab("Camera").add("GRIP frame count", 0).getEntry();
    s_area = Shuffleboard.getTab("Camera").add("Grip area", 0).getEntry();
    visionThread = new VisionThread(camera, new GripPipelineNew(), pipeline -> {
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

  public void collectData() {
    synchronized (imgLock) {
      s_centerX.setDouble(this.centerX);
      s_frameCnt.setDouble(this.frameCnt);
      s_area.setDouble(this.area);
      maxCenterX = this.centerX;
    }

    edu.wpi.first.wpilibj.Timer.delay(1.0 / 5.0);
  }

  public double getMaxCenterX() {
    return maxCenterX;
  }

  public void stopThread() {
    visionThread.interrupt();
  }
}
