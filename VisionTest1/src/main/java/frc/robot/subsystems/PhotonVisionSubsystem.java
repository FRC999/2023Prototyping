// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  /** Creates a new PhotoVisionSubsystem. */

  PhotonCamera camera = new PhotonCamera("Razer_Kiyo");
  boolean hasTargets;
    List<PhotonTrackedTarget> targets;
  // Get the current best target.
  // Query the latest result from PhotonVision
  

  
  
  public PhotonVisionSubsystem() {
    getYawPitch();
    getCorners();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }

  public void getTargets() {

    
  }

  public void getRealTime() {
  }

  public double getDistance() {
    return 0;
    
  }

  public double getCorners() {
    var result = camera.getLatestResult();
    
    if (!result.hasTargets()) { 
      System.out.println("No target connected" ); 
      return 0;
    }
    
    PhotonTrackedTarget bestTarget = result.getBestTarget();
    double sumY = 0;
    double sumX = 0;
    double[][] cornersArray = new double[4][2];
    
    List<TargetCorner> corners = bestTarget.getDetectedCorners();
    for(int i = 0; i<4; i++){
      cornersArray[i][0] = corners.get(i).x;
      cornersArray[i][1] = corners.get(i).y;
    }
    System.out.println("Corners Array");
    for(int i = 0; i<4; i++){
      System.out.println(cornersArray[i][0]+ "," + cornersArray[i][1]);
    }

    for(int i=3; i>=1; i--) {
      for (int j=0; j<i; j++) {
        if(cornersArray[j][1]>cornersArray[j+1][1]){
          double tempX = cornersArray[j][0];
          double tempY = cornersArray[j][1];
          cornersArray[j][0]=cornersArray[j+1][0];
          cornersArray[j][1]=cornersArray[j+1][1];
          cornersArray[j+1][0]=tempX;
          cornersArray[j+1][1]=tempY;
        }
      }
    }
    System.out.println("Y ordered Array");
    for(int i = 0; i<4; i++){
      System.out.println(cornersArray[i][0]+ "," + cornersArray[i][1]);
    }

    double yBottom = (cornersArray[0][1] + cornersArray[1][1])/2;
    double yTop = (cornersArray[2][1] + cornersArray[3][1])/2;
    double yLength = yTop - yBottom;
    double[][] distancePoints = {{30, 182.4}, {40, 133.0}, {50, 107.1}, {60, 88.8}, {70, 76.8}, {80, 66.6}, {90, 59.9}, {100, 53.3}, {110, 48.1}, {120, 44.1}};
    
    for (int i = 0; i<10; i++) {
      if (yLength>distancePoints[i][i]) {
      }
    }
    return yLength;
    
  }

  public void getCornersAngle() {
    var result = camera.getLatestResult();
    
    if (!result.hasTargets()) { 
      System.out.println("No target connected" ); 
    }
    
    PhotonTrackedTarget bestTarget = result.getBestTarget();
    double sumY = 0;
    double sumX = 0;
    double[][] cornersArray = new double[4][2];
    List<TargetCorner> corners = bestTarget.getDetectedCorners();
    for(int i = 0; i<4; i++){
      cornersArray[i][0] = corners.get(i).x;
      cornersArray[i][1] = corners.get(i).y;
    }
    System.out.println("Corners Array");
    for(int i = 0; i<4; i++){
      System.out.println(cornersArray[i][0]+ "," + cornersArray[i][1]);
    }

    for(int i=3; i>=1; i--) {
      for (int j=0; j<i; j++) {
        if(cornersArray[j][0]>cornersArray[j+1][0]){
          double tempX = cornersArray[j][0];
          double tempY = cornersArray[j][1];
          cornersArray[j][0]=cornersArray[j+1][0];
          cornersArray[j][1]=cornersArray[j+1][1];
          cornersArray[j+1][0]=tempX;
          cornersArray[j+1][1]=tempY;
        }
      }
    }
    System.out.println("X ordered Array");
    for(int i = 0; i<4; i++){
      System.out.println(cornersArray[i][0]+ "," + cornersArray[i][1]);
    }
    double xLength = ((cornersArray[2][0]+cornersArray[3][0])/2) - ((cornersArray[0][0]+cornersArray[1][0])/2);

    for(int i=3; i>=1; i--) {
      for (int j=0; j<i; j++) {
        if(cornersArray[j][1]>cornersArray[j+1][1]){
          double tempX = cornersArray[j][0];
          double tempY = cornersArray[j][1];
          cornersArray[j][0]=cornersArray[j+1][0];
          cornersArray[j][1]=cornersArray[j+1][1];
          cornersArray[j+1][0]=tempX;
          cornersArray[j+1][1]=tempY;
        }
      }
    }
    System.out.println("Y ordered Array");
    for(int i = 0; i<4; i++){
      System.out.println(cornersArray[i][0]+ "," + cornersArray[i][1]);
    }

    double yLength = ((cornersArray[2][1]+cornersArray[3][1])/2) - ((cornersArray[0][1]+cornersArray[1][1])/2);
    System.out.println("X length");
    System.out.println(xLength);
    System.out.println("Y length");
    System.out.println(yLength);

    double angle = Math.acos(xLength/yLength)*180/Math.PI;
    System.out.println("Angle");
    System.out.println(angle);

  }


  public String getYawPitch() {

    var result = camera.getLatestResult();
    hasTargets = result.hasTargets();

    System.out.println("T "+hasTargets);

    targets = result.getTargets();

    if (targets ==  null) { return "-2,-2"; }

    if (targets.isEmpty()) {return "-4,-4";}

    PhotonTrackedTarget target = result.getBestTarget();

    

    if (target != null) {
      System.out.println("I see the target");
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      return "YPT:"+yaw+","+pitch+","+hasTargets; 
    } else {
      System.out.println("I DO NOT see the target");
      return "-3,-3,"+hasTargets;
    }
  }
}
