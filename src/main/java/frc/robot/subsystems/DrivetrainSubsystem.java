// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Functions;

public class DrivetrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static double RobotYawAngle = 0;
  public static double RobotYawRate = 0;
  public static double RobotYawOld = 0;
  public DrivetrainSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called ever 0.02 seconds
    RobotYawAngle = Functions.DeltaAngleDeg(Constants.gyro.getYaw(), 0);
    RobotYawRate = -(Constants.gyro.getYaw() - RobotYawOld)/0.02;
    RobotYawOld = Constants.gyro.getYaw();
    SmartDashboard.putNumber("GyroYaw", RobotYawAngle);
    SmartDashboard.putNumber("YawRate", RobotYawRate);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
  
  public static void DriveFieldOrientedForward(double x, double y) {
    double angle = -(Math.toDegrees(Math.atan2(x, y)));
    double speed = Math.sqrt((x*x)+(y*y));
    DriveShortestAngle(angle, speed);
  }
  
  public static void DriveShortestAngle(double angle, double speed) {
    /*if (Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle)) > Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle + 180))) {
      DriveFieldOrientedAngle(angle + 180, -speed);
    }
    else {
      DriveFieldOrientedAngle(angle, speed);
    }*/
    DriveFieldOrientedAngle((Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle)) > Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle + 180)))?angle+180:angle, (Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle)) > Math.abs(Functions.DeltaAngleDeg(RobotYawAngle, angle + 180)))?-speed:speed);
  }
  public static void DriveFieldOrientedAngle (double angle, double speed) {
    double throttle = speed;
    if(Math.abs(Functions.DeltaAngleDeg(angle, Constants.gyro.getYaw())) > 10) {
      throttle = 0;
    }
    if(Math.abs(speed) < 0.001) {
      angle = Constants.gyro.getYaw();
    }
    SmartDashboard.putNumber("angle", angle);
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("deltaAngle", Functions.DeltaAngleDeg(angle, Constants.gyro.getYaw()));

    Drive(throttle, Functions.Clamp(Functions.DeltaAngleDeg(angle, Constants.gyro.getYaw()) * -(Constants.pMult),-Constants.maxTurnSpeed,Constants.maxTurnSpeed));
  }
  public static void Drive(double forwardInput, double turnInput) {
    Constants.flMotor.set((forwardInput + turnInput));
    Constants.frMotor.set((forwardInput - turnInput));
    Constants.blMotor.set((forwardInput + turnInput));
    Constants.brMotor.set(-(forwardInput - turnInput));
  }
}
