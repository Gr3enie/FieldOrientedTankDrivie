// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final Pigeon2 gyro = new Pigeon2(2);
  
  public static final VictorSP frMotor = new VictorSP(2);
  public static final VictorSP flMotor = new VictorSP(0);
  public static final VictorSP brMotor = new VictorSP(3);
  public static final VictorSP blMotor = new VictorSP(1);
  public static final double controllerDeadZone = 0.1;
  public static final double pMult = 0.02;
  public static final double dMult = 0.002;
  public static final double maxTurnSpeed = 0.5;
  
  public static final XboxController controller1 = new XboxController(0);
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
