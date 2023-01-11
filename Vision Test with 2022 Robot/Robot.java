// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.photonvision.PhotonCamera;

import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import java.lang.Math;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** This is a demo program showing how to use Mecanum control with the MecanumDrive class. */
public class Robot extends TimedRobot {
  private static final int kFrontLeftChannel = 23;
  private static final int kRearLeftChannel = 22;
  private static final int kFrontRightChannel = 21;
  private static final int kRearRightChannel = 20;

  private static final int kJoystickChannel = 0;
  private static final int kJoystick1Channel = 1;

  private MecanumDrive m_robotDrive;
  private Joystick m_stick;
  private Joystick m_stick1;
  AHRS ahrs;
  double gyroOffset;

  PhotonCamera camera = new PhotonCamera("Camera1");


  CANSparkMax frontLeft;
    CANSparkMax rearLeft;
    CANSparkMax frontRight;
    CANSparkMax rearRight;



  @Override
  public void robotInit() {
    ahrs = getAHRS();
    camera.setDriverMode(true);
    

    frontLeft = new CANSparkMax(kFrontLeftChannel, MotorType.kBrushless);
    rearLeft = new CANSparkMax(kRearLeftChannel, MotorType.kBrushless);
    frontRight = new CANSparkMax(kFrontRightChannel, MotorType.kBrushless);
    rearRight = new CANSparkMax(kRearRightChannel, MotorType.kBrushless);

    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    frontRight.setInverted(true);
    rearRight.setInverted(true);

    frontLeft.setIdleMode(IdleMode.kBrake);
    rearLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    rearRight.setIdleMode(IdleMode.kBrake);


    m_robotDrive = new MecanumDrive(frontLeft, rearLeft, frontRight, rearRight);

    m_stick = new Joystick(kJoystickChannel);
    m_stick1 = new Joystick(kJoystick1Channel);
  }
public PowerDistribution examplePD = new PowerDistribution(0, ModuleType.kCTRE);

  @Override
  public void teleopInit() {

    

    gyroOffset = -ahrs.getAngle();
  }

  @Override
  public void teleopPeriodic() {
    var result = camera.getLatestResult();
    
    for(int i=0; i<=15; i++) {
      SmartDashboard.putNumber("Current " + i, examplePD.getCurrent(i));

    }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////THIS NEEDS TESTED/////////////////////////////////////////////////////////////////////////////////////////////////////////////
    SmartDashboard.putNumber("FrontLeftTemp", frontLeft.getMotorTemperature());
    SmartDashboard.putNumber("RearLeftTemp", rearLeft.getMotorTemperature());
    SmartDashboard.putNumber("FrontRightTemp", frontRight.getMotorTemperature());
    SmartDashboard.putNumber("RearRightTemp", rearRight.getMotorTemperature());

    double ymove = -m_stick.getY();
    double xmove = m_stick.getX();
    double zmove =  m_stick1.getX();
    double rSpeed = ((-m_stick.getZ() + 1) / 2);


    if (0.1 >= Math.abs(ymove)) ymove = 0;
    if (0.1 >= Math.abs(xmove)) xmove = 0;
    if (0.1 >= Math.abs(zmove)) zmove = 0;

    double gyroAngle = (ahrs.getAngle() + gyroOffset) % 360;
    SmartDashboard.putNumber("Gyro Angle", gyroAngle);
    SmartDashboard.putNumber("Gyro Offset", gyroOffset);


    // Use the joystick X axis for lateral movement, Y axis for forward
    // movement, and Z axis for rotation.
    m_robotDrive.driveCartesian(ymove*rSpeed, xmove*rSpeed, zmove*rSpeed, gyroAngle);
  }


  public static AHRS getAHRS() {
    //Starts the IMU
    try {
      /***********************************************************************
       * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C) and USB. - See
       * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
       * 
       * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
       * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
       * 
       * VMX-pi: - Communication via USB. - See
       * https://vmx-pi.kauailabs.com/installation/roborio-installation/
       * 
       * Multiple navX-model devices on a single robot are supported.
       ************************************************************************/
      AHRS ahrs = new AHRS(SPI.Port.kMXP);
      // ahrs = new AHRS(SerialPort.Port.kUSB1);
      ahrs.enableLogging(true);
      return ahrs;
    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
      return null;
    }
  }
}
