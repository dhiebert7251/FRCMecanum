// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Drivetrain extends SubsystemBase {


  //drivetrain speed controllers    
  Spark leftFrontMotor;
  Spark rightFrontMotor;
  Spark leftRearMotor;
  Spark rightRearMotor;

  //drivetrain type
  MecanumDrive drive;

  //sensors
  Gyro gyro;

  
    /** Creates a new Drivetrain. */
    public Drivetrain() {
      leftFrontMotor = new Spark(Constants.LEFT_FRONT_MOTOR);
      leftFrontMotor.setInverted(false);
      leftRearMotor = new Spark(Constants.LEFT_REAR_MOTOR);
      leftRearMotor.setInverted(false);
      rightFrontMotor = new Spark(Constants.RIGHT_FRONT_MOTOR);
      rightFrontMotor.setInverted(false);
      rightRearMotor = new Spark(Constants.RIGHT_REAR_MOTOR);
      rightRearMotor.setInverted(false);
    
      drive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
      
 
    }
  
    public void driveWithJoysticks(double ySpeed, double xSpeed, double zRotation)
    {
      drive.driveCartesian(ySpeed, xSpeed, zRotation);
    }

    public void rotateFifteen(double zRotation)
    {
      drive.driveCartesian(0,0,zRotation);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    public void stop() {
      drive.stopMotor();
    }
}
