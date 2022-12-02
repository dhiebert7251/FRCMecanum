// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Drivetrain extends SubsystemBase {


  //drivetrain speed controllers   
   
  CANSparkMax leftFrontMotor;
  CANSparkMax rightFrontMotor;
  CANSparkMax leftRearMotor;
  CANSparkMax rightRearMotor;

  //drivetrain type
  MecanumDrive drive;

  //sensors
  Gyro gyro;

  RelativeEncoder leftFrontMotorEncoder;
  RelativeEncoder rightFrontMotorEncoder;
  RelativeEncoder leftRearMotorEncoder;
  RelativeEncoder rightRearMotorEncoder;

  
    /** Creates a new Drivetrain. */
    public Drivetrain() {
      TODO:
      /* set motor constants to CAN bus addresses
         may need to reverse inverted motors
      */

      //Motors
      leftFrontMotor = new CANSparkMax(Constants.LEFT_FRONT_MOTOR, MotorType.kBrushless);
      leftFrontMotor.setInverted(true);
      leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR, MotorType.kBrushless);
      leftRearMotor.setInverted(true);
      rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR, MotorType.kBrushless);
      rightFrontMotor.setInverted(false);
      rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR, MotorType.kBrushless);
      rightRearMotor.setInverted(false);
    
      //mecanum drivetrain
      drive = new MecanumDrive(leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor);
      
      //Sensors

      //Gyro
      ADXRS450_Gyro gyro = new ADXRS450_Gyro();

      //Drive motor encoders
      leftFrontMotorEncoder = leftFrontMotor.getEncoder();
      leftRearMotorEncoder = leftRearMotor.getEncoder();
      rightFrontMotorEncoder = rightFrontMotor.getEncoder();
      rightRearMotorEncoder = rightRearMotor.getEncoder();


      //Shuffleboard data  
      TODO:
      /*
      update SmartDashboard data to Shuffleboard data

      */

      Shuffleboard.getTab("Telemetry").add(gyro);
      SmartDashboard.putNumber("Front Left Position",leftFrontMotorEncoder.getPosition());
      SmartDashboard.putNumber("Front Left Velocity",leftFrontMotorEncoder.getVelocity());
      SmartDashboard.putNumber("Front Right Position",rightFrontMotorEncoder.getPosition());
      SmartDashboard.putNumber("Front Right Velocity",rightFrontMotorEncoder.getVelocity());
      SmartDashboard.putNumber("Rear Left Position",leftRearMotorEncoder.getPosition());
      SmartDashboard.putNumber("Rear Left Velocity",leftRearMotorEncoder.getVelocity());       
      SmartDashboard.putNumber("Rear Right Position", rightRearMotorEncoder.getPosition());
      SmartDashboard.putNumber("Rear Right Velocity",rightRearMotorEncoder.getVelocity());     


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

    public void update() {
      //potentially used to update shuffleboard sensor data

    }
    
}
