// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  //i think i gotta put this in here to make it work in other methods but this is weird and i dont understand it 
  private SparkClosedLoopController controllerFRDrive;
  private SparkClosedLoopController controllerRRDrive;
  private SparkClosedLoopController controllerRLDrive;
  private SparkClosedLoopController controllerFLDrive;

  private SparkClosedLoopController controllerFRSteer;
  private SparkClosedLoopController controllerRRSteer;
  private SparkClosedLoopController controllerRLSteer;
  private SparkClosedLoopController controllerFLSteer;

  //constant mulitples of velocities to scale inputs
  double kSteer = 1; 
  double kDrive = 1; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    //initialize motor controllers, these ids are definitely wrong and need to be changed before testing
    SparkMax motorFRDrive = new SparkMax(1, MotorType.kBrushless);
    SparkMax motorRRDrive = new SparkMax(3, MotorType.kBrushless);
    SparkMax motorRLDrive = new SparkMax(5, MotorType.kBrushless);
    SparkMax motorFLDrive = new SparkMax(7, MotorType.kBrushless);

    SparkMax motorFRSteer = new SparkMax(2, MotorType.kBrushless);
    SparkMax motorRRSteer = new SparkMax(4, MotorType.kBrushless);
    SparkMax motorRLSteer = new SparkMax(6, MotorType.kBrushless);
    SparkMax motorFLSteer = new SparkMax(8, MotorType.kBrushless);

    //initialize internal close loop controllers
    SparkClosedLoopController controllerFRDrive = motorFRDrive.getClosedLoopController();
    SparkClosedLoopController controllerRRDrive = motorRRDrive.getClosedLoopController();
    SparkClosedLoopController controllerRLDrive = motorRLDrive.getClosedLoopController();
    SparkClosedLoopController controllerFLDrive = motorFLDrive.getClosedLoopController();

    SparkClosedLoopController controllerFRSteer = motorFRSteer.getClosedLoopController();
    SparkClosedLoopController controllerRRSteer = motorRRSteer.getClosedLoopController();
    SparkClosedLoopController controllerRLSteer = motorRLSteer.getClosedLoopController();
    SparkClosedLoopController controllerFLSteer = motorFLSteer.getClosedLoopController();

    //initialize internal encoders
    RelativeEncoder encoderFRDrive = motorFRDrive.getEncoder();
    RelativeEncoder encoderRRDrive = motorRRDrive.getEncoder();
    RelativeEncoder encoderRLDrive = motorRLDrive.getEncoder();
    RelativeEncoder encoderFLDrive = motorFLDrive.getEncoder();
    
    RelativeEncoder encoderFRSteer = motorFRSteer.getEncoder();
    RelativeEncoder encoderRRSteer = motorRRSteer.getEncoder();
    RelativeEncoder encoderRLSteer = motorRLSteer.getEncoder();
    RelativeEncoder encoderFLSteer = motorFLSteer.getEncoder();
    
    //set config for drive motors
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    //set pid constants, these have not been tuned and the output range should be determined asap, velocity feed forward should be right though
    driveConfig.closedLoop
    .p(0.01)
    .i(0)
    .d(0)
    .outputRange(-kDrive, kDrive)
    //feed forward, the docs just said to put this in for velocity controllers
    .velocityFF(1/473);

    //set conversion factors for encoders, position is in revolutions and velocity in rpm i think so this will need to be changed
    driveConfig.encoder
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(Math.PI / 30);

    //apply config to drive motorsm
    motorFRDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRRDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRLDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFLDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //set config for steering motors
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    //set pid constants, these have not been tuned and the output range should be determined asap
    steerConfig.closedLoop
    .p(0.01)
    .i(0)
    .d(0)
    .outputRange(-1, 1);

    //set conversion factors for encoders, position is in revolutions and velocity in rpm i think so this will need to be changed
    steerConfig.encoder
        .positionConversionFactor(1)
        .velocityConversionFactor(1);

    //apply config to steering motors
    motorFRSteer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRRSteer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRLSteer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFLSteer.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /** fetch controller inputs
     * do math to find what psotions and velocities should be
     * give commands to motor controllers and update internal pids
     */
     
     //define offsets of wheels from center in m, assuming perfect rectangle
     double offsetX = 1;
     double offsetY = 1;
     
     //putting this here temporarily as i dont know what the code is for the controller bc BEN wont tell mE WHERE THE GITHUB IS
     double driveY = 0;
     double driveX = 0;
     double steer = 0; 

     double speed = kDrive * Math.sqrt((driveX * driveX) + (driveY * driveY));
     double angle = Math.atan(driveY/driveX);
     
     double omega = kSteer * steer;
    
     //case when no steering input, still works if speed input is zero
     if(steer == 0){
      //set drive motors to the length of the velocity vector
      controllerFRDrive.setReference(speed, ControlType.kVelocity);
      controllerRRDrive.setReference(speed, ControlType.kVelocity);
      controllerRLDrive.setReference(speed, ControlType.kVelocity);
      controllerFLDrive.setReference(speed, ControlType.kVelocity);

      //set steering motors to the angle of the velocity vector
      controllerFRSteer.setReference(angle, ControlType.kPosition);
      controllerRRSteer.setReference(angle, ControlType.kPosition);
      controllerRLSteer.setReference(angle, ControlType.kPosition);
      controllerFLSteer.setReference(angle, ControlType.kPosition);
     }

     //case when no drive input, but still steering
     else if((driveX == 0) && (driveY) == 0){
      double radiusFRX = -offsetX;
      double radiusFRY = -offsetY;
      double radiusRRX = -offsetX;
      double radiusRRY = offsetY;
      double radiusRLX = offsetX;
      double radiusRLY = offsetY;
      double radiusFLX = offsetX;
      double radiusFLY = - offsetY;

      double speedFR = kDrive * Math.sqrt((radiusFRX * radiusFRX) + (radiusFRY * radiusFRY));
      double speedRR = kDrive * Math.sqrt((radiusRRX * radiusRRX) + (radiusRRY * radiusRRY));
      double speedRL = kDrive * Math.sqrt((radiusRLX * radiusRLX) + (radiusRLY * radiusRLY));
      double speedFL = kDrive * Math.sqrt((radiusFLX * radiusFLX) + (radiusFLY * radiusFLY));

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);

      double angleFR = Math.atan(radiusFRY/radiusFRX);
      double angleRR = Math.atan(radiusRRY/radiusRRX);
      double angleRL = Math.atan(radiusRLY/radiusRLX);
      double angleFL = Math.atan(radiusFLY/radiusFLX);

      controllerFRSteer.setReference(angleFR, ControlType.kPosition);
      controllerRRSteer.setReference(angleRR, ControlType.kPosition);
      controllerRLSteer.setReference(angleRL, ControlType.kPosition);
      controllerFLSteer.setReference(angleFL, ControlType.kPosition);
     }

     //case when there is drive and steering input
     else{
      double radiusX = kDrive * driveY / omega;
      double radiusY = -1 * kDrive * driveX / omega;
      double radius = Math.sqrt((radiusX * radiusX) + (radiusY * radiusY));
      
      double radiusFRX = radiusX - offsetX;
      double radiusFRY = radiusY - offsetY;
      double radiusRRX = radiusX - offsetX;
      double radiusRRY = radiusY + offsetY;
      double radiusRLX = radiusX + offsetX;
      double radiusRLY = radiusY + offsetY;
      double radiusFLX = radiusX + offsetX;
      double radiusFLY = radiusY - offsetY;

      double speedFR = speed * Math.sqrt((radiusFRX * radiusFRX) + (radiusFRY * radiusFRY)) / radius;
      double speedRR = speed * Math.sqrt((radiusRRX * radiusRRX) + (radiusRRY * radiusRRY)) / radius;
      double speedRL = speed * Math.sqrt((radiusRLX * radiusRLX) + (radiusRLY * radiusRLY)) / radius;
      double speedFL = speed * Math.sqrt((radiusFLX * radiusFLX) + (radiusFLY * radiusFLY)) / radius;

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);

      double angleFR = Math.atan(radiusFRY/radiusFRX);
      double angleRR = Math.atan(radiusRRY/radiusRRX);
      double angleRL = Math.atan(radiusRLY/radiusRLX);
      double angleFL = Math.atan(radiusFLY/radiusFLX);

      controllerFRSteer.setReference(angleFR, ControlType.kPosition);
      controllerRRSteer.setReference(angleRR, ControlType.kPosition);
      controllerRLSteer.setReference(angleRL, ControlType.kPosition);
      controllerFLSteer.setReference(angleFL, ControlType.kPosition);
     }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
