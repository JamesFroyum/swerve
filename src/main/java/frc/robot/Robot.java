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

import edu.wpi.first.wpilibj.Joystick;
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

  //ok i think this is declaring the objects here so they can be used by all of the loops nested within, but still initialized in the robot loop
  private SparkMax motorFRDrive;
  private SparkMax motorRRDrive;
  private SparkMax motorRLDrive;
  private SparkMax motorFLDrive;

  private SparkMax motorFRSteer;
  private SparkMax motorRRSteer;
  private SparkMax motorRLSteer;
  private SparkMax motorFLSteer;
  
  private SparkClosedLoopController controllerFRDrive;
  private SparkClosedLoopController controllerRRDrive;
  private SparkClosedLoopController controllerRLDrive;
  private SparkClosedLoopController controllerFLDrive;

  private SparkClosedLoopController controllerFRSteer;
  private SparkClosedLoopController controllerRRSteer;
  private SparkClosedLoopController controllerRLSteer;
  private SparkClosedLoopController controllerFLSteer;

  private RelativeEncoder encoderFRDrive;
  private RelativeEncoder encoderRRDrive;
  private RelativeEncoder encoderRLDrive;
  private RelativeEncoder encoderFLDrive;

  private RelativeEncoder encoderFRSteer;
  private RelativeEncoder encoderRRSteer;
  private RelativeEncoder encoderRLSteer;
  private RelativeEncoder encoderFLSteer;

  private Joystick joystick;

  //ok lets make some constants and variables
  //constant mulitples of velocities to scale inputs
  final double kSteer = 1; 
  final double kDrive = 100; 

  //define offsets of wheels from center in m, assuming perfect rectangle
  final double offsetX = 1;
  final double offsetY = 1;

  //define inputs
  double driveY = 0;
  double driveX = 0;
  double steer = 0; 

  //define variables for mathh
  double speed = 0;
  double angle = 0;
  double lastAngle = 0;
  double angleOffset = 0;
  double omega = 0;

  double radiusFRX = 0;
  double radiusFRY = 0;
  double radiusRRX = 0;
  double radiusRRY = 0;
  double radiusRLX = 0;
  double radiusRLY = 0;
  double radiusFLX = 0;
  double radiusFLY = 0;

  double speedFR = 0;
  double speedRR = 0;
  double speedRL = 0;
  double speedFL = 0;

  double angleFR = 0;
  double angleRR = 0;
  double angleRL = 0;
  double angleFL = 0;

  double radiusX = 0;
  double radiusY = 0;
  double radius = 0; 

  final double steerRatio = (7);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    
    //initialize motor controllers
    motorFRDrive = new SparkMax(2, MotorType.kBrushless);
    motorRRDrive = new SparkMax(4, MotorType.kBrushless);
    motorRLDrive = new SparkMax(5, MotorType.kBrushless);
    motorFLDrive = new SparkMax(7, MotorType.kBrushless);

    motorFRSteer = new SparkMax(1, MotorType.kBrushless);
    motorRRSteer = new SparkMax(3, MotorType.kBrushless);
    motorRLSteer = new SparkMax(6, MotorType.kBrushless);
    motorFLSteer = new SparkMax(8, MotorType.kBrushless);

    //initialize internal close loop controllers
    controllerFRDrive = motorFRDrive.getClosedLoopController();
    controllerRRDrive = motorRRDrive.getClosedLoopController();
    controllerRLDrive = motorRLDrive.getClosedLoopController();
    controllerFLDrive = motorFLDrive.getClosedLoopController();

    controllerFRSteer = motorFRSteer.getClosedLoopController();
    controllerRRSteer = motorRRSteer.getClosedLoopController();
    controllerRLSteer = motorRLSteer.getClosedLoopController();
    controllerFLSteer = motorFLSteer.getClosedLoopController();

    //initialize internal encoders
    encoderFRDrive = motorFRDrive.getEncoder();
    encoderRRDrive = motorRRDrive.getEncoder();
    encoderRLDrive = motorRLDrive.getEncoder();
    encoderFLDrive = motorFLDrive.getEncoder();
    
    encoderFRSteer = motorFRSteer.getEncoder();
    encoderRRSteer = motorRRSteer.getEncoder();
    encoderRLSteer = motorRLSteer.getEncoder();
    encoderFLSteer = motorFLSteer.getEncoder();
    
    //initialize joystick
    joystick = new Joystick(0);
    
    //set config for drive motors
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    //set pid constants, these have not been tuned and the output range should be determined asap, velocity feed forward should be right though
    driveConfig.closedLoop
    .p(0.002)
    .i(0)
    .d(0)
    .outputRange(-kDrive, kDrive)
    //feed forward, the docs just said to put this in for velocity controllers
    .velocityFF(1/473);

    //when you change conversion factors, multiply pid constants by inverse
    //set conversion factors for encoders, position is in revolutions and velocity in rpm i think, so i converted to radians and rad/s for math
    driveConfig.encoder
        .positionConversionFactor(2 * Math.PI)
        .velocityConversionFactor(Math.PI/30);

    //apply config to drive motors
    motorFRDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRRDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorRLDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorFLDrive.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //set config for steering motors
    SparkMaxConfig steerConfig = new SparkMaxConfig();

    //set pid constants, these have not been tuned and the output range should be determined asap
    steerConfig.closedLoop
    .p(0.01)
    .i(0.0)
    .d(0.0)
    .outputRange(-100, 100);

    //set conversion factors for encoders, position is in revolutions and velocity in rpm i think so this will need to be changed
    steerConfig.encoder
        .positionConversionFactor(2*Math.PI)
        .velocityConversionFactor(1);

    //motorFRDrive.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //motorRRDrive.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //motorRLDrive.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //motorFLDrive.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

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
     
    //set inputs, theres no input smoothing here so should prolly do that at some point
    driveY = -1 * joystick.getRawAxis(5);
    driveX = joystick.getRawAxis(4);
    steer = Math.PI * joystick.getRawAxis(0); 
    //System.out.println("Steer: " + steer);

    speed = kDrive * Math.sqrt((driveX * driveX) + (driveY * driveY));
    angle = steerRatio * Math.atan(driveY/driveX);
     
    if(angle/Math.abs(angle) != lastAngle){
      if(lastAngle == -1){
        if(driveY > 0){
          angleOffset -= steerRatio * Math.PI;
        }
        else{
          angleOffset += steerRatio * Math.PI;
        }
      }
      else{
        if(driveY > 0){
          angleOffset += steerRatio * Math.PI;
        }
        else{
          angleOffset -= steerRatio * Math.PI;
        }
      }
    }
     
    System.out.println("alright its me waluigi and im tired but heres the angle and lastAgle i guess");
    System.out.println(angle);
    System.out.println(lastAngle);
    omega = kSteer * steer;
    lastAngle = (angle)/Math.abs(angle);
    angle += angleOffset;

    //case when no steering input, still works if speed input is zero
    if((steer >= -0.05) && (steer <= 0.05)){
      //.out.println("ayy its me mario im setting the the motors to drive");
      //System.out.println(speed);
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
    else if((driveX >= -0.05) && (driveX <= 0.05) && (driveY >= -0.05) && (driveY <= 0.05)){
      radiusFRX = -offsetX;
      radiusFRY = -offsetY;
      radiusRRX = -offsetX;
      radiusRRY = offsetY;
      radiusRLX = offsetX;
      radiusRLY = offsetY;
      radiusFLX = offsetX;
      radiusFLY = - offsetY;

      //System.out.println("HEeeyy its a me luigi nd im gonna set the motors to steer. mweahahahahaha");
      //System.out.println(speed);

      speedFR = kDrive * omega * Math.sqrt((radiusFRX * radiusFRX) + (radiusFRY * radiusFRY));
      speedRR = kDrive * omega * Math.sqrt((radiusRRX * radiusRRX) + (radiusRRY * radiusRRY));
      speedRL = kDrive * omega * Math.sqrt((radiusRLX * radiusRLX) + (radiusRLY * radiusRLY));
      speedFL = kDrive * omega * Math.sqrt((radiusFLX * radiusFLX) + (radiusFLY * radiusFLY));

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);

      angleFR = steerRatio * Math.atan(radiusFRY/radiusFRX);
      angleRR = steerRatio * Math.atan(radiusRRY/radiusRRX);
      angleRL = steerRatio * Math.atan(radiusRLY/radiusRLX);
      angleFL = steerRatio * Math.atan(radiusFLY/radiusFLX);

      controllerFRSteer.setReference(angleFR, ControlType.kPosition);
      controllerRRSteer.setReference(angleRR, ControlType.kPosition);
      controllerRLSteer.setReference(angleRL, ControlType.kPosition);
      controllerFLSteer.setReference(angleFL, ControlType.kPosition);
    }

    //case when there is drive and steering input
    else{
      radiusX = kDrive * driveY / omega;
      radiusY = -1 * kDrive * driveX / omega;
      radius = Math.sqrt((radiusX * radiusX) + (radiusY * radiusY));
      
      radiusFRX = radiusX - offsetX;
      radiusFRY = radiusY - offsetY;
      radiusRRX = radiusX - offsetX;
      radiusRRY = radiusY + offsetY;
      radiusRLX = radiusX + offsetX;
      radiusRLY = radiusY + offsetY;
      radiusFLX = radiusX + offsetX;
      radiusFLY = radiusY - offsetY;

      speedFR = speed * Math.sqrt((radiusFRX * radiusFRX) + (radiusFRY * radiusFRY)) / radius;
      speedRR = speed * Math.sqrt((radiusRRX * radiusRRX) + (radiusRRY * radiusRRY)) / radius;
      speedRL = speed * Math.sqrt((radiusRLX * radiusRLX) + (radiusRLY * radiusRLY)) / radius;
      speedFL = speed * Math.sqrt((radiusFLX * radiusFLX) + (radiusFLY * radiusFLY)) / radius;

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);

      angleFR = steerRatio * Math.atan(radiusFRY/radiusFRX);
      angleRR = steerRatio * Math.atan(radiusRRY/radiusRRX);
      angleRL = steerRatio * Math.atan(radiusRLY/radiusRLX);
      angleFL = steerRatio * Math.atan(radiusFLY/radiusFLX);

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
