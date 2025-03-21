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

import java.util.ArrayList;

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
  final double steerRatio = (21.5);

  //constant mulitples of velocities to scale inputs
  double kSteer = 0.1; 
  double kDrive = 100; 

  //define offsets of wheels from center in m, assuming perfect rectangle
  final double offsetX = 1;
  final double offsetY = 1;

  //angle offsets for wheels
  final double wheelOffsetFR = -1/4 * 2 * Math.PI * steerRatio * 0;
  final double wheelOffsetRR = 7/10 * 2 * Math.PI * steerRatio * 0;
  final double wheelOffsetRL = 1/6 * 2 * Math.PI * steerRatio * 0;
  final double wheelOffsetFL = -1/6 * 2 * Math.PI * steerRatio * 0;

  //define inputs
  double driveY = 0;
  double driveX = 0;
  double steer = 0; 

  double currentDriveY = 0;
  double currentDriveX = 0;
  double currentSteer = 0;

  //define variables for mathh
  double speed = 0;
  double angle = 0;
  double lastX = 1;
  double angleOffset = 0;
  double omega = 0;
  double n = 0;

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

  ArrayList<Double> smoothingListDriveX;
  ArrayList<Double> smoothingListDriveY;
  ArrayList<Double> smoothingListSteer; 

  double smoothingScalar = 0;

  //smoothingFrames is zero indexed, so a value of 4 means were smoothing over 5 frames
  int smoothingFrames = 4;
  double smoothingMinimum = 0.5;
  int i = 0;
  
  public void fuckYouImPuttingItInAMethod(){
    ArrayList<Double> smoothingListDriveX = new ArrayList<Double>();
    for (i = 0; i <= smoothingFrames; i++){
      smoothingListDriveX.add(0, 0d);
    }

    ArrayList<Double> smoothingListDriveY = new ArrayList<Double>();
    for (i = 0; i <= smoothingFrames; i++){
      smoothingListDriveY.add(0, 0d);
    }

    ArrayList<Double> smoothingListSteer = new ArrayList<Double>();
    for (i = 0; i <= smoothingFrames; i++){
      smoothingListSteer.add(0, 0d);
    }

    for (i = 0; i <= smoothingFrames; i++){
      smoothingScalar = smoothingScalar + Math.pow(Math.pow(smoothingMinimum, (1 / (smoothingFrames - 1))), i);
    }
  }

  double speedFRX = 0;
  double speedRRX = 0;
  double speedRLX = 0;
  double speedFLX = 0;

  double speedFRY = 0;
  double speedRRY = 0;
  double speedRLY = 0;
  double speedFLY = 0;


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
    .outputRange(-10000, 10000);

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
    encoderFLSteer.setPosition(0);
    encoderFRSteer.setPosition(0);
    encoderRLSteer.setPosition(0);
    encoderRRSteer.setPosition(0);
    n = 0;

    angle = 0;
    angleOffset = 0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if(n == 0){
    // encoderFLSteer.setPosition(0);
    // encoderFRSteer.setPosition(0);
    // encoderRLSteer.setPosition(0);
    // encoderRRSteer.setPosition(0);
    // angleOffset = 0;
    // n += 1;
    // }
    /** fetch controller inputs
    * do math to find what psotions and velocities should be
    * give commands to motor controllers and update internal pids
    */
     
    //adding the new input to the list of inputs for smoothing, and removing the last input
    currentDriveY = -1 * joystick.getRawAxis(5);
    currentDriveX = joystick.getRawAxis(4);
    currentSteer = Math.PI * joystick.getRawAxis(0); 

    smoothingListDriveY.add(0, currentDriveY);
    smoothingListDriveX.add(0, currentDriveY);
    smoothingListSteer.add(0, currentSteer);
    
    smoothingListDriveY.remove(smoothingFrames + 1);
    smoothingListDriveX.remove(smoothingFrames + 1);
    smoothingListSteer.remove(smoothingFrames + 1);

    //finding what values should be sent to the wheels using the smoothing lists
    driveY = 0;
    for(i = 0; i <= smoothingFrames; i++){
      driveY += Math.pow(Math.pow(smoothingMinimum, (1 / (smoothingFrames - 1))), i) * smoothingListDriveY.get(i);
    }
    driveY = driveY / smoothingScalar;

    driveX = 0;
    for(i = 0; i <= smoothingFrames; i++){
      driveX += Math.pow(Math.pow(smoothingMinimum, (1 / (smoothingFrames - 1))), i) * smoothingListDriveX.get(i);
    }
    driveX = driveX / smoothingScalar;

    steer = 0;
    for(i = 0; i <= smoothingFrames; i++){
      steer += Math.pow(Math.pow(smoothingMinimum, (1 / (smoothingFrames - 1))), i) * smoothingListSteer.get(i);
    }
    steer = steer / smoothingScalar;

    //find actual values using smoothed inputs
    speed = kDrive * Math.sqrt((driveX * driveX) + (driveY * driveY));
    angle = steerRatio * Math.atan(driveY/driveX);
     
    omega = kSteer * steer;
    
    //case when no steering input, still works if speed input is zero
    if((steer >= -0.05) && (steer <= 0.05)){
      System.out.println("ayy its me mario im setting the the motors to drive");
      System.out.println(speed);
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
      radiusFRX = offsetX;
      radiusFRY = offsetY;
      radiusRRX = offsetX;
      radiusRRY = -offsetY;
      radiusRLX = -offsetX;
      radiusRLY = -offsetY;
      radiusFLX = -offsetX;
      radiusFLY = offsetY;

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

      //calculate the angle of each wheel, need to add an offset when x is negative because arctan only works on a range of pi
      angleFR = steerRatio * Math.atan(radiusFRY/radiusFRX) + angleOffset;
      angleRR = steerRatio * Math.atan(radiusRRY/radiusRRX) + angleOffset;
      angleRL = steerRatio * Math.atan(radiusRLY/radiusRLX) + angleOffset;
      angleFL = steerRatio * Math.atan(radiusFLY/radiusFLX) + angleOffset;

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

      /*
      speedFR = speed * Math.sqrt((radiusFRX * radiusFRX) + (radiusFRY * radiusFRY)) / radius;
      speedRR = speed * Math.sqrt((radiusRRX * radiusRRX) + (radiusRRY * radiusRRY)) / radius;
      speedRL = speed * Math.sqrt((radiusRLX * radiusRLX) + (radiusRLY * radiusRLY)) / radius;
      speedFL = speed * Math.sqrt((radiusFLX * radiusFLX) + (radiusFLY * radiusFLY)) / radius;

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);

      angleFR = steerRatio * Math.atan(radiusFRY/radiusFRX) + angleOffset;
      angleRR = steerRatio * Math.atan(radiusRRY/radiusRRX) + angleOffset;
      angleRL = steerRatio * Math.atan(radiusRLY/radiusRLX) + angleOffset;
      angleFL = steerRatio * Math.atan(radiusFLY/radiusFLX) + angleOffset;

      controllerFRSteer.setReference(angleFR + wheelOffsetFR, ControlType.kPosition);
      controllerRRSteer.setReference(angleRR + wheelOffsetRR, ControlType.kPosition);
      controllerRLSteer.setReference(angleRL + wheelOffsetRL, ControlType.kPosition);
      controllerFLSteer.setReference(angleFL + wheelOffsetFL, ControlType.kPosition);
      */
       

      speedFRX = (speed / radius) * -radiusFRY;
      speedRRX = (speed / radius) * -radiusRRY;
      speedRLX = (speed / radius) * -radiusRLY;
      speedFLX = (speed / radius) * -radiusFLY;

      speedFRY = (speed / radius) * radiusFRX;
      speedRRY = (speed / radius) * radiusRRX;
      speedRLY = (speed / radius) * radiusRLX;
      speedFLY = (speed / radius) * radiusFLX;

      speedFR = kDrive * Math.sqrt((speedFRX * speedFRX) + (speedFRY * speedFRY));
      speedRR = kDrive * Math.sqrt((speedRRX * speedRRX) + (speedRRY * speedRRY));
      speedRL = kDrive * Math.sqrt((speedRLX * speedRLX) + (speedRLY * speedRLY));
      speedFL = kDrive * Math.sqrt((speedFLX * speedFLX) + (speedFLY * speedFLY));

      angleFR = Math.atan(speedFRY / speedFRX) + angleOffset;
      angleRR = Math.atan(speedRRY / speedRRX) + angleOffset;
      angleRL = Math.atan(speedRLY / speedRLX) + angleOffset;
      angleFL = Math.atan(speedFLY / speedFLX) + angleOffset;

      controllerFRSteer.setReference(angleFR, ControlType.kPosition);
      controllerRRSteer.setReference(angleRR, ControlType.kPosition);
      controllerRLSteer.setReference(angleRL, ControlType.kPosition);
      controllerFLSteer.setReference(angleFL, ControlType.kPosition);

      controllerFRDrive.setReference(speedFR, ControlType.kVelocity);
      controllerRRDrive.setReference(speedRR, ControlType.kVelocity);
      controllerRLDrive.setReference(speedRL, ControlType.kVelocity);
      controllerFLDrive.setReference(speedFL, ControlType.kVelocity);
      
    System.out.println("waaabufffeeeeet its me toad, and im heer to tell you tha youre duuumb here is the values for your speedFR and angleFR IDIOT NERD BOZO kk byyyyeyeeee");
    System.out.println(speedFR);
    System.out.println(angleFR);
    

    
    } 
    if(joystick.getRawButtonPressed(2)){
      controllerFRSteer.setReference(0, ControlType.kPosition);
    }
    if(joystick.getRawButtonPressed(1)){
      controllerFRSteer.setReference(2 * Math.PI * steerRatio, ControlType.kPosition);
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

