// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Useless Imports
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //Defines the different auto options
  private static final String kDefaultAuto = "Short Side Auto";
  private static final String kCustomAuto = "Score + Auto Balance";
  private static final String klipAuto = "Lip Side Auto";
  private static final String kscoreLipAuto = "Score + Lip Side Auto";
  private static final String kautoBalanceOnly = "Auto Balance";
  private static final String kscoreShortAuto = "Score + Short Side Auto";

  //Magic
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  // Define the four drive motors
  private final WPI_VictorSPX m_rearLeft = new WPI_VictorSPX(4);
  private final WPI_VictorSPX m_rearRight = new WPI_VictorSPX(1);
  private final WPI_VictorSPX m_frontLeft = new WPI_VictorSPX(2);
  private final WPI_VictorSPX m_frontRight = new WPI_VictorSPX(3);

  // Defines the IMU (Gyro)
  private final AHRS m_imu = new AHRS(I2C.Port.kOnboard);

  // Define the arm motor
  private final CANSparkMax m_armAngle = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_armExtend = new CANSparkMax(5, MotorType.kBrushless);

  // Groups the four motors into two groups
 private final MotorControllerGroup m_left = new MotorControllerGroup(m_rearLeft, m_frontLeft);
 private final MotorControllerGroup m_right = new MotorControllerGroup(m_rearRight, m_frontRight);

  // Defines the controller
 private final GenericHID m_controller = new GenericHID(1);
 private final GenericHID m_armController = new GenericHID(0);

 // Defines the PID functionality for SparkMax motor controllers
 private final SparkMaxPIDController m_pidController = m_armAngle.getPIDController();
 private final SparkMaxPIDController m_pidController2 = m_armExtend.getPIDController();

 // Defines an encoder
 private final RelativeEncoder m_encoder = m_armAngle.getEncoder();
 private final RelativeEncoder m_Encoder2 = m_armExtend.getEncoder();

 // Defines teh variables for the PID values for the first Spark Max
 public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

 // Defines the variables for the PID values for the second Spark Max
 public double kP2, kI2, kD2, kIz2, kFF2, kMaxOutput2, kMinOutput2, maxRPM2;

 // Defines the doubles for the request lights and arm positions, and teh boolean for the arm movement mode 
 public double level1, level2, level3;
 public boolean kyellowOn, kblueOn, goingLevel1, goingLevel2, goingLevel3;
 public boolean mode;

  // Defines the strings for teh speed mode and the message to Vick
 public String kspeedMode, kflashingWarning;

  // Defines the double for the speed multiplier
 public double kspeedMultiply;

  // Defines the compressor
 public final Compressor m_Compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  // Defines the solenoids for the grabber
 private final Solenoid m_solenoidExtend = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
 private final Solenoid m_solenoidDetract = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  // Defines the toggles for the request lights
 private final Solenoid m_blueLights = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
 private final Solenoid m_yellowLights = new Solenoid(PneumaticsModuleType.CTREPCM, 4);

  // Puts the two motor groups into a differential drive system
 private final DifferentialDrive m_drive = new DifferentialDrive(m_left, m_right);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
  // Defines the options for the auto-chooser
    m_chooser.setDefaultOption("Short Side Auto", kDefaultAuto);
    m_chooser.addOption("Score + Auto Balance", kCustomAuto);
    m_chooser.addOption("Lip Side Auto", klipAuto);
    m_chooser.addOption("Auto Balance", kautoBalanceOnly);
    m_chooser.addOption("Score + Lip Side Auto", kscoreLipAuto);
    m_chooser.addOption("Score + Short Side Auto", kscoreShortAuto);

  // Sets the message to Vick
    kflashingWarning = "There's a camera to help you!";

  // Magic
    SmartDashboard.putData("Auto choices", m_chooser);
    m_armAngle.restoreFactoryDefaults();

  // Starts the camera stream
    CameraServer.startAutomaticCapture();

  // Calibrates arm positions
    armPositions();

    // PID coefficients
    kP = 8e-5; 
    kI = 0;
    kD = 0;
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
    maxVel = 2000;
    maxAcc = 1500;
    mode = false;

    /**
    Default PID values
    kP = 6e-5; 
    kI = 0;
    kD = 0;
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;
     */

  // PID Coefficients 2
    kP2 = 6e-5; 
    kI2 = 0;
    kD2 = 0; 
    kIz2 = 0; 
    kFF2 = 0.000015; 
    kMaxOutput2 = 1; 
    kMinOutput2 = -1;
    maxRPM2 = 5700;

  // Sets PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

  // Sets smart PID coefficients
    int smartMotionSlot = 0;
    m_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    m_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    m_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    m_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);


  // Sets PID coefficients 2
    m_pidController2.setP(kP2);
    m_pidController2.setI(kI2);
    m_pidController2.setD(kD2);
    m_pidController2.setIZone(kIz2);
    m_pidController2.setFF(kFF2);
    m_pidController2.setOutputRange(kMinOutput2, kMaxOutput2);



  // Displays PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Max Velocity", maxVel);
    SmartDashboard.putNumber("Min Velocity", minVel);
    SmartDashboard.putNumber("Max Acceleration", maxAcc);
    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);
    SmartDashboard.putNumber("Set Position", 15);
    SmartDashboard.putNumber("Set Velocity", 0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */

  // Sets variables used in auto
int step = 0;
double autoStartTime = 0;

  // Defines the timers used in auto
Timer autoTimer = new Timer();
Timer autoTimer1 = new Timer();
Timer autoTimer2 = new Timer();
Timer autoTimer3 = new Timer();
Timer autoTimer4 = new Timer();
  // Magic
  double targetArmPos = 0;

  @Override
  public void autonomousInit() {
  // Magic
    m_autoSelected = m_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected);
  
  // Enables the compressor
    m_Compressor.enableDigital();

  // Closes the grabber
    m_solenoidDetract.set(true);
    m_solenoidExtend.set(false);

  // Sets step to 0
    step = 0;

  // Magic
    targetArmPos = 0;

  // Resets the timers
    autoTimer.reset();
    autoTimer1.reset();
    autoTimer2.reset();
    autoTimer3.reset();
    autoTimer4.reset();
    autoTimer.restart();

  // Sets the FF PID value
    m_pidController.setFF(0.000156);

  // Inverts left motors
    m_rearLeft.setInverted(true);
    m_frontLeft.setInverted(true);

  // Makes sure the right motors don't invert
    m_rearRight.setInverted(false);
    m_frontRight.setInverted(false);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  // Displays useful values on the SmartDashboard
    SmartDashboard.putNumber("Gyro Y Axis", m_imu.getRoll());
    SmartDashboard.putNumber("Position", m_encoder.getPosition());
    SmartDashboard.putNumber("step: ", step);
    SmartDashboard.putNumber("Timer", autoTimer3.get());

  // The big auto-switch statement
    switch (m_autoSelected) {
      case kCustomAuto:
      // Score and Autobalance
      // This auto shouldn't be run, there isn't enough time to both score and auto-balance. For this reason, I won't comment it. Most code is re-used elsewhere.
      if (step == 0){
        m_pidController.setReference(level2-10, CANSparkMax.ControlType.kSmartMotion);
        if(m_encoder.getPosition()<=level2-8){
          autoTimer1.restart();
          step = 1;
        }
      }

      if(step == 1){
        m_drive.tankDrive(0.6, 0.6);
        if(autoTimer1.get() > 0.5){
          autoTimer2.restart();
          step = 2;
        }
      }

      if(step == 2){
        m_solenoidExtend.toggle();
        m_solenoidDetract.toggle();
        step = 3;
      }

      if(step == 3){
        if(autoTimer2.get()>0.5){
          m_solenoidDetract.toggle();
          m_solenoidExtend.toggle();
          autoTimer3.restart();
          step = 4;
          }
      }

      if(step == 4){
        m_drive.tankDrive(-0.5, -0.5);
        if(autoTimer3.get() > 0.25){
          autoTimer4.restart();
          step = 5;
        }
      }

      if (step == 5){
        m_drive.tankDrive(-0.7, -0.7);
        m_pidController.setReference(level1, CANSparkMax.ControlType.kSmartMotion);
        if (Math.abs(m_imu.getRoll() ) > 14 || autoTimer4.get() > 5 ){
          step = 6;
        }
      }

      if (step == 6){
        autoBalance();
      }
      
        break;
      case kDefaultAuto:
      // Short side auto (Leave community on the short side)
      
      if (step == 0) { // Step 1
      // Drive forward at 50% speed
        m_drive.tankDrive(0.5, 0.5);
      }

    // Wait for 5.5 seconds
      if (autoTimer.get() > 5.5) {
      // Move step up one
        step++;
      }
      // Break this case
        break;
      case klipAuto:
      // Lip Side Auto (Leave community on the lip side)
      
      if (step == 0) { // Step 1
      // Drive forward at 60% speed
        m_drive.tankDrive(0.6, 0.6);
      }

    // Wait for 3.75 seconds
      if (autoTimer.get() > 3.75) {
      // Move step up one
        step++;
      }

    // Break this case
      break;
      case kautoBalanceOnly:
      // Autobalance (Only Autobalances)
    
    if (step == 0){ // Step 1
    // Drive backwards at 70% speed
      m_drive.tankDrive(-0.7, -0.7);

    // If angle becomes 15 or higher, or 5 seconds have elapsed...
      if (Math.abs(m_imu.getRoll() ) > 15 || autoTimer.get() > 5){
      // Move step up one
        step++;
      }
    }

    if (step == 1){ // Step 2
    // Mason's Auto-Balance Function
      autoBalance();
    }
    // Break this case
      break;
    case kscoreLipAuto:
    // Score + Lip Side Auto (Scores cube in middle then leaves community on lip side)

    if (step == 0){ // Step 1
    // Sets desired position for arm to move to
      m_pidController.setReference(level2-10, CANSparkMax.ControlType.kSmartMotion);

    // Once 2 before desired position...
      if(m_encoder.getPosition()<=level2-8){
      // Start second timer
        autoTimer1.restart();

      // Set step to 1
        step = 1;
      }
    }

    if(step == 1){ // Step 2
    // Move forward at 45% speed
      m_drive.tankDrive(0.45, 0.45);
    // Wait for 1 second
      if(autoTimer1.get() > 1){
      // Start third timer
        autoTimer2.restart();

      // Set step to 2
        step = 2;
      }
    }

    if(step == 2){ // Step 3
    // Open claw
      m_solenoidExtend.toggle();
      m_solenoidDetract.toggle();

    // Set step to 3
      step = 3;
    }

    if(step == 3){ // Step 4
    // Wait for 1 second
      if(autoTimer2.get()>1){
      // Start fourth timer
        autoTimer3.restart();

      // Set step to 4
        step = 4;
        }
    }

    if(step == 4){ // Step 5
    // Drive backwards at 60% speed
      m_drive.tankDrive(-0.6, -0.6);

    // Wait 3 seconds
      if(autoTimer3.get()==3){
      // Set step to 5
        step = 5;
      }

    if(step==5){ // Step 6
    // Stop motors (Maybe)
      m_drive.tankDrive(0, 0);
    }
    }
  // Break this case
    break;
    case kscoreShortAuto:
    // Score + Short Side Auto (Score a cube at middle level and leave community on short side)
    if (step == 0){ // Step 1
    // Sets desired position for arm to move to
      m_pidController.setReference(level2-10, CANSparkMax.ControlType.kSmartMotion);

    // Once 2 before desired position...
      if(m_encoder.getPosition()<=level2-5){
      // Start second timer
        autoTimer1.restart();

      // Set step to 1
        step = 1;
      }
    }

    if(step == 1){ // Step 2
    // Move forward at 45% speed
    m_drive.tankDrive(0.45, 0.45);
    // Wait for 1 second
      if(autoTimer1.get() > 1){
      // Start third timer
        autoTimer2.restart();

      // Set step to 2
        step = 2;
      }
    }

    if(step == 2){ // Step 3
      // Open claw
        m_solenoidExtend.toggle();
        m_solenoidDetract.toggle();
  
      // Set step to 3
        step = 3;
      }

    if(step == 3){ // Step 4
     // Wait for 1 second
      if(autoTimer2.get()>1){
      // Start fourth timer
        autoTimer3.restart();
    
      // Set step to 4
        step = 4;
      }
    }

    if(step == 4){ // Step 5
    // Drive backwards at 60% speed
      m_drive.tankDrive(-0.5, -0.5);

    // Wait 3 seconds
      if(autoTimer3.get()==3){
      // Set step to 5
        step = 5;
      }

    if(step==5){ // Step 6
    // Stop motors (Maybe)
      m_drive.tankDrive(0, 0);
    }
    }
  // Breaks this case
    break;
      default:
      // The "I Don't Want the Switch Statement to Break" auto

      // Breaks this case
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // Inverts the left side motors
    m_rearLeft.setInverted(true);
    m_frontLeft.setInverted(true);

    m_rearRight.setInverted(false);
    m_frontRight.setInverted(false);


    m_Compressor.enableDigital();
    m_solenoidDetract.set(true);
    m_solenoidExtend.set(false);
    m_blueLights.set(false);
    m_yellowLights.set(false);
    kspeedMode = "Medium";
    kspeedMultiply = -0.6;
    kyellowOn = false;
    kblueOn = false;
    mode = true;
    goingLevel1 = false; 
    goingLevel2 = false;
    goingLevel3 = false;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    pidValueUpdate();
    double setPoint, ProcessVariable;
    setPoint = 0;
    ProcessVariable = 0;
    if (mode == true) {
      setPoint = ((m_armController.getRawAxis(1)*maxRPM)*1.1);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
      ProcessVariable = m_encoder.getVelocity();
    } else if(m_armController.getRawButtonReleased(6)) {
      setPoint = SmartDashboard.getNumber("Set Position", 15);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);

    }

    if (m_armController.getRawButtonReleased(2)) {
      modeSwitch();
    }

    if (m_controller.getRawButtonReleased(8)) {
      armPositions();
    }

    if(m_armController.getRawButtonReleased(4)) {
      SmartDashboard.putNumber("Set Position", level3);
      modeSwitch();
      setPoint = SmartDashboard.getNumber("Set Position", level3);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      goingLevel3 = true;
    }
    if (goingLevel3==true&&m_encoder.getPosition() <= (level3+1)){
      modeSwitch();
      goingLevel3 = false;
    }

    if (m_armController.getRawButtonReleased(3)) {
      SmartDashboard.putNumber("Set Position", level2);
      modeSwitch();
      setPoint = SmartDashboard.getNumber("Set Position", level2);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      goingLevel2 = true;
    }
    if (goingLevel2==true&&m_encoder.getPosition() <= (level2+1)){
      modeSwitch();
      goingLevel2 = false;
    }

    if (m_armController.getRawButtonReleased(5)){
      SmartDashboard.putNumber("Set Position", level1);
      modeSwitch();
      setPoint = SmartDashboard.getNumber("Set Position", level1);
      m_pidController.setReference(setPoint, CANSparkMax.ControlType.kSmartMotion);
      goingLevel1 = true;
    }
    if (goingLevel1==true&&m_encoder.getPosition() <= (level1+1)){
      modeSwitch();
      goingLevel1 = false;
    }

    // Defines two vaibles used for robot movement
  double rightSpeed = kspeedMultiply*m_controller.getRawAxis(1);
  double leftSpeed = kspeedMultiply*m_controller.getRawAxis(5);

    m_drive.tankDrive(leftSpeed, rightSpeed);

    double setPoint2 = ((m_armController.getRawAxis(0)*maxRPM));
    m_pidController2.setReference(setPoint2, CANSparkMax.ControlType.kVelocity);

    if (m_armController.getRawButtonReleased(1)) {
      m_solenoidDetract.toggle();
      m_solenoidExtend.toggle();
    }

    if (m_controller.getRawButtonReleased(3)) {
      m_blueLights.toggle();
    }

    if (m_controller.getRawButtonReleased(4)) {
      m_yellowLights.toggle();
    }

    if (m_yellowLights.get()) {
      kyellowOn = true;
    } else {
      kyellowOn = false;
    }

    if (m_blueLights.get()) {
      kblueOn = true;
    } else {
      kblueOn = false;
    }

    if (m_controller.getRawButtonReleased(6)) {
      switch(kspeedMode) {
        case "Medium":
          kspeedMode = "Fast";
          kspeedMultiply = -0.9;
          break;
        case "Fast":
          kspeedMode = "Medium";
          kspeedMultiply = -0.6;
          break;
        default:
          kspeedMode = "Medium";
          kspeedMultiply = -0.6;
      }
    }

    if (m_controller.getRawButtonReleased(5)) {
      switch(kspeedMode) {
        case "Medium":
          kspeedMode = "Fast";
          kspeedMultiply = -0.9;
          break;
        case "Fast":
          kspeedMode = "Medium";
          kspeedMultiply = -0.6;
          break;
        default:
          kspeedMode = "Medium";
          kspeedMultiply = -0.6;
      }
    }

    if (m_armController.getRawButtonPressed(3)); {
      //m_pidController.setReference(-10, CANSparkMax.ControlType.kPosition);
    }

    if (m_controller.getRawAxis(3) > 0) {
      m_drive.tankDrive(-kspeedMultiply*m_controller.getRawAxis(3), -kspeedMultiply*m_controller.getRawAxis(3));
    }

    if (m_controller.getRawAxis(2) > 0) {
      m_drive.tankDrive(kspeedMultiply*m_controller.getRawAxis(2), kspeedMultiply*m_controller.getRawAxis(2));
    }
    
    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("ProcessVariable", ProcessVariable);
    SmartDashboard.putNumber("Output", m_armAngle.getAppliedOutput());
    SmartDashboard.putString("Speed Mode", kspeedMode);
    SmartDashboard.putBoolean("Blue Lights On?", kblueOn);
    SmartDashboard.putBoolean("Yellow Lights On?", kyellowOn);
    SmartDashboard.putNumber("Gyro Y Axis", m_imu.getRoll());
    SmartDashboard.putString("To Vick:", kflashingWarning);
    SmartDashboard.putNumber("Position", m_encoder.getPosition());
    SmartDashboard.putBoolean("Mode", mode);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putBoolean("a", goingLevel1);
    SmartDashboard.putBoolean("aa", goingLevel2);
    SmartDashboard.putBoolean("aaa", goingLevel3);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_Compressor.disable();
    kFF = 0.000015;
    m_pidController.setFF(0.000015);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("Feed Forward", kFF);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_Compressor.enableDigital();
    m_solenoidDetract.set(true);
    m_solenoidExtend.set(false);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    SmartDashboard.putNumber("Gyro Y Axis", m_imu.getRoll());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  static public final double map(double value, 
  double istart, 
  double istop, 
  double ostart, 
  double ostop) {
  return ostart + (ostop - ostart) * ((value - istart) / (istop - istart));
  }

  public void autoBalance(){
    double maxSpeed = 0.5;
    double minSpeed = 0.15;
    double fixedIMU = fixGyro(m_imu.getRoll());

    double outPut = map(Math.abs(fixedIMU),2,14,minSpeed,maxSpeed);

    if (outPut < minSpeed) outPut = 0;
    if (outPut > maxSpeed) outPut = maxSpeed;

    outPut *= Math.signum( -fixGyro( m_imu.getRoll() ));

    

    m_drive.tankDrive(outPut * 1.06, outPut);
  }


  public double fixGyro(double input){
    if (input > 180) return input - 360;
    return input;
  }

  public void pidValueUpdate(){
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    double maxV = SmartDashboard.getNumber("Max Velocity", 0);
    double minV = SmartDashboard.getNumber("Min Velocity", 0);
    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);
    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    if((maxV != maxVel)) { m_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    if((minV != minVel)) { m_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    if((maxA != maxAcc)) { m_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    if((allE != allowedErr)) { m_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
  }

  public void modeSwitch(){
    if(mode == true){
      kFF = 0.000156;
    } else {
      kFF = 0.000015;
    }
    m_pidController.setFF(kFF);
    SmartDashboard.putNumber("Feed Forward", kFF);
    mode = !mode;
  }

  public void armPositions(){
    level1 = m_encoder.getPosition();
    level2 = level1 - 140;
    level3 = level1 - 185;
  }
}