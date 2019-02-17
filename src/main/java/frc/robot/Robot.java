/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private Timer m_timer = new Timer();

  //Initialize drive motors
  private final Talon m_driveLeft = new Talon(0);
  private final Talon m_driveRight = new Talon(1);

  //Initialize intake motors
  private final Talon m_ballInMotor = new Talon(2);
  private final Talon m_hatchInMotor = new Talon(3);

  //prepare intake max speed
  private double m_maxIntakeSpeed = 1;

  //Initialize controllers
  private final Joystick m_driveStick = new Joystick(0);
  private final Joystick m_opStick = new Joystick(1);

  //Create drivetrain object from DifferentialDrive
  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_driveLeft, m_driveRight);

  //Create TalonSRX object for elevator, magnetic limit switch for reverse limit on elevator, and configuration object so we can assign same config to any SRX
  private final TalonSRX m_elevator = new TalonSRX(0);
  private final DigitalInput m_magSwitch = new DigitalInput(0);
  private TalonSRXConfiguration m_eleConfig = new TalonSRXConfiguration();
  private int m_elevPos = 0;

  //Same as elevator initialization, but for the pivot
  private final TalonSRX m_pivot = new TalonSRX(2);
  //add limit switch for pivot here
  private TalonSRXConfiguration m_pivotConfig = new TalonSRXConfiguration();
  private boolean m_pivotFolded = true;
  private int m_pivotPos = 0;


  private int cycles = 0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    //Create auto selector
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    //jeremy big gay

    //change drive axes for forward-back to up-down on left joystick (1), and left-right to left-right on right joystick (2)
    m_driveStick.setXChannel(3);
    m_driveStick.setYChannel(1);

    //elevator motor configuration
    m_elevator.setInverted(true);
    m_elevator.getAllConfigs(m_eleConfig);
    m_eleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    m_eleConfig.primaryPID.selectedFeedbackCoefficient = 1;
    m_elevator.setSensorPhase(true);
    m_eleConfig.forwardSoftLimitEnable = true;
    m_eleConfig.forwardSoftLimitThreshold = 25000;
    m_eleConfig.reverseSoftLimitEnable = true;
    m_eleConfig.reverseSoftLimitThreshold = 0;
    m_eleConfig.slot0.kP = 0.25;
    m_eleConfig.slot0.kI = 0;
    m_elevator.configAllSettings(m_eleConfig);

    //pivot motor configuration
    m_pivot.setInverted(true);
    m_pivot.getAllConfigs(m_pivotConfig);
    m_pivotConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    m_pivotConfig.primaryPID.selectedFeedbackCoefficient = 1;
    m_pivot.setSensorPhase(true);
    m_pivotConfig.forwardSoftLimitEnable = true;
    m_pivotConfig.forwardSoftLimitThreshold = 2500;
    m_pivotConfig.reverseSoftLimitEnable = true;
    m_pivotConfig.reverseSoftLimitThreshold = 0;
    m_pivotConfig.slot0.kP = 0.25;
    m_pivotConfig.slot0.kI = 0;
    m_pivot.configAllSettings(m_pivotConfig);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
    cycles = 0;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        // moves drive train at full speed forward and back for 20 minutes to test gear wear or something, changes direction every 5 minutes
        if(cycles<2){
          if(m_timer.get()<300){
            m_driveTrain.tankDrive(1, 1);
          }else if(m_timer.get()<600){
            m_driveTrain.tankDrive(-1, -1);
          }else{
            cycles++;
            m_timer.reset();
          }
        }else{
          m_driveTrain.tankDrive(0, 0);
        }
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    //invert y axis (default controller backwards)
    double m_stickY = m_driveStick.getY()*-1;

    //drive with assigned joysticks
    m_driveTrain.tankDrive(m_stickY, m_driveStick.getX()*-1);

    //move elevator at half speed of Y axis
    m_elevator.set(ControlMode.PercentOutput, m_stickY*0.5);    

    //Use smartdashboard to set max speed of claw motor
    if(SmartDashboard.getNumber("Maximum Motor Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=-1){
      m_maxIntakeSpeed = SmartDashboard.getNumber("Maximum Motor Speed", 1);
    }

    //if cross is pressed
    if(m_driveStick.getRawButton(2)){
      //set to low position
      m_elevPos = 0;
    //else if square is pressed
    }else if(m_driveStick.getRawButton(1)){
      //set to middle position
      m_elevPos = 13000;
    //else if triangle is pressed
    }else if(m_driveStick.getRawButton(4)){
      //set to highest position
      m_elevPos = 25000;
    //else if circle is pressed
    }else if(m_driveStick.getRawButtonPressed(3)){
      //and the pivot motor is folded in
      if(m_pivotFolded==true){
        //fold out
        m_pivot.set(ControlMode.Position, 1000);
      }else{
        //fold in
        m_pivot.set(ControlMode.Position, 0);
      }
      //reverse status
      m_pivotFolded = !m_pivotFolded;
    }

    m_elevator.set(ControlMode.Position, m_elevPos);

    //when the magnetic sensor is triggered (i.e. elevator is at the bottom)
    if(m_magSwitch.get()==false){
      //reset encoder to 0
      m_elevator.setSelectedSensorPosition(0, 0, 100);
    }

    //when the magnetic sensor is triggered (i.e. pivot is fully folded)
    /*if(m_magSwitch.get()==false){
      //reset encoder to 0
      m_pivot.setSelectedSensorPosition(0, 0, 100);
    }*/

    double ballInOutput = 0;

    if(m_opStick.getRawButton(5)){
      ballInOutput = m_maxIntakeSpeed;
    }else if(m_opStick.getRawButton(7)){
      ballInOutput = -m_maxIntakeSpeed;
    }else{
      ballInOutput = 0;
    }

    if (m_ballInMotor.getSpeed()>ballInOutput){
      m_ballInMotor.setSpeed(m_ballInMotor.getSpeed() - 0.1);
    }else if (m_ballInMotor.getSpeed()<ballInOutput){
      m_ballInMotor.setSpeed(m_ballInMotor.getSpeed() + 0.1);
    }

    double hatchInOutput = 0;

    if(m_driveStick.getRawButton(8) || m_opStick.getRawButton(6)){
      hatchInOutput = m_maxIntakeSpeed;
    }else if(m_driveStick.getRawButton(7) || m_opStick.getRawButton(8)){
      hatchInOutput = -m_maxIntakeSpeed;
    }else{
      hatchInOutput = 0;
    }

    if (m_hatchInMotor.getSpeed()>hatchInOutput){
      m_hatchInMotor.setSpeed(m_hatchInMotor.getSpeed() - 0.1);
    }else if (m_hatchInMotor.getSpeed()<hatchInOutput){
      m_hatchInMotor.setSpeed(m_hatchInMotor.getSpeed() + 0.1);
    }

    SmartDashboard.putNumber("Extra Motor Speed", m_hatchInMotor.getSpeed());
  }

  @Override
  public void testInit() {
    System.out.println(m_pivotConfig);
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double m_stickY = m_driveStick.getY()*-1;

    if(m_driveStick.getRawButton(6)){
      //if cross
      if(m_driveStick.getRawButton(2)){
        //set to in position
        m_elevPos = 0;
      //else if square is pressed
      }else if(m_driveStick.getRawButton(1)){
        //set to out position
        m_elevPos = 13000;
      //if triangle
      }else if(m_driveStick.getRawButton(4)){
        //set to out position
        m_pivotPos = 24900;
      }
    }else{
      //if cross
      if(m_driveStick.getRawButton(2)){
        //set to in position
        m_pivotPos = 0;
      //else if square is pressed
      }else if(m_driveStick.getRawButton(1)){
        //set to out position
        m_pivotPos = 1500;
      //if triangle
      }else if(m_driveStick.getRawButton(4)){
        //set to out position
        m_pivotPos = 2500;
      }
    }
    
    m_pivot.set(ControlMode.Position, m_pivotPos);
    m_elevator.set(ControlMode.Position, m_elevPos);
    
    //when the magnetic sensor is triggered (i.e. elevator is at the bottom)
    if(m_magSwitch.get()==false){
      //reset encoder to 0
      m_elevator.setSelectedSensorPosition(0, 0, 100);
    }
    
    m_driveTrain.tankDrive(m_stickY, m_driveStick.getX()*-1);

  }
}
