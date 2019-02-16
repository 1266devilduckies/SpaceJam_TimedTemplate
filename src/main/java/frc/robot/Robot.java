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

  //Initialize drive motors and controllers
  private final Talon m_driveLeft = new Talon(0);
  private final Talon m_driveRight = new Talon(1);

  private final Joystick m_stick = new Joystick(0);
  private final Joystick m_stick2 = new Joystick(1);

  //Create drivetrain object from DifferentialDrive
  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_driveLeft, m_driveRight);

  private final TalonSRX m_elevator = new TalonSRX(0);
  private final DigitalInput m_magSwitch = new DigitalInput(0);
  private TalonSRXConfiguration m_config = new TalonSRXConfiguration();
  //private boolean reset = false;

  private int cycles = 0;

  private final Talon m_clawMotor = new Talon(2);
  private double m_maxExtraSpeed = 1;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    
    //jeremy big gay

    //change drive axes for forward-back to up-down on left joystick (1), and left-right to left-right on right joystick (2)
    m_stick.setXChannel(3);
    m_stick.setYChannel(1);

    m_elevator.setInverted(true);

    m_elevator.getAllConfigs(m_config);
    m_config.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    m_config.primaryPID.selectedFeedbackCoefficient = 1;
    m_elevator.setSensorPhase(true);
    m_config.forwardSoftLimitEnable = true;
    m_config.forwardSoftLimitThreshold = 27000;
    m_config.reverseSoftLimitEnable = true;
    m_config.reverseSoftLimitThreshold = 0;
    m_config.slot0.kP = 0;
    m_elevator.configAllSettings(m_config);
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
    double m_stickY = m_stick.getY()*-1;

    //drive with assigned joysticks
    m_driveTrain.tankDrive(m_stick.getY(), m_stick.getX());

    if(m_magSwitch.get()==false){
      m_elevator.setSelectedSensorPosition(0, 0, 100);
    }

    m_elevator.set(ControlMode.PercentOutput, m_stickY*0.5);

    if(SmartDashboard.getNumber("Maximum Motor Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=-1){
      m_maxExtraSpeed = SmartDashboard.getNumber("Maximum Motor Speed", 1);
    }

    double targetOutput = 0;

    if(m_stick.getRawButton(4)){
      targetOutput = m_maxExtraSpeed;
    }else if(m_stick.getRawButton(2)){
      targetOutput = m_maxExtraSpeed*-1;
    }else{
      targetOutput = 0;
    }

    if (m_clawMotor.getSpeed()>targetOutput){
      m_clawMotor.setSpeed(m_clawMotor.getSpeed() - 0.1);
    }else if (m_clawMotor.getSpeed()<targetOutput){
      m_clawMotor.setSpeed(m_clawMotor.getSpeed() + 0.1);
    }

    SmartDashboard.putNumber("Extra Motor Speed", m_clawMotor.getSpeed());
  }

  @Override
  public void testInit() {
    System.out.println(m_config);
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    double m_stickY = m_stick.getY()*-1;
    m_elevator.set(ControlMode.PercentOutput, m_stickY*0.5);
    
    
  }
}
