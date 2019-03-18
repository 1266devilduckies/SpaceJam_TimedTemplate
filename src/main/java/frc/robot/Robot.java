/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.github.cliftonlabs.json_simple.*;

import java.math.BigDecimal;

import edu.wpi.first.cameraserver.CameraServer;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.VideoMode;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;

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
  private double drive_kP = 0;
  private double drive_kI = 0;
  private double drive_kF = 0;
  private double drive_integral = 0;

  //Initialize intake motors
  private final Talon m_ballInMotor = new Talon(2);
  private final Talon m_hatchInMotor = new Talon(3);

  //prepare intake max speed
  private double m_maxIntakeSpeed = 1;

  //Initialize controllers
  private final Joystick m_driveStick = new Joystick(0);
  private double m_stickyY = 0;
  private double m_stickyX = 0;
  private double stickAngle = 0;
  private final Joystick m_opStick = new Joystick(1);

  //Create drivetrain object from DifferentialDrive
  private final DifferentialDrive m_driveTrain = new DifferentialDrive(m_driveLeft, m_driveRight);
  private double m_maxSpeed = 1;

  //Create TalonSRX object for elevator, magnetic limit switch for reverse limit on elevator, and configuration object so we can assign same config to any SRX
  private final TalonSRX m_elevator = new TalonSRX(0);
  private final DigitalInput m_elevSwitch = new DigitalInput(0);
  private TalonSRXConfiguration m_eleConfig = new TalonSRXConfiguration();
  private int m_elevPos = 0;
  private int m_elevLimit = 27000;
  private final double elevator_kF = 0;
  private int m_stickyElevPos = 0;
  private int m_elevAddition = 0;

  //Same as elevator initialization, but for the pivot
  private final TalonSRX m_pivot = new TalonSRX(2);
  private DigitalInput m_pivotSwitch = new DigitalInput(1);
  private TalonSRXConfiguration m_pivotConfig = new TalonSRXConfiguration();
  private int m_pivotPos = 0;
  private int m_pivotLimit = 3200;
  private double pivot_kF = 0;
  private int m_pivotAddition = 0;
  private int MIN_PIVOT = 700;
  private int HATCH_PIVOT = 1500;

  //Photoelectric sensor initialization
  private final DigitalInput m_photoElec = new DigitalInput(2);

  //Gyroscope initialization
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  private int turnCount = 0;

  //create serialport object for comm with jevois
  SerialPort usbSerial = null;
  private JsonArray jevoisArray = null;

  //create solenoid object for hatchSnatcher
  private DoubleSolenoid m_hatchSnatcher = new DoubleSolenoid(0,1);

  //Create necessary control variables
  private int mode = 0;
  private boolean visionToggle = false;
  private boolean pivotOut = false;
  private int cycles = 0;
  private boolean autoRan = false;

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
    
    //jeremy big dumb

    //change drive axes for forward-back to up-down on left joystick (1), and left-right to left-right on right joystick (2)
    m_driveStick.setXChannel(2);
    m_driveStick.setYChannel(3);

    //elevator motor configuration
    m_elevator.setInverted(true);
    m_elevator.getAllConfigs(m_eleConfig);
    m_eleConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    m_eleConfig.primaryPID.selectedFeedbackCoefficient = 1;
    m_elevator.setSensorPhase(true);
    m_eleConfig.forwardSoftLimitEnable = true;
    m_eleConfig.forwardSoftLimitThreshold = m_elevLimit;
    m_eleConfig.reverseSoftLimitEnable = true;
    m_eleConfig.reverseSoftLimitThreshold = 0;
    m_eleConfig.slot0.kP = 0.35;
    m_eleConfig.slot0.kI = 0;
    m_elevator.configAllSettings(m_eleConfig);

    //pivot motor configuration
    m_pivot.setInverted(true);
    m_pivot.getAllConfigs(m_pivotConfig);
    m_pivotConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.QuadEncoder;
    m_pivotConfig.primaryPID.selectedFeedbackCoefficient = 1;
    m_pivot.setSensorPhase(true);
    m_pivotConfig.forwardSoftLimitEnable = true;
    m_pivotConfig.forwardSoftLimitThreshold = m_pivotLimit;
    m_pivotConfig.reverseSoftLimitEnable = true;
    m_pivotConfig.reverseSoftLimitThreshold = -700;
    m_pivotConfig.slot0.kP = 0.35;
    m_pivotConfig.slot0.kI = 0;
    m_pivot.configAllSettings(m_pivotConfig);

    SmartDashboard.putNumber("Maximum Drive Speed", 1);
    SmartDashboard.putNumber("Maximum Intake Speed", 1);
    SmartDashboard.putNumber("drive_kP", drive_kP);
    SmartDashboard.putNumber("drive_kI", drive_kI);
    SmartDashboard.putNumber("drive_kF", drive_kF);

    m_gyro.reset();
    m_gyro.calibrate();

    try {
			System.out.print("Creating JeVois SerialPort...");
			usbSerial = new SerialPort(115200,SerialPort.Port.kUSB1);
			System.out.println("SUCCESS!!");
		} catch (Exception e) {
			System.out.println("FAILED!!  Fix and then restart code...");
                        e.printStackTrace();
    }

    if(usbSerial == null){
      SmartDashboard.putBoolean("Jevois Connected?", false);
    }else{
      SmartDashboard.putBoolean("Jevois Connected?", true);
    }

    new Thread(() -> {
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setVideoMode(VideoMode.PixelFormat.kYUYV, 320, 240, 15);
      
      //lmao this dont work
      Mat sourceImg = new Mat();
      Mat output = new Mat();

      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 320, 240);
      
      while(!Thread.interrupted()) {
          cvSink.grabFrame(sourceImg);
          Imgproc.cvtColor(sourceImg, output, Imgproc.COLOR_BGR2GRAY);
          outputStream.putFrame(output);
      }
    }).start();
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

    while(m_elevSwitch.get() == true && m_timer.get() < 1){
      m_elevator.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, elevator_kF);
    }

    /*while(m_pivotSwitch.get() == true){
      m_pivot.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, elevator_kF);
    }*/
    
    m_pivot.setSelectedSensorPosition(-800);
    m_pivotPos=0;
    m_elevPos=0;
    m_elevator.setSelectedSensorPosition(0);
    mode = 0;

    autoRan = true;
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
        teleopPeriodic();
        break;
    }
  }

  @Override
  public void teleopInit() {
    if(autoRan == false){
      while(m_elevSwitch.get() == true && m_timer.get() < 1){
        m_elevator.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, elevator_kF);
      }

      /*while(m_pivotSwitch.get() == true){
        m_pivot.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, elevator_kF);
      }*/
      
      m_pivot.setSelectedSensorPosition(-800);
      m_pivotPos=0;
      m_elevPos=0;
      m_elevator.setSelectedSensorPosition(0);
      mode = 0;
    }
    autoRan = false;
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    drive_kP = SmartDashboard.getNumber("drive_kP", 0);
    drive_kI = SmartDashboard.getNumber("drive_kI", 0);
    drive_kF = SmartDashboard.getNumber("drive_kF", 0);

    //double pivotAngle = m_pivot.getSelectedSensorPosition() * 0.08789 + 40;
    //pivot_kF = 0.87716*Math.cos(pivotAngle);
    //pivot_kF = 0.1*Math.cos(pivotAngle);
    //elevator_kF = 0.46845;

    double gyro = 0;
    double error = 0;
    double turn_power = 0;

    if(m_driveStick.getRawButtonPressed(13)){
      m_gyro.reset();
    }

    if(m_driveStick.getRawButtonPressed(8)==true){
      visionToggle = !visionToggle;
    }

    if(m_elevator.getSelectedSensorPosition() > 1000){
      visionToggle = false;
    }

    if(usbSerial != null && usbSerial.getBytesReceived()>0){
      jevoisArray = Jsoner.deserialize(usbSerial.readString(), new JsonArray());
      SmartDashboard.putBoolean("Sees Target?", !jevoisArray.isEmpty());
    }else{
      SmartDashboard.putBoolean("Sees Target?", false);
    }

    double centerY = 0;
    double dist = -10000;

    if(usbSerial != null && visionToggle == true){
      if(usbSerial.getBytesReceived()>0){
        System.out.println(jevoisArray);
        if(jevoisArray.isEmpty()==false){
          double centerX = jevoisArray.getDouble(0);
          centerY = jevoisArray.getBigDecimal(1).doubleValue();
          double between = jevoisArray.getDouble(2);
          double width = (centerX*11.5)/between;
          if(m_photoElec.get() == true){
            dist = (0.008*Math.pow(centerY, 2))+(0.1828*centerY)+10.225;
          }else{
          }
          error = Math.atan(width/dist);
          //gyro = m_gyro.getAngle();
        }
      }
    }else{
      gyro = m_gyro.getAngle() - (360*turnCount);

      if(gyro>180){
        turnCount++;
      }else if (gyro<=-180){
        turnCount--;
      }

      gyro = m_gyro.getAngle() - (360*turnCount);
      
      if (Math.abs( m_driveStick.getX() ) > 0.5 || Math.abs( m_driveStick.getY() ) > 0.5){
        stickAngle = m_driveStick.getDirectionDegrees();
      }

      error = stickAngle - gyro;

      SmartDashboard.putNumber("Stick Error", m_driveStick.getDirectionDegrees());
    }

    if(error > 180) {
      error-=360;
    }else if (error<-180){
      error+=360;
    }
    
    drive_integral += (error*.02);

    if(error != 0){
      turn_power = (drive_kP * error) + (drive_kI*drive_integral) + drive_kF;
    }

    if(turn_power > m_maxSpeed){
      turn_power = m_maxSpeed;
    }else if (turn_power < m_maxSpeed*-1){
      turn_power = m_maxSpeed*-1;
    }

    //m_robotDrive.arcadeDrive(m_stick.getY()*m_maxSpeed*-1, m_stick.getX()*m_maxSpeed);
    //m_robotDrive.arcadeDrive(m_stick.getRawAxis(1)*m_maxSpeed*-1,turn_power,false);

    SmartDashboard.putBoolean("Vision Toggled On?", visionToggle);
    SmartDashboard.putNumber("Gyro Angle", gyro);
    SmartDashboard.putBoolean("Gyro Connected", m_gyro.isConnected());
    SmartDashboard.putNumber("Error", error);
    SmartDashboard.putNumber("Turn Power", turn_power);
  
    if(SmartDashboard.getNumber("Maximum Drive Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=0){
      m_maxSpeed = SmartDashboard.getNumber("Maximum Drive Speed", 1);
    }

    //invert y axis and add ramp (default controller backwards)
    double targetY = Math.round(m_driveStick.getRawAxis(1)*-10.0)/10.0;

    if(m_stickyY>targetY){
      m_stickyY -= 0.1;
    }else if (m_stickyY<targetY){
      m_stickyY += 0.1;
    }

    SmartDashboard.putNumber("stickyY", m_stickyY);
    SmartDashboard.putNumber("y",targetY);

    //add ramp to x
    //double targetX = Math.round(turn_power*100.0)/100.0;
    double targetX = Math.round(m_driveStick.getX()*100.0)/100.0;

    if(m_stickyX>targetX){
      m_stickyX -= 0.1;
    }else if (m_stickyX<targetX){
      m_stickyX -= 0.1;
    }

    //drive with assigned joysticks
    m_driveTrain.arcadeDrive(m_stickyY, targetX);  

    //Use smartdashboard to set max speed of claw motor
    if(SmartDashboard.getNumber("Maximum Motor Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=-1){
      m_maxIntakeSpeed = SmartDashboard.getNumber("Maximum Motor Speed", 1);
    }

    if(m_photoElec.get() == true){
      //if cross
      if(m_opStick.getRawButtonPressed(2)){
        //set to intake
        mode = 0;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //else if square is pressed
      }else if(m_opStick.getRawButtonPressed(1)){
        //set to low rocket ball mode
        mode = 3;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //elif circle
      }else if(m_opStick.getRawButtonPressed(3)){
        //set to mid rocket ball mode
        mode = 5;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //if triangle
      }else if(m_opStick.getRawButtonPressed(4)){
        //set to high rocket ball position
        mode = 7;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      }else if(m_opStick.getPOV()==90){
        mode = 2;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      }
    }else{
      //if cross
      if(m_opStick.getRawButtonPressed(2)){
        //set to intake pos
        mode = 0;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //else if square is pressed
      }else if(m_opStick.getRawButtonPressed(1)){
        //set to cargo/low rocket hatch mode
        mode = 1;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //elif circle
      }else if(m_opStick.getRawButtonPressed(3)){
        //set to mid rocket hatch mode
        mode = 4;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      //if triangle
      }else if(m_opStick.getRawButtonPressed(4)){
        //set to high rocket hatch mode
        mode = 6;
        m_elevAddition = 0;
        m_pivotAddition = 0;
      }
    }

    if(m_opStick.getRawButtonPressed(7)){
      mode = 8;
      m_elevAddition = 0;
      m_pivotAddition = 0;
    }

    SmartDashboard.putNumber("mode", mode);

    switch(mode){
      //cargo ship/low rocket hatch
      case 1:
        m_elevPos = 0;
        m_pivotPos = m_pivotLimit;
        break;
      
      //cargo ship ball
      case 2:
        m_elevPos = 16000;
        m_pivotPos = m_pivotLimit;
        break;

      //low rocket ball
      case 3:
        m_elevPos = 10000;
        m_pivotPos = m_pivotLimit;
        break;

      //mid rocket hatch
      case 4:
        m_elevPos = 15000;
        m_pivotPos = m_pivotLimit;
        break;

      //mid rocket ball
      case 5:
        m_elevPos = 22000;
        m_pivotPos = m_pivotLimit;
        break;
        
      //high rocket hatch
      case 6:
        m_elevPos = 23500;
        m_pivotPos = m_pivotLimit;
        break;
      
      //high rocket ball
      case 7:
        m_elevPos = m_elevLimit;
        m_pivotPos = 2000;
        break;

      //defense
      case 8:
        m_elevPos = 0;
        m_pivotPos = -700;
      
      //intake
      default:
        m_elevPos = 0;
        m_pivotPos = m_pivotLimit;
        break;
    }

    if(m_opStick.getRawButton(13)){
      if(m_opStick.getPOV()==0 && (m_elevPos + m_elevAddition < m_elevLimit)){
        m_elevAddition += 100;
      }else if(m_opStick.getPOV()==180 && (m_elevPos + m_elevAddition > 0)){
        m_elevAddition -= 100;
      }
    }else{
      if(m_opStick.getPOV()==0 && (m_pivotPos + m_pivotAddition < m_pivotLimit)){
        m_pivotAddition += 100;
      }else if(m_opStick.getPOV()==180 && m_pivotPos + m_pivotAddition > MIN_PIVOT){
        m_pivotAddition -= 100;
      }
    }

    SmartDashboard.putNumber("Paddition", m_pivotAddition);
    SmartDashboard.putNumber("Eaddition", m_elevAddition);

    m_elevator.configReverseSoftLimitEnable(true);

    //when the magnetic sensor is triggered (i.e. elevator is at the bottom)
    if(m_elevSwitch.get()==false){
      //reset encoder to 0
      m_elevator.setSelectedSensorPosition(0);
    }

    if(Math.abs(m_stickyElevPos-m_elevPos) > 100 && m_elevAddition == 0 && ((m_hatchSnatcher.get()==Value.kReverse && m_pivot.getSelectedSensorPosition() > (HATCH_PIVOT+300)) || ( m_hatchSnatcher.get() == Value.kForward && m_pivot.getSelectedSensorPosition() > MIN_PIVOT+50))){
      pivotOut = false;
    }else if(m_elevPos + m_pivotAddition == 0){
      m_stickyElevPos = 0;
      if(m_elevSwitch.get()==true){
        m_elevator.configReverseSoftLimitEnable(false);
        if(m_elevator.getSelectedSensorPosition() > 1000){
          m_elevator.set(ControlMode.Position, 0);
        }else{
          m_elevator.set(ControlMode.PercentOutput, -0.2);
        }
      }
    }else{
      m_stickyElevPos = m_elevPos + m_elevAddition;
      m_elevator.set(ControlMode.Position, m_stickyElevPos, DemandType.ArbitraryFeedForward, elevator_kF);
    }

    if(m_opStick.getRawButtonPressed(5)){
      pivotOut = !pivotOut;
      m_pivotAddition = 0;
    }

    m_pivot.configReverseSoftLimitEnable(true);

    if(Math.abs(m_opStick.getRawAxis(1)) > 0.1){
      m_pivot.configReverseSoftLimitEnable(false);
      m_pivot.set(ControlMode.PercentOutput, m_opStick.getRawAxis(1)*-0.2, DemandType.ArbitraryFeedForward, pivot_kF);
      pivotOut = false;
    }else if (pivotOut == false && m_hatchSnatcher.get() == Value.kReverse){
      m_pivotPos = HATCH_PIVOT-MIN_PIVOT;
      m_pivot.set(ControlMode.Position, (HATCH_PIVOT-MIN_PIVOT), DemandType.ArbitraryFeedForward, pivot_kF);
    }else if (pivotOut == false){
      if(m_pivotSwitch.get()==true){
        m_pivot.configReverseSoftLimitEnable(false);
        if(m_pivot.getSelectedSensorPosition() > 2000){
          m_pivot.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, pivot_kF);
        }else if(m_pivot.getSelectedSensorPosition() > 1000){
          m_pivot.set(ControlMode.Position, -0.2);
        }else if(m_pivot.getSelectedSensorPosition() > 0){
          m_pivot.set(ControlMode.PercentOutput, -0.1, DemandType.ArbitraryFeedForward, pivot_kF);
        }
      }else{
        m_pivot.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, pivot_kF);
      }
      //m_pivot.set(ControlMode.Position, 0, DemandType.ArbitraryFeedForward, pivot_kF);
    }else if (m_elevator.getSelectedSensorVelocity()<=100 && Math.abs(m_elevator.getSelectedSensorPosition()-m_elevPos) < 1000){
      int pivot_epic = m_pivotPos + m_pivotAddition - MIN_PIVOT;
      m_pivot.set(ControlMode.Position, pivot_epic, DemandType.ArbitraryFeedForward, pivot_kF);
    }

    //when the magnetic pivot sensor is triggered (i.e. pivot is at the bottom)
    if(m_pivotSwitch.get()==false){
      //reset encoder to 0
      m_pivot.setSelectedSensorPosition(0);
    }
        
    SmartDashboard.putNumber("ElevPos", m_elevPos);
    SmartDashboard.putNumber("PivotPos", m_pivotPos);
    SmartDashboard.putNumber("elevator", m_elevator.getSelectedSensorPosition());
    SmartDashboard.putNumber("pivot", m_pivot.getSelectedSensorPosition());
    SmartDashboard.putBoolean("photosens",m_photoElec.get());

    double ballTarget = 0;
    double ballSpeed = Math.round(m_ballInMotor.getSpeed() * 10.0) / 10.0;

    if(m_opStick.getRawButton(6)){
      ballTarget = m_maxIntakeSpeed;
    }else if(m_opStick.getRawButton(8)){
      ballTarget = -m_maxIntakeSpeed;
    }else{
      ballTarget = 0;
    }

    /*if(ballSpeed>ballTarget){
      m_ballInMotor.setSpeed(ballSpeed - 0.1);
    }else if (ballSpeed<ballTarget){
      m_ballInMotor.setSpeed(ballSpeed + 0.1);
    }*/

    m_ballInMotor.set(ballTarget);

    /*double hatchTarget = 0;
    double hatchSpeed = Math.round(m_hatchInMotor.getSpeed() * 10.0) / 10.0;

    if(m_driveStick.getRawButton(5) || m_opStick.getRawButton(5)){
      hatchTarget = m_maxIntakeSpeed;
    }else if(m_driveStick.getRawButton(7) || m_opStick.getRawButton(7)){
      hatchTarget = -m_maxIntakeSpeed;
    }else{
      hatchTarget = 0;
    }

    if(hatchSpeed > hatchTarget){
      m_hatchInMotor.setSpeed(hatchSpeed - 0.1);
    }else if (hatchSpeed < hatchTarget){
      m_hatchInMotor.setSpeed(hatchSpeed + 0.1);
    }

    SmartDashboard.putNumber("Intake Motor Speed", m_hatchInMotor.getSpeed());*/

    if(m_opStick.getRawButtonPressed(9)|| m_driveStick.getRawButtonPressed(5)){
      m_hatchSnatcher.set(Value.kForward);
    }else if(m_opStick.getRawButtonPressed(10)||m_driveStick.getRawButtonPressed(6)){
      m_hatchSnatcher.set(Value.kReverse);
    }
  }

  @Override
  public void testInit() {
    System.out.println(m_pivotConfig);

    while(m_elevSwitch.get() == true){
      m_elevator.set(ControlMode.PercentOutput, 0.2, DemandType.ArbitraryFeedForward, elevator_kF);
    }
    
    m_pivot.setSelectedSensorPosition(0);
    m_pivotPos=0;
    m_elevPos=0;
    m_elevator.setSelectedSensorPosition(0);
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {

    //double pivotAngle = m_pivot.getSelectedSensorPosition() * 0.04395 + 45;
    //double pivot_kF = 0.87716*Math.cos(pivotAngle);

    //if(m_timer.get() %2 == 0){
      System.out.println(m_elevator.getSelectedSensorPosition());
    //}

    double m_stickyY = m_driveStick.getRawAxis(1)*-1;

    if(m_driveStick.getRawButton(13)){
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
        m_elevPos = m_elevLimit;
      }else if(m_driveStick.getPOV()==0 && m_elevPos < m_elevLimit){
        m_elevPos += 100;
      }else if(m_driveStick.getPOV()==180 && m_elevPos > 0){
        m_elevPos -= 100;
      }
    }else{
      //if cross
      if(m_driveStick.getRawButton(2)){
        //set to in position
        m_pivotPos = 0;
      //else if square is pressed
      }else if(m_driveStick.getRawButton(1)){
        //set to out position
        m_pivotPos = 1300;
      //if triangle
      }else if(m_driveStick.getRawButton(4)){
        //set to out position
        m_pivotPos = m_pivotLimit;
      }else if(m_driveStick.getPOV()==0 && m_pivotPos<m_pivotLimit){
        m_pivotPos += 100;
      }else if(m_driveStick.getPOV()==180 && m_pivotPos>0){
        m_pivotPos -= 100;
      }
    }

    if(m_elevPos == 0){
      if(m_elevSwitch.get()==true){
        if(m_elevator.getSelectedSensorPosition() > 1000){
          m_elevator.set(ControlMode.Position, 0);
        }else{
          m_elevator.set(ControlMode.PercentOutput, -0.2);
        }
      }
    }else{
      m_elevator.set(ControlMode.Position, m_elevPos, DemandType.ArbitraryFeedForward, elevator_kF);
    }

    m_pivot.set(ControlMode.Position, m_pivotPos,DemandType.ArbitraryFeedForward, pivot_kF);
    
    //when the magnetic sensor is triggered (i.e. elevator is at the bottom)
    if(m_elevSwitch.get()==false){
      //reset encoder to 0
      m_elevator.setSelectedSensorPosition(0, 0, 100);
    }

    if(SmartDashboard.getNumber("Maximum Motor Speed", 1)<=1 && SmartDashboard.getNumber("motorMaxSpeed", 1)>=-1){
      m_maxIntakeSpeed = SmartDashboard.getNumber("Maximum Motor Speed", 1);
    }

    double ballTarget = 0;

    if(m_driveStick.getRawButton(6)){
      ballTarget = m_maxIntakeSpeed;
    }else if(m_driveStick.getRawButton(8)){
      ballTarget = -m_maxIntakeSpeed;
    }else{
      ballTarget = 0;
    }

    m_ballInMotor.setSpeed(ballTarget);

    /*
    if (ballSpeed>ballTarget){
      m_ballInMotor.setSpeed(ballSpeed - 0.1);
    }else if (ballSpeed<ballTarget){
      m_ballInMotor.setSpeed(ballSpeed + 0.1);
    }
    */

    //unused motor hatch code
    /*double hatchTarget = 0;

    if(m_driveStick.getRawButton(5)){
      hatchTarget = m_maxIntakeSpeed;
    }else if(m_driveStick.getRawButton(7)){
      hatchTarget = -m_maxIntakeSpeed;
    }else{
      hatchTarget = 0;
    }

    m_hatchInMotor.set(hatchTarget);
    */

    /*
    if (m_hatchInMotor.getSpeed()>hatchTarget){
      m_hatchInMotor.setSpeed(m_hatchInMotor.getSpeed() - 0.1);
    }else if (m_hatchInMotor.getSpeed()<hatchTarget){
      m_hatchInMotor.setSpeed(m_hatchInMotor.getSpeed() + 0.1);
    }
    */

    SmartDashboard.putNumber("Extra Motor Speed", m_hatchInMotor.getSpeed());
    
    m_driveTrain.arcadeDrive(m_stickyY, m_driveStick.getX());

    if(m_driveStick.getRawButtonPressed(9)){
      m_hatchSnatcher.set(Value.kForward);
    }else if(m_driveStick.getRawButtonPressed(10)){
      m_hatchSnatcher.set(Value.kReverse);
    }

  }
}
