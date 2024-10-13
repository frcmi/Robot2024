// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AmpArmConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.ChosenAuto;
import frc.robot.subsystems.*;
import frc.robot.commands.TeleopSwerve;
import frc.robot.generated.TunerConstants;

import java.sql.Driver;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.SetTrailLights;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final CommandXboxController m_DriverButton = new CommandXboxController(OperatorConstants.kDriverButtonPort);

  // The robot's subsystems and commands are defined here...
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public final AmpArmSubsystem ampArmSubsystem = new AmpArmSubsystem();
  public final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  public final AmpShooterSubsystem ampShooterSubsystem = new AmpShooterSubsystem(ampArmSubsystem, m_LEDSubsystem);
  public final SpeakerShooterSubsystem speakerShooterSubsystem = new SpeakerShooterSubsystem(driverController.leftTrigger());
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(speakerShooterSubsystem.beambreak::get, m_LEDSubsystem);
  // public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  public final CommandSwerveDrivetrain commandSwerveDrivetrain = TunerConstants.DriveTrain;
  public final ClimberSubsystem climberSubsystem = new ClimberSubsystem();
  // public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();

  public final Pose2d gotoAutoThing = new Pose2d(2.843,5.819, new Rotation2d(Math.PI));
  public final Pose2d gotoAutoThing2 = new Pose2d(8.244,2.471, new Rotation2d(Math.PI));

  private boolean slewLimited = false;
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  
  public RobotContainer() {
    // if (Robot.isSimulation()) swerveSubsystem.visionSubsystemForSim = visionSubsystem;

    commandSwerveDrivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        commandSwerveDrivetrain.applyRequest(() -> drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));
    // Configure the trigger bindings
    configureBindings();
    configurePPTriggers();
    configureAutoChooser();
  }

  private void configurePPTriggers() {
    NamedCommands.registerCommand("Flash rainbow", m_LEDSubsystem.runRainbow());
    NamedCommands.registerCommand("intake", new RepeatCommand(intakeSubsystem.intakeSpeaker()));
    NamedCommands.registerCommand("stop intake", intakeSubsystem.stop());

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */

   private void configureAutoChooser() {
   }

  private void configureBindings() {
    
    // new Trigger(
    //   () -> !speakerShooterSubsystem.beambreak.get()
    // )
    //   .whileTrue(
    //     new RepeatCommand(
    //       new PrintCommand("FUN")
    //     )
    //   );
    driverController.start().whileTrue(ampShooterSubsystem.fixStuckNote());
    // driverController.start().onTrue(new InstantCommand(() -> {
    //   this.slewLimited = !this.slewLimited;
    //   SmartDashboard.putBoolean("Slew Limited", this.slewLimited);
    // }));
    // driverController.povLeft()
    //   .whileTrue(speakerShooterSubsystem.shoot());

    // new Trigger(() -> !speakerShooterSubsystem.beambreak.get())
    //   .whileTrue(speakerShooterSubsystem.shoot());
    // new Trigger()
    //         .or(
    //         .whileTrue(speakerShooterSubsystem.shoot());

    // Ben Control Scheme *****************************

    // Intake
    driverController.rightBumper().whileTrue(intakeSubsystem.intakeSpeaker());
    // RB Intake speaker
    // driverController.rightBumper().whileTrue(intakeSubsystem.intakeSpeaker());
    // RT Shoot speaker
    driverController.rightTrigger().whileTrue(intakeSubsystem.intakeSpeakerNoBeamBreak(IntakeConstants.kSpeakerShootSpeed));

    // LB Intake amp
    driverController.leftBumper().whileTrue(intakeSubsystem.intakeAmp().alongWith(ampShooterSubsystem.intakeAmp()));
    // LT Shoot amp
    driverController.leftTrigger().whileTrue(ampShooterSubsystem.shootAmp());
    driverController.leftTrigger().onFalse(ampArmSubsystem.lowerArm());

    // A Raise amp arm
    Command raiseAmp = ampArmSubsystem.raiseToAmp().withTimeout(1);
    NamedCommands.registerCommand("Shoot Amp", raiseAmp
    .andThen(ampShooterSubsystem.shootAmp()).withTimeout(0.75)
    .andThen(ampArmSubsystem.moveTo(AmpArmConstants.kMinAngle)).withTimeout(0.5));
    
    driverController.a().onTrue(ampArmSubsystem.raiseToAmp());
    // driverController.a().onTrue(ampArmSubsystem.moveTo(AmpArmConstants.kShootAngle));

    // Y Reset Field Orientation
    driverController.y().onTrue(commandSwerveDrivetrain.runOnce(() -> commandSwerveDrivetrain.seedFieldRelative()));

    // B Spit out note
    driverController.b().whileTrue(ampShooterSubsystem.shootAmp().alongWith(intakeSubsystem.extractNote()));

    // povUp Raise Climber
    driverController.povUp().whileTrue(climberSubsystem.up());
    // povDown Lower Climber
    driverController.povDown().whileTrue(climberSubsystem.down());

    // povLeft Auto Shoot amp
    // povRight Auto Shoot speaker

  //  driverController.povRight().onTrue(new AutoAlignCommand(swerveSubsystem));

    // X Toggle Sensitivity (translation and rotation)
    // driverController.x().onTrue(new InstantCommand(swerveSubsystem::switchSensitivity, swerveSubsystem));

    //small backup
    // TODO make this work for commandSwerveDriveTrain
    //driverController.povLeft().onTrue(swerveSubsystem.backupSlightly());
    // use this every time the driver goes up to the amp to score

    // ***********************************************

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_LEDSubsystem.coop();
    // m_driverController.x().onTrue(m_LEDSubsystem.ampSpeaker());//new SetTrailLights(m_LEDSubsystem));
    // m_driverController.b().onTrue(new SetTrailLights(m_LEDSubsystem, false));
    // m_driverController.a().whileTrue(new SetTrailLights(m_LEDSubsystem, true));
    // m_driverController.y().onTrue(m_LEDSubsystem.dropDisk());
    // m_driverController.leftBumper().onTrue(m_LEDSubsystem.setLights());
    // m_driverController.rightBumper().onTrue(m_LEDSubsystem.ledOff());
    //you have to press right bumper then left bumper to turn off the lights, I don't why, ask the lights
    
    m_DriverButton.button(5).onTrue(m_LEDSubsystem.dropDisk());
    
    m_DriverButton.button(7).onTrue(m_LEDSubsystem.coop());

    m_DriverButton.button(1).onTrue(m_LEDSubsystem.ampSpeaker());

    m_DriverButton.button(9).onTrue(m_LEDSubsystem.ledOff());

    // // m_DriverButton.button(3).onTrue(m_LEDSubsystem.readyToSpeaker());

    m_DriverButton.button(2).whileTrue(new SetTrailLights(m_LEDSubsystem, false));
    // m_DriverButton.button(2).whileTrue(new SetTrailLights(m_LEDSubsystem, true));
    
    NamedCommands.registerCommand("Flash rainbow", m_LEDSubsystem.runRainbow());
    m_DriverButton.button(8).whileTrue(m_LEDSubsystem.runRainbow());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // SmartDashboard.putString("command name swerve", "test");
    // if (swerveSubsystem.getCurrentCommand() != null) {
    //   SmartDashboard.putString("command name swerve", swerveSubsystem.getCurrentCommand().getName());
    // }

    return autoChooser.getSelected();
  }
}
