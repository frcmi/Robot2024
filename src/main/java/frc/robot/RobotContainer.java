// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SpeakerShooterSubsystem;
import frc.robot.commands.AutoChooserCommand;
import frc.robot.commands.TeleopSwerve;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.commands.AutoAlignCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToPositionPathPlanner;
import frc.robot.commands.SetTrailLights;
import frc.robot.subsystems.DriveStationSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  // private final AmpShooterSubsystem ampShooterSubsystem = new AmpShooterSubsystem();
  private final SpeakerShooterSubsystem speakerShooterSubsystem = new SpeakerShooterSubsystem();
  // private final DriveStationSubsystem m_driveStationSubsystem = new DriveStationSubsystem();
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  // public final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_DriverButton = new CommandXboxController(OperatorConstants.kDriverButtonPort);

  private final AutoChooser autoChooser = new AutoChooser();

  public final Pose2d gotoAutoThing = new Pose2d(5.0,5.0, new Rotation2d(Math.PI));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(
        new TeleopSwerve(
            swerveSubsystem, 
            () -> driverController.getLeftY(), 
            () -> driverController.getLeftX(), 
            () -> driverController.getRightX(), 
            () -> false //robotCentric.getAsBoolean()
        )
    );
    // Configure the trigger bindings
    configureBindings();
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
  private void configureBindings() {
    NamedCommands.registerCommand("E", new PrintCommand("YOOOOOO"));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Intake
    // driverController.leftBumper().whileTrue(intakeSubsystem.intakeAmp());
    // driverController.rightBumper().whileTrue(intakeSubsystem.intakeSpeaker());

    // Shooter
    driverController.rightTrigger().whileTrue(speakerShooterSubsystem.shootSpeaker());
    // // Shooter
    // driverController.a().whileTrue(speakerShooterSubsystem.shootSpeaker());
    // driverController.b().whileTrue(ampShooterSubsystem.shootAmp());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driveStationSubsystem.coop();
    // m_driverController.x().onTrue(m_driveStationSubsystem.ampSpeaker());//new SetTrailLights(m_driveStationSubsystem));
    // m_driverController.b().onTrue(new SetTrailLights(m_driveStationSubsystem, false));
    // m_driverController.a().whileTrue(new SetTrailLights(m_driveStationSubsystem, true));
    // m_driverController.y().onTrue(m_driveStationSubsystem.dropDisk());
    // m_driverController.leftBumper().onTrue(m_driveStationSubsystem.setLights());
    // m_driverController.rightBumper().onTrue(m_driveStationSubsystem.ledOff());
    //you have to press right bumper then left bumper to turn off the lights, I don't why, ask the lights
    
    // // m_DriverButton.button(5).onTrue(m_driveStationSubsystem.dropDisk());
    
    // // m_DriverButton.button(7).onTrue(m_driveStationSubsystem.coop());

    // // m_DriverButton.button(1).onTrue(m_driveStationSubsystem.ampSpeaker());

    // // m_DriverButton.button(9).onTrue(m_driveStationSubsystem.readyToAmp());

    // // m_DriverButton.button(3).onTrue(m_driveStationSubsystem.readyToSpeaker());

    // m_DriverButton.button(2).whileTrue(new SetTrailLights(m_driveStationSubsystem, true));
    // m_DriverButton.button(2).whileTrue(new SetTrailLights(m_driveStationSubsystem, true));
    
    //  m_DriverButton.button(3).and(m_DriverButton.button(4).and(m_DriverButton.button(5))).whileTrue(m_driveStationSubsystem.runRainbow());
    //made a rainbow command because its funny, probably won't use at comps though
    //made a rainbow command because its funny, probably won't use at comps though
    // m_driveStationSubsystem.coop();
    // m_driveStationSubsystem.setLights().schedule();
    driverController.y().onTrue(new InstantCommand(swerveSubsystem::zeroHeading, swerveSubsystem));
    // m_swerveSubsystem.setDefaultCommand(m_swerveSubsystem.test());
    
    driverController.leftTrigger().onTrue(new AutoAlignCommand(swerveSubsystem).getCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // return Autos.testAuto(swerveSubsystem, intakeSubsystem,  () -> swerveSubsystem.getPose()); // Just for testing, will implement autoChooser later
    // return new AutoChooserCommand(autoChooser);
    return Autos.ppAuto(swerveSubsystem, intakeSubsystem, speakerShooterSubsystem);
    //return new DriveToPositionPathPlanner(swerveSubsystem, swerveSubsystem::getPose, gotoAutoThing).gimmeCommand(); //Autos.ppAuto(swerveSubsystem, intakeSubsystem, speakerShooterSubsystem);
  }

  
}
