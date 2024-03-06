# Robot2024

FRC Mercer Island Robotics robot code. Uses WPILib 2024.1.1.

## Controller Button Mappings
- Intake Speaker: right bumper
- Intake Speaker Ignoring Beambreak: right trigger
- Intake Amp: left bumper
- Shoot Amp: left trigger
- Deploy Amp Arm: a
- Reset Heading: y
- Extract Note: b
- Climbers Up: d-pad/pov up
- Climbers Down: d-pad/pov down
- Auto align speaker: d-pad/pov right
- Sensitivity Switch: x

## Getting started

All code is in the `src/main/java/` directory. Main robot code is in the `frc.robot` package. Utility and math classes are in `frc.lib`. Each subsystem is used for a specific purpose, as described below:
- `AmpArm`: Driver for the amp shooter arm
- `AmpShooter`: Driver for the amp shooter mechanism
- `Climber`: Driver for the chain climber
- `DriveStation`: LED controller
- `Intake`: Driver for the note intake system
- `SpeakerShooter`: Driver for the speaker shooter mechanism
- `Swerve`: Driver for the robot swerve drive base
- `Vision`: Driver for the mounted camera; estimates position within the field