// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.SwerveSubsystem;
import frc.thunder.LightningContainer;
import frc.thunder.filter.XboxControllerFilter;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class RobotContainer extends LightningContainer {

    // private PhotonVision vision;
    

    
  private XboxController driverXbox;
  // The robot's subsystems and commands are defined here...
  private SwerveSubsystem drivebase;
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the rotational velocity 
  // buttons are quick rotation positions to different ways to face
  // WARNING: default buttons are on the same buttons as the ones defined in configureBindings
  AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftY(),
                                                                                               OperatorConstants.LEFT_Y_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getLeftX(),
                                                                                               OperatorConstants.LEFT_X_DEADBAND),
                                                                 () -> -MathUtil.applyDeadband(driverXbox.getRightX(),
                                                                                               OperatorConstants.RIGHT_X_DEADBAND),
                                                                 driverXbox::getYButtonPressed,
                                                                 driverXbox::getAButtonPressed,
                                                                 driverXbox::getXButtonPressed,
                                                                 driverXbox::getBButtonPressed);

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
  Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX(),
      () -> driverXbox.getRightY());

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
  Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRightX() * -1);

  Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
      () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> driverXbox.getRawAxis(2));

    @Override
    protected void initializeSubsystems() {
        drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
        driverXbox = new XboxController(0);

    }

    @Override
    protected void initializeNamedCommands() {
    }

    @Override
    protected void configureButtonBindings() {

        (new Trigger(driverXbox::getAButtonPressed)).onTrue((Commands.runOnce(drivebase::zeroGyro)));
        (new Trigger(driverXbox::getAButtonPressed)).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        (new Trigger(driverXbox::getRightBumperPressed)).onTrue(Commands.none());
        drivebase.setDefaultCommand(
            !RobotBase.isSimulation() ? driveFieldOrientedDirectAngle : driveFieldOrientedDirectAngleSim);
    
    }

    @Override
    protected void configureSystemTests() {
    }

    @Override
    protected void configureDefaultCommands() {
    }

    @Override
    protected void releaseDefaultCommands() {
    }

    @Override
    protected void initializeDashboardCommands() {
    }

    @Override
    protected void configureFaultCodes() {
    }

    @Override
    protected void configureFaultMonitors() {
    }

    @Override
    protected Command getAutonomousCommand() {
        return null;
    }
}
