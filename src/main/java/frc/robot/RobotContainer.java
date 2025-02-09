// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swerve.Pigeon2IO;
import frc.robot.subsystems.swerve.SDSModuleIOSpark;
import frc.robot.subsystems.swerve.SwerveDrive;

public class RobotContainer {
    private CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    // private CommandXboxController auxController = new CommandXboxController(OperatorConstants.kAuxiliaryControllerPort);

    private SwerveDrive swerve;

    public RobotContainer() {
        Preferences.removeAll();

        swerve = new SwerveDrive(
            new Pigeon2IO(),
            new SDSModuleIOSpark(0),
            new SDSModuleIOSpark(1),
            new SDSModuleIOSpark(2),
            new SDSModuleIOSpark(3)
        );

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(swerve.runDriveInputs(
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            driverController.leftBumper()::getAsBoolean,
            driverController.leftTrigger()::getAsBoolean
        ));

        // driverController.a().onTrue(swerve.runTestDrive());
        // driverController.a().onFalse(swerve.runStopDrive());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
