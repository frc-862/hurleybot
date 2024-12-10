// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.thunder.LightningContainer;
import frc.thunder.filter.XboxControllerFilter;

public class RobotContainer extends LightningContainer {

    private XboxControllerFilter driver;

    public RobotContainer() {
    }

    @Override
    protected void initializeSubsystems() {
        driver = new XboxControllerFilter(0, 0.1, -1, 1, XboxControllerFilter.filterMode.SQUARED);
    }

    @Override
    protected void initializeNamedCommands() {
    }

    @Override
    protected void configureButtonBindings() {
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
