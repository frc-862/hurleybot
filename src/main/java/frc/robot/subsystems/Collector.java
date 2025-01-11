// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;

public class Collector extends SubsystemBase {
    private ThunderBird motor;
    private VelocityVoltage velocityController;
    public Collector() {
        motor = new ThunderBird(10, "rio", false, 0, false);
        velocityController = new VelocityVoltage(0).withSlot(0);
        motor.configPIDF(0, 0.1, 0, 0, 0);
        motor.applyConfig();
        motor.setControl(velocityController);
    }

    @Override
    public void periodic() {

    }

    public void setVelocity(double velocity){
        velocityController.withVelocity(velocity);
    }

    public Command getCommand(double velocity){
        return new StartEndCommand(() -> setVelocity(velocity),  () -> setVelocity(0));
    }
}
