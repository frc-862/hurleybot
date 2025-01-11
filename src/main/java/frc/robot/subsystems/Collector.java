// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;

public class Collector extends SubsystemBase {
    private ThunderBird motor;
    public Collector() {
        motor = new ThunderBird(10, "Canivore", false, 60, false);
    }

    @Override
    public void periodic() {
    }

    public void setPower(double power){
        motor.setControl(new DutyCycleOut(power));
    }
}