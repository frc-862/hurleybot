// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.thunder.hardware.ThunderBird;

public class Pivot extends SubsystemBase {

    private ThunderBird motor;
    private final PositionVoltage voltageController;
    public Pivot() {
        motor = new ThunderBird(9, "rio", false, 0.1, true);
        motor.configPIDF(0, 0.1, 0, 0, 0);
        motor.applyConfig();
        voltageController = new PositionVoltage(0).withSlot(0);
        motor.setControl(voltageController);
    }

    @Override
    public void periodic() {

    }

    public void setAngle(double position){
        voltageController.withPosition(position);
    }
}
