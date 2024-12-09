// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelShooterSubsystem;

public class FlywheelShooterCommand extends Command {
  /** Creates a new FlywheelShooterCommand. */
  private final FlywheelShooterSubsystem shooterSubsystem;

  public FlywheelShooterCommand(FlywheelShooterSubsystem subsystem) {
    shooterSubsystem = subsystem;
    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double pidResultInRPM = this.shooterSubsystem.calculatePID(3000); //calculates the PID with a given setpoint (3000RPM in this case)
    this.shooterSubsystem.setMotorsSpeedInRPM(pidResultInRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
