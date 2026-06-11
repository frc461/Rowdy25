package io.github.frc461.rowdy25.commands;

/*
 * Copyright (C) 2025-present 461 Boosters FIRST, Inc. dba Westside Robotics - The Rowdy 25.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

import edu.wpi.first.wpilibj2.command.Command;
import io.github.frc461.rowdy25.subsystems.intake.Intake;

/**
 * A command that manages the intake subsystem through a state machine.
 * <p>
 * The command handles multiple intake states including normal intake, slow intake,
 * outtake, override, and algae holding. It transitions between states based on
 * sensor readings such as beam break detection and coral presence.
 *
 * @author Eugene Zhang, <a href="https://github.com/ez500">GitHub</a>
 */
public class IntakeCommand extends Command {
    /** The intake subsystem. */
    private final Intake intake;

    /**
     * Constructs an IntakeCommand.
     *
     * @param intake The intake subsystem.
     */
    public IntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    /**
     * Executes the command's control logic every 20ms.
     * <p>
     * Evaluates the current intake state and applies the appropriate motor speed
     * based on beam break and coral detection sensor feedback. State transitions
     * occur automatically as conditions change.
     */
    @Override
    public void execute() {
        switch (intake.getState()) {
            case INTAKE:
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSlowState();
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setOuttakeSlowState();
                } else {
                    intake.setIntakeSpeed(0.45);
                }
                break;
            case INTAKE_SLOW:
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSpeed(0.15);
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setOuttakeSlowState();
                } else {
                    intake.setIntakeState(false);
                }
                break;
            case INTAKE_OUT:
                intake.setIntakeSpeed(0.65);
                break;
            case INTAKE_OVERRIDE:
                intake.setIntakeSpeed(0.35);
                break;
            case OUTTAKE:
                intake.setIntakeSpeed(-0.5);
                break;
            case OUTTAKE_SLOW:
                if (intake.hasCoral() || intake.algaeStuck()) {
                    intake.setIdleState();
                } else if (intake.coralEntered() && !intake.beamBreakBroken()) {
                    intake.setIntakeSlowState();
                } else if (intake.beamBreakBroken() && !intake.coralEntered()) {
                    intake.setIntakeSpeed(-0.15);
                } else {
                    intake.setIntakeState(false);
                }
                break;
            case OUTTAKE_L1:
                intake.setIntakeSpeed(-0.4);
                break;
            case HAS_ALGAE:
                intake.setIntakeSpeed(0.03); // TODO SHOP: TEST THIS
                break;
            case IDLE:
                intake.setIntakeSpeed(0.0);
                break;
        }
    }
}