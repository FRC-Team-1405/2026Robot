package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class CalibrateTurret extends Command {
    private Turret theTurret;

    public CalibrateTurret(Turret calibrateTurret) {
        theTurret = calibrateTurret;
        addRequirements(theTurret);
    }

    @Override
    public void initialize() {
        theTurret.turnClockwise();
    }
   
    @Override
    public boolean isFinished() {
        return (theTurret.isOnLimitSwitch());
    }

    @Override
    public void end(boolean interrupted) {
        theTurret.stopTurret();
        theTurret.calibrateClock();
    }
}
