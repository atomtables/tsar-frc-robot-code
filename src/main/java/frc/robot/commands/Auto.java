package frc.robot.commands;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.AutoConstants;

public class Auto {
    private CommandSwerveDrivetrain swerve;

    public Auto(CommandSwerveDrivetrain swerve) {
        this.swerve = swerve;
    }
    

    public void initializeOdometry() {
        double initialHeading = TunerConstants.getGyroHeading();
        double x = AutoConstants.INITIAL_X;
        double y = AutoConstants.INITIAL_Y;
        
        swerve.resetPose(new Pose2d(x, y, Rotation2d.fromDegrees(initialHeading)));
    }

}
