// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import frc.robot.Constants.RollerConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.OperatorConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.commands.ArmDownCommand;
import frc.robot.commands.ArmUpCommand;
import frc.robot.commands.ClimberDownCommand;
import frc.robot.commands.ClimberUpCommand;
import frc.robot.commands.CoralOutCommand;
import frc.robot.commands.CoralStackCommand;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final RollerSubsystem roller = new RollerSubsystem();
    public final ArmSubsystem arm = new ArmSubsystem();
    private final SendableChooser<Command> autoChooser;
    private double slow = 1;

    public RobotContainer() { 
        NamedCommands.registerCommand("DepositCoral", new CoralOutCommand(roller, 0.1));
        NamedCommands.registerCommand("ArmDown", new ArmDownCommand(arm, 0.5));
        NamedCommands.registerCommand("ArmUp", new ArmUpCommand(arm, 0.5));

        autoChooser = AutoBuilder.buildAutoChooser("Wait Auto");
        autoChooser.setDefaultOption("Wait Auto", new PathPlannerAuto("Wait Auto")); 
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
}

    public Command getAutonomousCommand() {/*
    try {
        return new PathPlannerAuto("Wait Auto");
    } catch (Exception e) {
        DriverStation.reportError("Error loading autonomous path: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    */
   return autoChooser.getSelected();
}


    private void configureBindings() {
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                        .withRotationalRate(joystick.getRightX() * MaxAngularRate)));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(
                () -> point.withModuleDirection(new Rotation2d(-joystick.getRightY(), -joystick.getRightX()))));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.povUp().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.povDown().onTrue(
            new SequentialCommandGroup(
                new ArmDownCommand(arm, 0.2),
                new AlgieInCommand(roller, 0.5),
                new ArmUpCommand(arm, 0.5)
            )
        );
        joystick.leftBumper().whileTrue(new AlgieInCommand(roller, 1));
        joystick.rightBumper().whileTrue(new AlgieOutCommand(roller, 1));
        joystick.leftTrigger().whileTrue(new ArmUpCommand(arm, 1));
        joystick.rightTrigger().whileTrue(new ArmDownCommand(arm, 1));
        joystick.x().onTrue(new SequentialCommandGroup(
            new ParallelCommandGroup(
            new CoralOutCommand(roller, 0.05),
            new ArmDownCommand(arm, 0.15)
            ),
            new ArmUpCommand(arm, 0.5)
        ));
        joystick.y().onTrue(
            new SequentialCommandGroup(
                new CoralStackCommand(roller, 0.1),
                new ArmDownCommand(arm, 0.5),
                new ArmUpCommand(arm, 0.5)
            )
        );
        joystick.a().whileTrue(new ClimberUpCommand(climber));
        joystick.b().whileTrue(new ClimberDownCommand(climber));
    }
}
