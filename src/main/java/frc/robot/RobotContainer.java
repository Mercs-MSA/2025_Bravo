// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.*;
import frc.robot.Constants.ScoringStageVal;
import frc.robot.subsystems.Elevator1;
import frc.robot.subsystems.Elevator2;
import frc.robot.subsystems.FunnelPivot;
import frc.robot.subsystems.IntakeBeambreak;
import frc.robot.subsystems.IntakeFlywheels;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CANdle_LED;
import frc.robot.commands.StaticCustomCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);


    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Elevator1 m_Elevator1 = new Elevator1(false);

    public final Elevator2 m_Elevator2 = new Elevator2(false);

    public final Climber m_Climber = new Climber(false);

    public final IntakeFlywheels m_IntakeFlywheels = new IntakeFlywheels(true);

    public final IntakeBeambreak m_intakeBeamBreak = new IntakeBeambreak();

    public final FunnelPivot m_FunnelPivot = new FunnelPivot(true);

    public final CANdle_LED m_leds = new CANdle_LED();

    private final SendableChooser<Command> autoChooser;

    private Map<String, Command> m_map = Map.ofEntries(
        Map.entry("Command 111", Commands.print("Command one was selected!")),
        Map.entry("PathWithDriveToPos", Commands.sequence(StaticCustomCommands.setDriveToPos("ReefTest"),StaticCustomCommands.toPos(drivetrain))),
        Map.entry("Intake Collect", StaticCustomCommands.intakeCollect(m_IntakeFlywheels, m_intakeBeamBreak, MaxAngularRate)),
        Map.entry("Score", StaticCustomCommands.intakeOut(m_IntakeFlywheels, m_intakeBeamBreak, MaxAngularRate)),
        Map.entry("L1", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.L1),StaticCustomCommands.elevatorToStage())),
        Map.entry("L2", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.L2),StaticCustomCommands.elevatorToStage())),
        Map.entry("L3", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.L3),StaticCustomCommands.elevatorToStage())),
        Map.entry("L4", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.L4),StaticCustomCommands.elevatorToStage())),
        Map.entry("ELEVIntakePos", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.INTAKEREADY),StaticCustomCommands.elevatorToStage())),
        Map.entry("MoveFunnel", Commands.sequence(StaticCustomCommands.changeScoreStage(ScoringStageVal.INTAKEREADY),StaticCustomCommands.funnelPivot(Constants.FunnelPivotConstants.posUp))),
        Map.entry("Brake", drivetrain.applyRequest(() -> brake)), //WHILETRUE
        Map.entry("Thingy", drivetrain.applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX())))), //WHILETRUE
        Map.entry("THingy 2", drivetrain.sysIdDynamic(Direction.kForward)), //WHILETRUE
        Map.entry("Thingy 3", drivetrain.sysIdDynamic(Direction.kReverse)), //WHILETRUE
        Map.entry("Thingy 4", StaticCustomCommands.intakeOut(m_IntakeFlywheels, m_intakeBeamBreak, 5)),
        Map.entry("Thingy 5", drivetrain.runOnce(() -> drivetrain.seedFieldCentric())),
        Map.entry("Thingy 6", StaticCustomCommands.setDriveToPos("Source").andThen(StaticCustomCommands.changeScoreStage(ScoringStageVal.INTAKEREADY)).andThen(
            new ParallelCommandGroup(StaticCustomCommands.toPos(drivetrain), StaticCustomCommands.elevatorToStage(),StaticCustomCommands.intakeCollect(m_IntakeFlywheels, m_intakeBeamBreak, 1)))), //WHILE TRUE
        Map.entry("Thingy 9", StaticCustomCommands.changeScoreStage(ScoringStageVal.L1)),
        Map.entry("Thingy 10", StaticCustomCommands.changeScoreStage(ScoringStageVal.L2)),
        Map.entry("Thingy 11", StaticCustomCommands.changeScoreStage(ScoringStageVal.L3)),
        Map.entry("Thingy 12", StaticCustomCommands.changeScoreStage(ScoringStageVal.L4)),
        Map.entry("Thingy 13", StaticCustomCommands.changeScoreStage(ScoringStageVal.CLIMBING)),
        Map.entry("Thingy 14", new SequentialCommandGroup(StaticCustomCommands.changeScoreStage(ScoringStageVal.INTAKEREADY),StaticCustomCommands.elevatorToStage())),
        Map.entry("Thingy 15", StaticCustomCommands.climbToggle()),
        Map.entry("Thingy 16", StaticCustomCommands.funnelToggle()),
        Map.entry("Thingy 17", StaticCustomCommands.intakeCollect(m_IntakeFlywheels, m_intakeBeamBreak, 8)),
        Map.entry("Thingy 18", StaticCustomCommands.seedToMegaTag(drivetrain, Constants.VisionConstants.limelightFrontName)),
        Map.entry("Thingy 19", StaticCustomCommands.loadDriveToPos(0).andThen(new ParallelCommandGroup (
            StaticCustomCommands.toPos(drivetrain),StaticCustomCommands.elevatorToStage(),StaticCustomCommands.candleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Strobe)))),//WHILETRUE
        Map.entry("Thingy 20", StaticCustomCommands.loadDriveToPos(1).andThen(new ParallelCommandGroup (
            StaticCustomCommands.toPos(drivetrain),StaticCustomCommands.elevatorToStage(),StaticCustomCommands.candleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Strobe)))), //WHILETRUE
        Map.entry("Thingy 21", StaticCustomCommands.candleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Larson)),
        Map.entry("Thingy 22", StaticCustomCommands.candleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Twinkle))
        );

       
    public RobotContainer() {
        NamedCommands.registerCommands(m_map);

        autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");
        SmartDashboard.putData("Auto Mode", autoChooser);
        configureBindings();
        driverControls();
        operatorControls();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))
        ));
        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driver.back().and(driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driver.back().and(driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));   
        driver.start().and(driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driver.start().and(driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        drivetrain.registerTelemetry(logger::telemeterize);
        // reset the field-centric heading on left bumper press
        // driver.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    }

        public void driverControls() {

            // driver.x().onTrue(new CommandElevatorToPos(Constants.Elevator1Constants.positionUp));
            // driver.y().onTrue(new CommandElevatorToPos(Constants.Elevator1Constants.positionDown));


            //DONE
            driver.leftBumper().onTrue(new CommandIntakeOut(m_IntakeFlywheels, m_intakeBeamBreak, 5));

            //DONE
            driver.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

            //DONE
            driver.back().onTrue(new CommandCandleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Larson));
            driver.start().onTrue(new CommandCandleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Twinkle));
            //DONE
            driver.a().whileTrue(
                new CommandSetDriveToPos("Source").andThen(
                new CommandChangeScoreStage(ScoringStageVal.INTAKEREADY)).andThen(
                new ParallelCommandGroup(
                    new CommandToPos(drivetrain), 
                    new CommandElevatorToStage(),
                    new CommandIntakeCollect(m_IntakeFlywheels, m_intakeBeamBreak, 1)
            )));

            //DONE
            driver.leftTrigger(0.8).whileTrue(new CommandLoadDriveToPos(() -> Constants.DriveToPosRuntime.autoTargets.get(0)).andThen(new ParallelCommandGroup (
                new CommandToPos(drivetrain),
                new CommandElevatorToStage(),
                new CommandCandleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Strobe)
                )));//keep
            //DONE
            driver.rightTrigger(0.8).whileTrue(new CommandLoadDriveToPos(() -> Constants.DriveToPosRuntime.autoTargets.get(1)).andThen(new ParallelCommandGroup (
                new CommandToPos(drivetrain),
                new CommandElevatorToStage(),
                new CommandCandleSetAnimation(m_leds, CANdle_LED.AnimationTypes.Strobe)
                )));//keep
            //DON'T NEED?
            // driver.leftTrigger().whileTrue(new CommandSetDriveToPos("ReefTest").andThen(new ParallelCommandGroup (
            //     new CommandToPos(drivetrain),
            //     new CommandElevatorToStage()
            //     )));//keep

          
        }

        public void operatorControls(){

            //DONE
            operator.pov(180).onTrue(new CommandChangeScoreStage(ScoringStageVal.L1));
            //DONE
            operator.pov(270).onTrue(new CommandChangeScoreStage(ScoringStageVal.L2));
            //DONE
            operator.pov(0).onTrue(new CommandChangeScoreStage(ScoringStageVal.L3));
            //DONE
            operator.pov(90).onTrue(new CommandChangeScoreStage(ScoringStageVal.L4));

            //DONE
            operator.a().onTrue(new CommandChangeScoreStage(ScoringStageVal.CLIMBING));
            //DONE
            operator.b().onTrue(new SequentialCommandGroup(
                new CommandChangeScoreStage(ScoringStageVal.INTAKEREADY),
                new CommandElevatorToStage()));

            //DONE
            operator.leftStick().onTrue(new CommandClimbToggle());
            //DONE
            operator.rightStick().onTrue(new CommandFunnelToggle());
            //DONE
            operator.leftBumper().onTrue(new CommandIntakeCollect(m_IntakeFlywheels, m_intakeBeamBreak, 8));

            //DONE
            driver.y().onTrue(new SeedToMegaTag(drivetrain, Constants.VisionConstants.limelightFrontName));

            // SmartDashboard.putData("SeedToMegaTag1_Front", new SeedToMegaTag(drivetrain, Constants.VisionConstants.limelightFrontName));
            // SmartDashboard.putData("SeedToMegaTag1_Back", new SeedToMegaTag(drivetrain, Constants.VisionConstants.limelightBackName));

        



        }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected(); 
    }
}
