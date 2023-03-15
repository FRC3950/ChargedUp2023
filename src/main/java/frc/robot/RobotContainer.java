package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    //Test Variables
    double armAngle = 200;

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick manipulate = new Joystick(1);
    //private final Joystick buttonBox = new Joystick(1);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton startCenteringDrive = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Telescope s_Telescope = new Telescope();
    private final Wrist s_Wrist = new Wrist();
    private final ArmSubsystem s_Arm = new ArmSubsystem();

    /* PathPlanner */
    HashMap<String, Command> eventMap = new HashMap<>();

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        s_Swerve::getPose, // Pose2d supplier
        s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.kSwerve.swerveKinematics,
        new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(1.0, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
        s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
        eventMap,
        true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
        s_Swerve // The drive subsystem. Used to properly set the requirements of path following commands
    );


    /* Commands */
    private final AutoBalanceCommand balanceCommand = new AutoBalanceCommand(s_Swerve);
    private final exampleAuto exampleAuto = new exampleAuto(s_Swerve);
    private final Command a = s_Arm.zeroSensorFalcons();

    private final SequentialCommandGroup armToMid = new ArmToAngleGroup(s_Arm, 275.5);
    private final SequentialCommandGroup armToHigh = new ArmToAngleGroup(s_Arm, 295.5);
    private final SequentialCommandGroup goToIntakePosition = new IntakeOut_CommandGroup(s_Wrist, s_Arm, s_Telescope, s_Intake);


    private final SequentialCommandGroup armTo_0 = new ArmToAngleGroup(s_Arm, 0 );
    private final SequentialCommandGroup armTo_50 = new ArmToAngleGroup(s_Arm, 50);
    private final SequentialCommandGroup armTo_100 = new ArmToAngleGroup(s_Arm, 100);
    private final SequentialCommandGroup armTo_150 = new ArmToAngleGroup(s_Arm, 150);
    private final SequentialCommandGroup armTo_200 = new ArmToAngleGroup(s_Arm, 200);
    private final SequentialCommandGroup armTo_250 = new ArmToAngleGroup(s_Arm, 250);
    private final SequentialCommandGroup armTo_275 = new ArmToAngleGroup(s_Arm, 275);



    
    /* Auto Commands */
    private SendableChooser<Command> autoChooser = new SendableChooser<>();
    private SendableChooser<Command> armToAngleSelect = new SendableChooser<>();
    Command fullAuto = autoBuilder.fullAuto(PathPlanner.loadPathGroup("auto_DriveToCone", 3, 3));






    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));


        s_Telescope.setDefaultCommand(
            new TelescopeBangBang(
                s_Telescope,
                () -> -1.0 * manipulate.getRawAxis(5) * manipulate.getRawAxis(5) * Math.signum(manipulate.getRawAxis(5)) 
                 //() -> -1 *manipulate.getRawAxis(5)
                 )
                 );

        s_Arm.setDefaultCommand(
            new ArmPercentCommand(
                s_Arm,
                 () -> -0.75 * manipulate.getRawAxis(1) * manipulate.getRawAxis(1) * Math.signum(manipulate.getRawAxis(1)) 
                 )
                 );

        s_Wrist.setDefaultCommand(
            new WristPercentCommand(
                s_Wrist,

                () -> Math.abs(manipulate.getRawAxis(XboxController.Axis.kLeftTrigger.value)) > Math.abs(manipulate.getRawAxis(XboxController.Axis.kRightTrigger.value)) ? 
               0.3 * manipulate.getRawAxis(XboxController.Axis.kLeftTrigger.value) : 
               0.6 * -manipulate.getRawAxis(XboxController.Axis.kRightTrigger.value)

                //() -> -0.8 * manipulate.getRawAxis(4) * manipulate.getRawAxis(4) * Math.signum(manipulate.getRawAxis(4)) 

                
                //() ->  -0.7*manipulate.getRawAxis(4)
                 )
                 );

        // Configure the button bindings
        configureButtonBindings();

        // Autochooser
        //autoChooser.addOption("Example S Curve", exampleAuto);

       

        createAllAutoPathCommandsBasedOnPathDirectory();


        SmartDashboard.putData("Auto Selection", autoChooser);

        SmartDashboard.putData(s_Arm);
        SmartDashboard.putData("Akjkjtuo Balance", balanceCommand);

        SmartDashboard.putData("Lock Arm (Manual)", new InstantCommand(s_Arm::lockArm));
        SmartDashboard.putData("Unlock Arm (Manual)",new InstantCommand(s_Arm::unlockArm));

        SmartDashboard.putData("Reset Mag Enocder", new InstantCommand(s_Arm::resetEncoderCountArmMotors));

        SmartDashboard.putData("Rise Arm To Mid Angle", armToMid);
        SmartDashboard.putData("Rise Arm To High Angle", armToHigh);

       SmartDashboard.putData("Go to intake",goToIntakePosition);


       //Arm to angle
    
        SmartDashboard.putData("Arm Move to (0) ", armTo_0);
        SmartDashboard.putData("Arm Move to (50) ", armTo_50);
        SmartDashboard.putData("Arm Move to (100) ", armTo_100);
        SmartDashboard.putData("Arm Move to (150) ", armTo_150);
        SmartDashboard.putData("Arm Move to (200) ", armTo_200);
        SmartDashboard.putData("Arm Move to (250) ", armTo_250);
        SmartDashboard.putData("Arm Move to (275) ", armTo_275);

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); //Button this way might be 'safer' since the button is private/final when defined outside the constructor. //Honestly not sure what the best practice is or if it matters. Probably would never collide if we kept it public/changeable... idk?

        new JoystickButton(driver, XboxController.Button.kB.value) // Should eventually do all buttons like this?
            .whileTrue(balanceCommand);

         new JoystickButton(driver, XboxController.Button.kLeftBumper.value)
             .onTrue(new runPathAuto(s_Swerve, Constants.PathPlannerSimpleTrajectories.advanceNorth_22inches));

        

           

        startCenteringDrive.whileTrue(s_Swerve.driveHorizontalCommand());
            //This demonstrates Instance Command FActory Methods - it's cool :D
            //It turns to Zero Heading, might need to add PID or change to CLOSED LOOP
        new JoystickButton(driver, XboxController.Button.kA.value)
            .whileTrue(s_Swerve.turnToZeroCommand());


        new JoystickButton(manipulate, XboxController.Button.kX.value)
            .whileTrue(new StartEndCommand(() -> s_Intake.setIntake(0.75),  () -> s_Intake.setIntake(0.0), s_Intake));
        
        new JoystickButton(manipulate, XboxController.Button.kY.value)
            .whileTrue(new StartEndCommand(() -> s_Intake.setIntake(-0.75), () -> s_Intake.setIntake(0.0), s_Intake));
        
        new JoystickButton(manipulate, XboxController.Button.kRightBumper.value)
            .onTrue(new InstantCommand(s_Intake::toggleSolenoid, s_Intake));

    }

    /**
     * This method creates a {@link runPathAuto} command for each saved path and
     * adds the command to autoChooser for selection.
     */
    public void createAllAutoPathCommandsBasedOnPathDirectory() {

        File folder = new File(Filesystem.getDeployDirectory() + "/pathplanner/");
        File[] listOfFiles = folder.listFiles();
        for (File file : listOfFiles) {
            if (file.isFile()) {
                if (file.getName().split("\\.") != null) {
                    PathPlannerTrajectory loadedTrajectory = PathPlanner.loadPath(file.getName().split("\\.")[0],
                            new PathConstraints(3, 3));
                    autoChooser.addOption("A_" + file.getName().split("\\.")[0],
                            new runPathAuto(s_Swerve, loadedTrajectory));
                }
            }
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        // return new exampleAuto(s_Swerve);
        return autoChooser.getSelected();
    }
}
