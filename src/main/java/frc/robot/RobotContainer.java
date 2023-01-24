package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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

    ArrayList<String> trajectoryPathArrayList = new ArrayList<String>();
    Trajectory trajectory = new Trajectory();

    PathPlannerTrajectory examplePath = PathPlanner.loadPath("square", new PathConstraints(3, 3));
//PP_Test_1_CircleStation
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

    /* Commands */
    private final AutoBalanceCommand balanceCommand = new AutoBalanceCommand(s_Swerve);
    private final exampleAuto exampleAuto = new exampleAuto(s_Swerve);
    private final runPathAuto ppExampleAuto = new runPathAuto(s_Swerve, examplePath);
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {


        try {

            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/TwoCone_DOCKED.wpilib.json");
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            autoChooser.addOption("A_TwoCone", new runPathAuto(s_Swerve, trajectory));

        } catch (IOException ex) {

            DriverStation.reportError("Unable to open trajectory: ", ex.getStackTrace());

        }

        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean()));

        // Configure the button bindings
        configureButtonBindings();

        // Autochooser
       // createAllAutoPathCommandsBasedOnPathDirectory();
        autoChooser.addOption("a_Original Test Auto", exampleAuto);
        autoChooser.addOption("a_Patrick Auto that goes straight", ppExampleAuto);
        SmartDashboard.putData("Auto Selection", autoChooser);

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
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        new JoystickButton(driver, XboxController.Button.kB.value) // Should eventually do all buttons like this?
                .whileTrue(balanceCommand);
    }

    /**
     * This method creates a {@link runPathAuto} command for each saved path and
     * adds the command to autoChooser for selection.
     */
    public void createAllAutoPathCommandsBasedOnPathDirectory() {
        File folder = new File("src/main/deploy/paths");
        File[] listOfFiles = folder.listFiles();
        for (File file : listOfFiles) {
            if (file.isFile()) {

                try {

                    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + file.getName());
                    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                    autoChooser.addOption("A_" + file.getName().split("\\.")[0], new runPathAuto(s_Swerve, trajectory));

                } catch (IOException ex) {

                    DriverStation.reportError("Unable to open trajectory: " + file.getName(), ex.getStackTrace());

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
