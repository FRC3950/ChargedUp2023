






package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.autos.AutoBalancePIDCommand;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;

    public boolean isInInfoMode = false; //Should include this on all subsystems as a quick-toggle to SD stuff 

    //SmartDashBoard
    double angleToTurn = 0.0;
    double horizontalSpeed_SD = 0.5;
    

    //Might Consider Paramterizing with SmartDashboard
    public Command turnToZeroCommand(){

       return this.run(() -> this.drive(new Translation2d(), SmartDashboard.getNumber("angleToTurn", 0), true, true));
    }

    public Command driveHorizontalCommand(){

        return this.runEnd(()-> this.driveHorizontal(SmartDashboard.getNumber("horizontalSpeed_SD", 0.25)), ()-> this.driveHorizontal(0.0));
    }
    

    public SequentialCommandGroup driveAndAutoAlignOnBeam(){

        //might need the first 2 to be racing each other.
        return new SequentialCommandGroup(

        new ParallelRaceGroup(
            new InstantCommand( () -> this.driveHorizontal(.25), this),
            new WaitUntilCommand(() -> this.getPitch() > 3 || this.getPitch() < -3).withTimeout(4)
        ),
           
            new AutoBalancePIDCommand(this)

        );
    }
    

    public Swerve() {
        gyro = new Pigeon2(Constants.kSwerve.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();
        SmartDashboard.putNumber("angleToTurn", angleToTurn);
        SmartDashboard.putNumber("horizontalSpeed_SD", horizontalSpeed_SD);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.kSwerve.Mod0.constants),
            new SwerveModule(1, Constants.kSwerve.Mod1.constants),
            new SwerveModule(2, Constants.kSwerve.Mod2.constants),
            new SwerveModule(3, Constants.kSwerve.Mod3.constants)
        };


        //Fix for setting module offsets
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.kSwerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.kSwerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kSwerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void driveHorizontal(double speed){
        SwerveModuleState[] swerveModuleStates = 
            Constants.kSwerve.swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds( speed * Constants.kSwerve.maxSpeed, 0, 0, getYaw())
            );

        for(SwerveModule mod: mSwerveMods)
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kSwerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (Constants.kSwerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){

        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  
        if(isInInfoMode){
            for(SwerveModule mod : mSwerveMods){
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
                SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            }
            SmartDashboard.putNumber("Pitch", getPitch());
            SmartDashboard.putNumber("Swerve: Roll", getRoll());
        }
    }
;}