package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

    // Constants such as camera and target height stored. Change per robot and goal!
    final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(24);
    final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(2);

    // How far from the target we want to be
    final double GOAL_RANGE_METERS = Units.feetToMeters(1);

    // Vision controller setup
    PhotonCamera camera = new PhotonCamera("photonvision");

    // PID constants should be tuned per robot
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.0;
    PIDController forwardController = new PIDController(LINEAR_P, 1.5, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.0;
    PIDController turnController = new PIDController(ANGULAR_P, 1.5, ANGULAR_D);

    //XboxController xboxController = new XboxController(0);
    Joystick stick = new Joystick(0);

    // Drive motors
    WPI_TalonFX leftMotor = new WPI_TalonFX(0); // Change 1 to match the actual CAN ID
    WPI_TalonFX rightMotor = new WPI_TalonFX(1); // Change 2 to match the actual CAN ID
    DifferentialDrive drive = new DifferentialDrive(leftMotor, rightMotor);

    @Override
    public void robotInit() {
        // Configure TalonFX motors for closed-loop control using integrated sensors
        leftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        rightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    }

    @Override
    public void teleopPeriodic() {
        double forwardSpeed;
        double rotationSpeed;

       
            // Vision-alignment mode
            // Query the latest result from PhotonVision
            var result = camera.getLatestResult();

            if (result.hasTargets()) {
                double range =
                PhotonUtils.calculateDistanceToTargetMeters(
                        CAMERA_HEIGHT_METERS,
                        TARGET_HEIGHT_METERS,
                        CAMERA_PITCH_RADIANS,
                        Units.degreesToRadians(result.getBestTarget().getPitch()));


                forwardSpeed = -forwardController.calculate(range, GOAL_RANGE_METERS);
                rotationSpeed = -turnController.calculate(result.getBestTarget().getYaw(), 0);
                
            } else {
                // If we have no targets, stay still.
                forwardSpeed = 0;
                rotationSpeed = 0;
            }
            drive.arcadeDrive(forwardSpeed, rotationSpeed);
        } 

        // Use our forward/turn speeds to control the drivetrain
       
    
}
