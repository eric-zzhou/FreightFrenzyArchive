//package org.firstinspires.ftc.teamcode.testing;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.freightFrenzy.AutonomousMethods;
//import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;
//
//@Autonomous(name="BlueTrajectoryTest", group="Linear Opmode")
//@Disabled
//public class BlueTrajectoryTest extends AutonomousMethods {
//    // Declare OpMode members.
//    private ElapsedTime runtime = new ElapsedTime();
//
//    @Override
//    public void runOpMode() {
//        initializeAutonomousAttachments(hardwareMap, telemetry);
//        setLiftMotorPosition(0.6, Constants.armDrive);
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//        runtime.reset();
//
//        // Detect
//
//        sleep(1000); // Spin the duck
//        toTargetDistance(3, Direction.BACKWARD, 0.2, 1000, -1);
//        sleep(1200); // Just for testing
//        encoderStrafeDriveInchesRight(-30, 1);
//        sleep(240);
//        encoderTurn(0, 0.3, 0.6);
//        sleep(240);
//        toTargetDistance(18, Direction.BACKWARD, 0.6, 6000, -1);
//        sleep(1200); // Place the cube
//        encoderTurn(90, 0.6, 0.6);
//        sleep (240);
////        toTarget(6, 30, Direction.FORWARD, Direction.LEFT, -1, 30, 1, 100000);
////        myRobot.runArmMotor(0);
//    }
//}
