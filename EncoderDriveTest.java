package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.freightFrenzy.AutonomousMethods;
import org.firstinspires.ftc.teamcode.freightFrenzy.Constants;

@Autonomous(name="EncoderDriveTest", group="Linear Opmode")
@Disabled
public class EncoderDriveTest extends AutonomousMethods {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    double speed = 1;

    @Override
    public void runOpMode() {
        initializeAutonomousDrivetrain(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


//        encoderStrafeDriveInchesRight(24, 1);
//        encoderStraightDrive(24, 0.9);
        encoderTurn(90, 0.6, 1);
        sleep(2400);
        encoderTurn(270, 0.6, 1);
    }
}
