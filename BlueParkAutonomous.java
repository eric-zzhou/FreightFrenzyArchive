package org.firstinspires.ftc.teamcode.freightFrenzy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Blue Park", group = "Linear Opmode")

public class BlueParkAutonomous extends AutonomousMethods {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        initializeAutonomousAttachments(hardwareMap, telemetry);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        resetArmGap();
        sleep(120);
        park(false, true, true);

        AutoTransitioner.transitionOnStop(this, "Blue TeleOp");
    }
}
