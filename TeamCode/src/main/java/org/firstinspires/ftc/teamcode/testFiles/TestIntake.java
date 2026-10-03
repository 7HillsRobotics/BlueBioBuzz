package org.firstinspires.ftc.teamcode.testFiles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * OpMode to test the Intake subsystem (Intake roller, Windmill indexer, Feeder).
 */
@SuppressWarnings("unused")
@TeleOp(name = "Test: Intake", group = "Test")
public class TestIntake extends LinearOpMode {

    @Override
    public void runOpMode() {
        Intake intake = new Intake(hardwareMap);

        telemetry.addLine("Intake Test Ready. Press START.");
        telemetry.update();

        waitForStart();

        boolean intakeOn = false;
        boolean windmillOn = false;
        boolean feederOn = false;

        boolean aPrev = false;
        boolean bPrev = false;
        boolean xPrev = false;

        while (opModeIsActive()) {
            // Toggle Intake Roller
            if (gamepad1.a && !aPrev) {
                intakeOn = !intakeOn;
                if (intakeOn) intake.intakeOn();
                else intake.intakeOff();
            }
            aPrev = gamepad1.a;

            // Toggle Windmill Indexer
            if (gamepad1.b && !bPrev) {
                windmillOn = !windmillOn;
                if (windmillOn) intake.indexWindmill();
                else intake.windmillOff();
            }
            bPrev = gamepad1.b;

            // Toggle Feeder
            if (gamepad1.x && !xPrev) {
                feederOn = !feederOn;
                if (feederOn) intake.feedToTurret();
                else intake.feederOff();
            }
            xPrev = gamepad1.x;

            // Trigger override controls
            if (gamepad1.right_trigger > 0.1) {
                intake.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0.1) {
                intake.setFeederPower(gamepad1.left_trigger);
            }

            // Emergency stop
            if (gamepad1.y) {
                intake.stop();
                intakeOn = false;
                windmillOn = false;
                feederOn = false;
            }

            telemetry.addLine("=== INTAKE SUBSYSTEM TEST ===");
            telemetry.addData("Intake Roller (A)", intakeOn ? "ON (0.8)" : "OFF");
            telemetry.addData("Windmill Indexer (B)", windmillOn ? "ON (0.5)" : "OFF");
            telemetry.addData("Feeder Motor (X)", feederOn ? "ON (0.8)" : "OFF");
            telemetry.addData("Right Trigger Intake Power", gamepad1.right_trigger);
            telemetry.addData("Left Trigger Feeder Power", gamepad1.left_trigger);
            telemetry.addLine("\nControls:");
            telemetry.addLine("A: Toggle Intake Roller");
            telemetry.addLine("B: Toggle Windmill Indexer");
            telemetry.addLine("X: Toggle Feeder");
            telemetry.addLine("Y: Stop All");
            telemetry.update();
        }

        intake.stop();
    }
}
