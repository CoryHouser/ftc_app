package org.firstinspires.ftc.team8745;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by houserm on 1/14/17.
 */

@Autonomous(name = "BeaconShooterAuto8745")

public class BeaconShooterAuto8745 extends LinearOpMode {
    Hardware8745 robot = new Hardware8745();

    int foo;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);
        waitForStart();

        for (int i = 1; i <= 2; i++) {
            robot.waitNSeconds(3);
            //shooterRight.setPower(robot.kShooterEnginePower);
            //shooterLeft.setPower(robot.kShooterEnginePower);
            robot.shooterServo.setPosition((robot.kServoNullPosition + (-robot.kServoRange)));
            robot.waitNSeconds(1);
            robot.shooterServo.setPosition(robot.kServoNullPosition + robot.kServoRange);
            robot.waitNSeconds(1);
            //shooterRight.setPower(0);
          //  shooterLeft.setPower(0);
          //  ball_pickup.setPower(1);
        }
       // shooterRight.setPower(0);
      //  shooterLeft.setPower(0);
       // ball_pickup.setPower(0);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (super.opModeIsActive()) {
            //Move to center
            robot.moveToPosition(-48, -.2);

            robot.resetEncoders();
            //Turn to beacon
            robot.turnIMU(-90);

            robot.resetEncoders();
            //Move to beacon
            robot.moveToPosition(-48, -.4);

            robot.resetEncoders();

            robot.moveToPosition(1, .1);


            telemetry.addLine("Made it to loop");
            telemetry.update();
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.waitNSeconds(1);

            int colors = robot.Color_Case();
            telemetry.addData("colors", colors);
            telemetry.addData("CSensorL Red", robot.CSensorL.red());
            telemetry.addData("CSensorL Blue", robot.CSensorL.blue());
            telemetry.addData("CSensorR Red", robot.CSensorR.red());
            telemetry.addData("CSensorR Blue", robot.CSensorR.blue());
            telemetry.update();
            if (colors == 01 || colors == 10 || colors == 11 || colors == 12 || colors == 21) {

                robot.moveToPosition(15, .4);

            } else {
                robot.waitNSeconds(5);
                robot.moveToPosition(-2, .1);
                robot.moveToPosition(15, .4);
            }
            robot.moveToPosition(-48, -.2);

            robot.resetEncoders();
            //Turn to beacon
            robot.turnIMU(-90);

            robot.resetEncoders();
            //Move to beacon
            robot.moveToPosition(-48, -.4);

            robot.resetEncoders();

            robot.moveToPosition(1, .1);


            telemetry.addLine("Made it to loop");
            telemetry.update();
            robot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.waitNSeconds(1);

            if (colors == 01 || colors == 10 || colors == 11 || colors == 12 || colors == 21) {

                robot.moveToPosition(10, .4);

                return;

            } else {
                robot.waitNSeconds(5);
                robot.moveToPosition(-2, .1);
                robot.moveToPosition(15, .4);
            }
        }
    }
}