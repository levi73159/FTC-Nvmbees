package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.Servo

@Autonomous(name = "Turret Bot")
class Turret : LinearOpMode() {
    override fun runOpMode() {
        val camera: HuskyLens = super.hardwareMap.get(HuskyLens::class.java, "camera");
        val servoX: Servo = super.hardwareMap.get(Servo::class.java, "cannonX");

        camera.resetDeviceConfigurationForOpMode();
        camera.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION)

        waitForStart();
        servoX.position = 0.5;

        while (!super.isStopRequested) {
            super.telemetry.addData("Init", true)
            val blocks = camera.blocks()
            val arrows = camera.arrows();
            for (block in blocks) {
                super.telemetry.addLine(String.format("BLOCK: id: %d, (x, y): (%d, %d), left: %d, top: %d, w: %d, h: %d", block.id, block.x, block.y, block.left, block.top, block.width, block.height))
            }
            for (block in arrows) {
                super.telemetry.addLine(block.toString())
            }
            super.telemetry.update();
        }
    }

}