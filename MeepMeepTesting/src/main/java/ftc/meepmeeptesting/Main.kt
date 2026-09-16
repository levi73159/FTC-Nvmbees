package ftc.meepmeeptesting

import com.acmerobotics.roadrunner.InstantFunction
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.SequentialAction
import com.acmerobotics.roadrunner.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import java.util.Vector
import kotlin.math.PI


enum class TeamColor { RED, BLUE, }
val teamcolor = TeamColor.BLUE

fun toSided(pose2d: Pose2d): Pose2d {
    val fliped = pose2d.position.y * if (teamcolor == TeamColor.RED) 1.0 else -1.0
    val otherHeading =  if (teamcolor == TeamColor.RED) pose2d.heading.toDouble() else Rotation2d.fromDouble(2 * PI) - pose2d.heading
    return Pose2d(Vector2d(pose2d.position.x, fliped), otherHeading)
}

fun toSided(x: Double): Double {
    return if (teamcolor == TeamColor.RED) x else x * -1.0
}

fun toSided(color: TeamColor, x: Double): Double {
    return if (color == TeamColor.RED) x else x * -1.0
}

fun toSidedDeg(color: TeamColor, x: Double): Double {
    return if (color == TeamColor.RED) x else 360 - x;
}

fun main(args: Array<String>) {
    System.setProperty("sun.java2d.opengl", "true");
    val meepMeep = MeepMeep(800)

    val beginPosRed = Pose2d(Vector2d(61.6, 18.2), Math.toRadians(180.0))
    val beginPosBlue = Pose2d(Vector2d(61.6, -18.2), Math.toRadians(180.0))

    val beginTowerRed = Pose2d(Vector2d(-53.5, 54.5), Math.toRadians(135.0))
    val beginTowerBlue = toSided(beginTowerRed)

    val redPos = Pose2d(Vector2d(-53.5+25.0, 54.5-25.0), Math.toRadians(135.0+10.0))
    val bluePos = toSided(redPos)


    val dif = 22.2
    val heading = Math.toRadians(-270.0)
    val ball = Vector2d(-11.2, 24.0)
    val ballOrder = 0

    val ballPos = Vector2d(ball.x + (dif * ballOrder), ball.y)
    val otherPos = Vector2d(ball.x + (dif * if (ballOrder == 2) 0 else ballOrder + 1), ball.y)

    val ball1 = toSided(Pose2d(ballPos, heading))
    val ball2 = toSided(Pose2d(otherPos, heading))

    val pos = if (teamcolor == TeamColor.RED) redPos else bluePos
    val beginPos = when (true) {
        true -> if (teamcolor == TeamColor.RED) beginTowerRed else beginTowerBlue
        false -> if (teamcolor == TeamColor.RED) beginPosRed else beginPosBlue
    }
    val myBot =
        DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 60.0, Math.toRadians(180.0), Math.toRadians(180.0), 15.0)
            .setStartPose(beginPos)
            .build()

    val drive = myBot.drive

    val toTower = drive.actionBuilder(beginPos)
        .splineToConstantHeading(
            pos.position,
            tangent = Math.toRadians(toSided(175.0))
        )
        .turnTo(pos.heading)
        .waitSeconds(5.0)
        .build()

    val toTowerAtTower = drive.actionBuilder(beginPos)
        .strafeToSplineHeading(
            pos.position,
            pos.heading,
        )
        .waitSeconds(5.0)
        .build()

    val ballPath = drive.actionBuilder(pos)
        .strafeToSplineHeading(ball1.position, ball1.heading)
        .strafeTo(Vector2d(ball1.position.x, ball1.position.y + toSided(30.0)))
        .build()

    val otherBall = drive.actionBuilder(pos)
        .strafeToSplineHeading(ball2.position, ball1.heading)
        .strafeTo(Vector2d(ball2.position.x, ball1.position.y + toSided(30.0)))
        .build()

    val backToTower = drive.actionBuilder(Pose2d(ball1.position + Vector2d(0.0, toSided(30.0)), ball1.heading))
        .setReversed(true)
        .splineToLinearHeading(
            pos,
            PI,
        )
        .waitSeconds(1.6)
        .build()

    val backToTower2 = drive.actionBuilder(Pose2d(ball2.position + Vector2d(0.0, toSided(30.0)), ball2.heading))
        .setReversed(true)
        .splineToLinearHeading(
            pos,
            PI,
        )
        .waitSeconds(1.6)
        .build()

    val moveAway = drive.actionBuilder(pos).strafeTo(Vector2d(pos.position.x - 10.0, pos.position.y - toSided(10.0))).build()

    myBot.runAction(SequentialAction(toTowerAtTower, ballPath, backToTower, otherBall, backToTower2, moveAway))


    meepMeep.setBackground(Background.FIELD_DECODE_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(myBot)
        .start()
}