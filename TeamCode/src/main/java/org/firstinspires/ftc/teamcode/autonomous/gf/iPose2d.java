package org.firstinspires.ftc.teamcode.autonomous.gf;

public interface iPose2d<S> extends iRotation2d<S>, iTranslation2d<S> {
    public Pose2d getPose();

    public S transformBy(Pose2d transform);

    public S mirror();
}
