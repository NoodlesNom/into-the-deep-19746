package org.firstinspires.ftc.teamcode.autonomous.gf;

import org.firstinspires.ftc.teamcode.util.CSVWritable;
import org.firstinspires.ftc.teamcode.util.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
    double distance(final S other);

    boolean equals(final Object other);

    String toString();

    String toCSV();
}
