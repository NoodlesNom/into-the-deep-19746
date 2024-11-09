package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

public class BotLog {

    public static int LOGNONE = 0;
    public static int LOGDEBUG = 1;
    public static int LOGVERBOSE = 2;
    public static int LOGINFO = 4;
    public static int LOGWARN = 8;

    public static int LOGLEVEL = LOGDEBUG;

    public static void logD(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGDEBUG)!=0) {
            Log.d("Lgr_"+tag, String.format(msgFormat, args));
        }
    }

    public static void logV(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGVERBOSE)!=0) {
            Log.v("Lgr_"+tag, String.format(msgFormat, args));
        }
    }

    public static void logI(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGINFO)!=0) {
            Log.i("Lgr_"+tag, String.format(msgFormat, args));
        }
    }

    public static void logW(String tag, String msgFormat, Object... args) {
        if ((LOGLEVEL & LOGWARN)!=0) {
            Log.w("Lgr_"+tag, String.format(msgFormat, args));
        }
    }
}