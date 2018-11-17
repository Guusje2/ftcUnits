package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.os.Environment;
import android.widget.Toast;

import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.sql.Time;
import java.util.Calendar;
import java.util.HashMap;

/**
 * Created by guusd on 7/20/2018.
 * for logging in the ftc to a text file
 */

public class logUtils {

    public static enum logType {
        normal,
        warning,
        error
    };
    static public String teamname = "FTCUnits";
    FileOutputStream fOUT = null;
    static public HashMap<Integer, FileWriter> filewriters;
    static public boolean isOpModeRunning = false;

    public static void StartLogging(Integer id) throws IOException {
        String FileName =  teamname + Calendar.getInstance().getTime().toString() + ".csv";
        File file = new File(getPublicAlbumStorageDir(teamname),FileName );
        if(filewriters == null){
        filewriters = new HashMap<Integer, FileWriter>();
        }
        filewriters. put(id,new FileWriter(file));
    }

    public static void setTeamname(String _teamname){
        teamname = _teamname;
    }
    public  static File getPublicAlbumStorageDir(String albumName) {
        // Get the directory for the user's public pictures directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), albumName);
        if (!file.mkdirs()) {

        }
        return file;
    }

    public static void StopLogging(int id) {
        try {
            filewriters.get(id).close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void Log(logType type, String message, int id) {

        try {
            filewriters.get(id).write("\n"+ type.toString() + "," + Calendar.getInstance().getTime() + "," + message);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
