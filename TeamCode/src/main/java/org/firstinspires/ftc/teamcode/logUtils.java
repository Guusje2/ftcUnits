package org.firstinspires.ftc.teamcode;


import android.os.Environment;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Calendar;
import java.util.HashMap;

/**
 * Created by guusd on 7/20/2018.
 * for logging in the ftc to a .csv file
 */

public class logUtils {

    /** the type of logmessage, used for sorting in a spreadsheet editor*/
    public  enum logType {
        normal,
        warning,
        error
    };
    /**name of the FTC team, used in the file namesw */
    static public String teamname = "FTCUnits";
    FileOutputStream fOUT = null;
    /**the list of filewriters, can be seen as the list of logging channels */
    static public HashMap<Integer, FileWriter> filewriters;

    public static void StartLogging(Integer id) throws IOException {
        String FileName =  teamname + Calendar.getInstance().getTime().toString() + ".csv";
        File file = new File(getPublicAlbumStorageDir(teamname),FileName );
        if(filewriters == null){
            filewriters = new HashMap<Integer, FileWriter>();
        }
        filewriters.put(id,new FileWriter(file));
    }

    /**sets the FTC team name, used in the file name */
    public static void setTeamname(String _teamname){
        teamname = _teamname;
    }

    /**gets the public image folder */
    public  static File getPublicAlbumStorageDir(String albumName) {
        // Get the directory for the user's public pictures directory.
        File file = new File(Environment.getExternalStoragePublicDirectory(
                Environment.DIRECTORY_DOCUMENTS), albumName);
        if (!file.mkdirs()) {

        }
        return file;
    }


    /**closes the filewriter given by the id, do this to make sure your data gets saved */
    public static void StopLogging(int id) {
        try {
            filewriters.get(id).close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /** log your data to the filewriter given by the ID */
    public static void Log(logType type, String message, int id) {

        try {
            filewriters.get(id).write("\n"+ type.toString() + "," + Calendar.getInstance().getTime() + "," + message);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

}
