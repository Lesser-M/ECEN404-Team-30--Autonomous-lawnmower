package lawnmower.app;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;

import androidx.core.content.ContextCompat;

public class PermissionManager {

    private static PermissionManager sInstance = null;
    private PermissionManager() {}

    public static PermissionManager getInstance() {
        if (sInstance == null) {
            sInstance = new PermissionManager();
        }
        return sInstance;
    }

    public String[] getAppPermissions() {
        return new String[] {
                Manifest.permission.ACCESS_COARSE_LOCATION,
                Manifest.permission.ACCESS_FINE_LOCATION
        };
    }

    public boolean hasPermissions(Context context) {
        String[] permissions = getAppPermissions();
        for (String permission: permissions) {
            if (ContextCompat.checkSelfPermission(context, permission) == PackageManager.PERMISSION_DENIED) return false;
        }
        return true;
    }
}
