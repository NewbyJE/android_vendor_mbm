package com.mbm.mbmservice;

import java.util.ArrayList;

import android.util.Log;

public class ApnInfo {
    private static final String TAG = "MBM_GPS_SERVICE";

    private String apn;
    private String password;
    private String username;
    private String type;
    private String mcc;
    private String mnc;
    private String authtype;
    private String current;

    public ApnInfo(String apn, String username, String password, String mcc, String mnc, String type, String current, String authtype) {
        this.apn = apn.trim();
        this.username = username.trim();
        this.password = password.trim();
        this.mcc = mcc.trim();
        this.mnc = mnc.trim();
        this.type = type.trim();
        this.current = current.trim();
        this.authtype = authtype.trim();
    }

    public String getApn() {
        return apn;
    }

    public String getPassword() {
        return password;
    }

    public String getUsername() {
        return username;
    }

    public String getType() {
        return type;
    }

    public String getMcc() {
        return mcc;
    }

    public String getMnc() {
        return mnc;
    }

    public String getAuthtype() {
        return authtype;
    }

    public String getsCurrent() {
        return current;
    }

    public static ApnInfo getPreferredApn(ArrayList<ApnInfo> apns, OperatorInfo operator) {
        int suplApnId = -1;
        int defaultApnId = -1;

        for(int i=0; i<apns.size(); i++) {
            ApnInfo apn = apns.get(i);
            if(apn.getsCurrent().contains("1")) {
                if(apn.getType().toLowerCase().contains("supl"))
                    suplApnId = i;
                else if(apn.getType().toLowerCase().contains("internet") || 
                        apn.getType().toLowerCase().contains("default") ||
                        apn.getType().toLowerCase().contains("*") ||
                        apn.getType().toLowerCase().length() == 0)
                    defaultApnId = i;
            }
        }

        Log.d(TAG, "Found supl apn, id: " + suplApnId);
        Log.d(TAG, "Found default apn, id: " + defaultApnId);

        if(suplApnId != -1)
            return apns.get(suplApnId);
        else if(defaultApnId != -1)
            return apns.get(defaultApnId);
        else
            return null;
    }

    public String toString() {
        return "Apn: " + apn + "\nType: " + type + "\nUser: " + username + "\nMCC/MNC: " + mcc + "/" + mnc + "\nCurrent: " + current;
    }
}
