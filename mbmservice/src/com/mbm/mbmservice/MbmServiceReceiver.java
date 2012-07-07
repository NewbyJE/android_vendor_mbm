package com.mbm.mbmservice;

import java.util.ArrayList;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.database.Cursor;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.net.Uri;
import android.provider.Settings;
import android.telephony.TelephonyManager;
import android.util.Log;

public class MbmServiceReceiver extends BroadcastReceiver {
    private static final String TAG = "MBM_GPS_SERVICE";

    private OperatorInfo currentOperator;

    public MbmServiceReceiver() {
        super();
        currentOperator = new OperatorInfo("", "", "");
    }

    private void updateOperatorInfo(TelephonyManager tm) {
        String networkOperator = tm.getNetworkOperator();
        String networkOperatorName = tm.getNetworkOperatorName();
        boolean roaming = tm.isNetworkRoaming();
        if (networkOperator != null && networkOperator.length() >= 5) {
            Log.d(TAG, "NetworkOperator: " + networkOperator);
            String oMcc = networkOperator.substring(0, 3);
            String oMnc = networkOperator.substring(3);
            currentOperator.setName(networkOperatorName);
            currentOperator.setMcc(oMcc);
            currentOperator.setMnc(oMnc);

            MbmService.getCurrentStatus().setOperatorInfo(currentOperator);
        }
    }

    public void onCellLocationChanged(TelephonyManager tm) {
        Log.d(TAG, "onCellLocationChanged");
        updateOperatorInfo(tm);
    }

    @Override
    public void onReceive(Context context, Intent intent) {
        Log.d(TAG, "MBM onReceive");
        String id = "";
        String apn = "";
        String password = "";
        String name = "";
        String username = "";
        String type = "";
        String mcc = "";
        String mnc = "";
        String authtype = "";
        String current = "";

        Status currentStatus = MbmService.getCurrentStatus();

        if (intent.getAction().equals(Intent.ACTION_BOOT_COMPLETED)) {
            Log.d(TAG, "BOOT COMPLETED");
            Intent i = new Intent();
            i.setAction("com.mbm.mbmservice.MbmService");
            context.startService(i);
        } else if (intent.getAction().equals(
                "android.location.PROVIDERS_CHANGED")) {
            Log.d(TAG, "Location providers changed");
            MbmService.onLocationProviderChanged();
        } else if (intent.getAction().equals(
                Intent.ACTION_AIRPLANE_MODE_CHANGED)) {
            Object state = intent.getExtras().get("state");
            currentStatus.setAirplaneMode(state);
        } else if (intent.getAction().equals(
                ConnectivityManager.CONNECTIVITY_ACTION)) {
            NetworkInfo info = intent
                    .getParcelableExtra(ConnectivityManager.EXTRA_NETWORK_INFO);
            NetworkInfo extrainfo = intent
                    .getParcelableExtra(ConnectivityManager.EXTRA_OTHER_NETWORK_INFO);
            boolean no_connection = intent.getBooleanExtra(
                    ConnectivityManager.EXTRA_NO_CONNECTIVITY, false);

            currentStatus.setNetworkInfo(info);
            currentStatus.setExtraNetworkInfo(extrainfo);
            currentStatus.setNoConnectivity(no_connection);
        } else if (intent.getAction().equals(
                ConnectivityManager.ACTION_BACKGROUND_DATA_SETTING_CHANGED)) {
            ConnectivityManager cm = (ConnectivityManager) context
                    .getSystemService(Context.CONNECTIVITY_SERVICE);
            boolean backgroundData = cm.getBackgroundDataSetting();

            currentStatus.setBackgroundDataSetting(backgroundData);
        } else if (intent.getAction().equals(
                "android.intent.action.ANY_DATA_STATE")) {
            Object state = intent.getExtras().get("state");
            if (intent.getExtras().get("state") != null) {
                ConnectivityManager cm = (ConnectivityManager) context
                        .getSystemService(Context.CONNECTIVITY_SERVICE);
                boolean backgroundData = cm.getBackgroundDataSetting();
                currentStatus.setBackgroundDataSetting(backgroundData);

                boolean mobileDataAllowed = Settings.Secure.getInt(
                        context.getContentResolver(), "mobile_data", 1) == 1;
                currentStatus.setMobileDataAllowed(mobileDataAllowed);

                boolean roamingAllowed = Settings.Secure.getInt(
                        context.getContentResolver(),
                        Settings.Secure.DATA_ROAMING, 1) == 1;
                currentStatus.setRoamingAllowed(roamingAllowed);

                TelephonyManager tm = (TelephonyManager) context
                        .getSystemService(Context.TELEPHONY_SERVICE);
                updateOperatorInfo(tm);

                if (state.equals("CONNECTED") || state.equals("DISCONNECTED")) {
                    currentStatus.setDataState(state.toString().toLowerCase());

                    ArrayList<ApnInfo> apns = new ArrayList<ApnInfo>(5);
                    Cursor mCursor = context.getContentResolver().query(
                            Uri.parse("content://telephony/carriers"), null,
                            null, null, null);
                    if (mCursor != null) {
                        while (mCursor != null && mCursor.moveToNext()) {
                            id = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("_id"));
                            name = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("name"));
                            apn = ""
                                    + mCursor.getString(
                                            mCursor.getColumnIndex("apn"))
                                            .toLowerCase();
                            username = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("user"));
                            password = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("password"));
                            mcc = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("mcc"));
                            mnc = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("mnc"));
                            authtype = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("authtype"));
                            type = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("type"));
                            current = ""
                                    + mCursor.getString(mCursor
                                            .getColumnIndex("current"));
                            apns.add(new ApnInfo(apn, username, password, mcc,
                                    mnc, type, current, authtype));
                        }
                        mCursor.close();
                        ApnInfo pApn = ApnInfo.getPreferredApn(apns,
                                currentOperator);
                        currentStatus.setApnInfo(pApn);
                    } else {
                        currentStatus.setApnInfo(null);
                    }
                }
            }
        } else if (intent.getAction().equals(
                TelephonyManager.ACTION_PHONE_STATE_CHANGED)) {
            Log.d(TAG, "phone state changed");

            ConnectivityManager cm = (ConnectivityManager) context
                    .getSystemService(Context.CONNECTIVITY_SERVICE);
            boolean backgroundData = cm.getBackgroundDataSetting();
            currentStatus.setBackgroundDataSetting(backgroundData);

            boolean mobileDataAllowed = Settings.Secure.getInt(
                    context.getContentResolver(), "mobile_data", 1) == 1;
            currentStatus.setMobileDataAllowed(mobileDataAllowed);

            boolean roamingAllowed = Settings.Secure.getInt(
                    context.getContentResolver(), Settings.Secure.DATA_ROAMING,
                    1) == 1;
            currentStatus.setRoamingAllowed(roamingAllowed);
        } else if (intent.getAction().equals(Intent.ACTION_TIME_CHANGED)) {
            currentStatus.setTime(currentStatus.getCurrentTime());
        }
    }
}
