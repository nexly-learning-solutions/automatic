package com.yourdomain.company.aimyhome;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothServerSocket;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.os.Handler;
import android.os.Message;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.content.ContextCompat;

import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;
import android.view.animation.TranslateAnimation;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.LinearLayout;
import android.widget.ListView;
import android.widget.TextView;

import com.suke.widget.SwitchButton;
import com.yourdomain.company.aimyhome.Fragment.FanSpeed;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;
import java.util.Set;
import java.util.UUID;

import jp.hamcheesedev.outlinedtextview.CompatOutlinedTextView;
import me.itangqi.waveloadingview.WaveLoadingView;

public class MainActivity extends AppCompatActivity {

    ListView listView;
    int int_value = 0;
    String rev_value = null;
    BluetoothAdapter bluetoothAdapter;
    BluetoothDevice[] btArray;
    SendReceive sendReceive;
    View menuView;
    WaveLoadingView mWaveLoadingView;
    LinearLayout connect, fail, connecting;
    boolean isUp;
    static final int STATE_LISTENING = 1;
    static final int STATE_CONNECTING=2;
    static final int STATE_CONNECTED=3;
    static final int STATE_CONNECTION_FAILED=4;
    static final int STATE_MESSAGE_RECEIVED=5;
    int REQUEST_ENABLE_BLUETOOTH=1;
    private static final String APP_NAME = "MyHome";
    private static final UUID MY_UUID=UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    String action;
    int check_connect = 0;
    int back_press = 0;
    LinearLayout notify_on, notify_off, power_on, power_off, ai_off, ai_on, sensor_on, sensor_off;
    TextView temp, fahren, humidity;
    String fan_value = "0";
    CompatOutlinedTextView fan_speed;
    com.suke.widget.SwitchButton light, fan, wifi, tv, door_lock, air;
    int switch_control_value = 0;
    TextView recv, date_view, day_view;
    TextView light_text, tv_text, wifi_text, air_text, door_text;
    int switch_off_value = 0;
    int fan_change_value = 0;
    int light_tt = 0, wifi_tt = 0, tv_tt = 0, air_tt = 0, door_tt = 0, fan_tt = 0;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_LIGHT_STATUS_BAR);
        getWindow().setStatusBarColor(ContextCompat.getColor(MainActivity.this,R.color.colorPrimary));

        IntentFilter filter = new IntentFilter();
        filter.addAction(BluetoothDevice.ACTION_ACL_CONNECTED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECT_REQUESTED);
        filter.addAction(BluetoothDevice.ACTION_ACL_DISCONNECTED);
        this.registerReceiver(broadcastReceiver, filter);

        mWaveLoadingView = (WaveLoadingView) findViewById(R.id.waveLoadingView);

        connect = (LinearLayout) findViewById(R.id.connect);
        fail = (LinearLayout) findViewById(R.id.fail);
        connecting = (LinearLayout) findViewById(R.id.connecting);

        notify_off = (LinearLayout)findViewById(R.id.notification_off);
        notify_on = (LinearLayout)findViewById(R.id.notification_on);
        power_on = (LinearLayout)findViewById(R.id.power_on);
        power_off = (LinearLayout)findViewById(R.id.power_off);
        ai_off = (LinearLayout)findViewById(R.id.ai_off);
        ai_on = (LinearLayout)findViewById(R.id.ai_on);
        sensor_off = (LinearLayout)findViewById(R.id.sensor_off);
        sensor_on = (LinearLayout)findViewById(R.id.sensor_on);

        temp = (TextView)findViewById(R.id.temp);
        fan_speed = (CompatOutlinedTextView) findViewById(R.id.fan_speed);
        fahren = (TextView)findViewById(R.id.fahren);
        humidity = (TextView)findViewById(R.id.humidity);
        recv = (TextView)findViewById(R.id.recv);
        date_view = (TextView)findViewById(R.id.mounth_value);
        day_view = (TextView)findViewById(R.id.day_value);

        light_text = (TextView)findViewById(R.id.light_text);
        wifi_text = (TextView)findViewById(R.id.wifi_text);
        tv_text = (TextView)findViewById(R.id.tv_text);
        air_text = (TextView)findViewById(R.id.air_text);
        door_text = (TextView)findViewById(R.id.door_text);

        light = (com.suke.widget.SwitchButton) findViewById(R.id.switch_light);
        fan = (com.suke.widget.SwitchButton) findViewById(R.id.switch_fan);
        wifi = (com.suke.widget.SwitchButton) findViewById(R.id.switch_wifi);
        tv = (com.suke.widget.SwitchButton) findViewById(R.id.switch_tv);
        door_lock = (com.suke.widget.SwitchButton) findViewById(R.id.switch_door);
        air = (com.suke.widget.SwitchButton) findViewById(R.id.switch_air);


        date_formate();

        // on click methode for menu function

        fan_speed.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                FanSpeed fanSpeed = new FanSpeed();
                fanSpeed.show(getSupportFragmentManager(),"FanDialogBox");
            }
        });

        notify_off.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                notify_on.setVisibility(View.VISIBLE);
                notify_off.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                        String string = "NOTIFI ON";
                        sendReceive.write(string.getBytes());
                }
            }
        });

        notify_on.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                notify_on.setVisibility(View.GONE);
                notify_off.setVisibility(View.VISIBLE);
                if (check_connect == 5)
                {

                    String string = "NOTIFI OFF";
                    sendReceive.write(string.getBytes());

                }
            }
        });

        power_on.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                switch_off_value = 5;

                power_on.setVisibility(View.GONE);
                power_off.setVisibility(View.VISIBLE);

                light_text.setText("OFF");
                wifi_text.setText("OFF");
                tv_text.setText("OFF");
                air_text.setText("OFF");
                door_text.setText("OFF");

                light.setChecked(false);
                fan.setChecked(false);
                wifi.setChecked(false);
                tv.setChecked(false);
                air.setChecked(false);
                door_lock.setChecked(false);

                fan_value = "0";
                fan_speed.setText(fan_value + "%");
                int i = Integer.valueOf(fan_value);
                mWaveLoadingView.setProgressValue(i);

                if (check_connect == 5)
                {
                    String string = "POWER OFF";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        power_off.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {


                power_on.setVisibility(View.VISIBLE);
                power_off.setVisibility(View.GONE);

                if (check_connect == 5)
                {
                    String string = "POWER ON";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        ai_off.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                ai_on.setVisibility(View.VISIBLE);
                ai_off.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                    String string = "AI ON";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        ai_on.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                ai_off.setVisibility(View.VISIBLE);
                ai_on.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                    String string = "AI OFF";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        sensor_on.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                sensor_off.setVisibility(View.VISIBLE);
                sensor_on.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                    String string = "SENSOR OFF";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        sensor_off.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {

                sensor_on.setVisibility(View.VISIBLE);
                sensor_off.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                    String string = "SENSOR ON";
                    sendReceive.write(string.getBytes());
                }
            }
        });

        findViewByIdes();

        bluetoothAdapter=BluetoothAdapter.getDefaultAdapter();

        if(!bluetoothAdapter.isEnabled())
        {
            Intent enableIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableIntent,REQUEST_ENABLE_BLUETOOTH);
        }

        menuView = findViewById(R.id.my_menu_view);
        menuView.setVisibility(View.INVISIBLE);
        isUp = false;

        implementListeners();


        light.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {


                if (view.isChecked())
                {
                    switch_off_value = 0;
                    light_text.setText("ON");
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);
                if (check_connect == 5)
                {
                    if (light_tt == 0){
                        String string = "LIGHT ON";
                        sendReceive.write(string.getBytes());
                    }

                }

                }else {
                    light_text.setText("OFF");
                    if (switch_off_value == 0)
                    {
                        if (check_connect == 5)
                        {
                            if (light_tt == 0)
                            {
                                String string = "LIGHT OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }
                    }
                }
            }
        });

        fan.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {

                if (view.isChecked())
                {
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);

                    if (fan_tt == 0) {

                        switch_off_value = 0;

                        if (switch_control_value == 0) {
                            if (fan_change_value == 0) {
                                fan_value = "70";
                                fan_speed.setText(fan_value + "%");
                                int i = Integer.valueOf(fan_value);
                                mWaveLoadingView.setProgressValue(i);
                            } else {

                                int i = Integer.valueOf(fan_value);
                                mWaveLoadingView.setProgressValue(i);
                            }
                        }

                        if (check_connect == 5) {
                            if (fan_change_value == 0) {
                                fan_value = "70";
                                fansend_value();
                            }

                        }

                    }

                }else {

                    if (fan_tt == 0) {

                        fan_change_value = 0;
                        fan_value = "0";
                        fan_speed.setText("0%");
                        int i = Integer.valueOf(fan_value);
                        mWaveLoadingView.setProgressValue(i);
                        switch_control_value = 0;
                        if (switch_off_value == 0) {
                            if (check_connect == 5) {
                                String string = "FAN OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }

                    }
                }
            }
        });

        wifi.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {

                if (view.isChecked())
                {
                    switch_off_value = 0;
                    wifi_text.setText("ON");
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);
                    if (check_connect == 5)
                    {
                        if (wifi_tt == 0)
                        {
                            String string = "WIFI ON";
                            sendReceive.write(string.getBytes());
                        }
                    }

                }else {
                    wifi_text.setText("OFF");
                    if (switch_off_value == 0)
                    {
                        if (check_connect == 5)
                        {
                            if (wifi_tt == 0)
                            {
                                String string = "WIFI OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }
                    }
                }
            }
        });

        tv.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {

                if (view.isChecked())
                {
                    switch_off_value = 0;
                    tv_text.setText("ON");
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);
                    if (check_connect == 5)
                    {
                        if (tv_tt == 0)
                        {
                            String string = "TV ON";
                            sendReceive.write(string.getBytes());
                        }
                    }

                }else {
                    tv_text.setText("OFF");
                    if (switch_off_value == 0)
                    {
                        if (check_connect == 5)
                        {
                            if (tv_tt == 0)
                            {
                                String string = "TV OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }
                    }
                }
            }
        });

        air.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {

                if (view.isChecked())
                {
                    switch_off_value = 0;
                    air_text.setText("ON");
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);
                    if (check_connect == 5)
                    {
                        if (air_tt == 0)
                        {
                            String string = "AIR ON";
                            sendReceive.write(string.getBytes());
                        }
                    }

                }else {
                    air_text.setText("OFF");
                    if (switch_off_value == 0)
                    {
                        if (check_connect == 5)
                        {
                            if (air_tt == 0)
                            {
                                String string = "AIR OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }
                    }
                }
            }
        });

        door_lock.setOnCheckedChangeListener(new SwitchButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(SwitchButton view, boolean isChecked) {

                if (view.isChecked())
                {
                    switch_off_value = 0;
                    door_text.setText("ON");
                    power_on.setVisibility(View.VISIBLE);
                    power_off.setVisibility(View.GONE);
                    if (check_connect == 5)
                    {
                        if (door_tt == 0)
                        {
                            String string = "DOOR ON";
                            sendReceive.write(string.getBytes());
                        }
                    }

                }else {
                    door_text.setText("OFF");
                    if (switch_off_value == 0)
                    {
                        if (check_connect == 5)
                        {
                            if (door_tt == 0)
                            {
                                String string = "DOOR OFF";
                                sendReceive.write(string.getBytes());
                            }
                        }
                    }
                }
            }
        });



    }

    private void date_formate() {

        String day = new SimpleDateFormat("EEEE", Locale.getDefault()).format(new Date());
        String date = new SimpleDateFormat("d MMM, yyyy", Locale.getDefault()).format(new Date());
        date_view.setText(date);
        day_view.setText(day);
    }

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
        BluetoothDevice device;
        @Override
        public void onReceive(Context context, Intent intent) {
            action = intent.getAction();
            device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
            if (BluetoothDevice.ACTION_ACL_CONNECTED.equals(action)) {
                check_connect = 5;
                connect.setVisibility(View.VISIBLE);
                fail.setVisibility(View.GONE);
                connecting.setVisibility(View.GONE);
                firstConnect();
            } else if (BluetoothDevice.ACTION_ACL_DISCONNECTED.equals(action)) {
                check_connect = 0;
                connect.setVisibility(View.GONE);
                connecting.setVisibility(View.GONE);
                fail.setVisibility(View.VISIBLE);
            }
        }
    };

    private void firstConnect() {

        Handler handler=new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {


                if (check_connect == 5)
                {
                    String string = "START DEVICE";
                    sendReceive.write(string.getBytes());
                }

            }
        },1000);

    }

    /////////////

    private void implementListeners() {


        listView.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> adapterView, View view, int i, long l) {
                ClientClass clientClass=new ClientClass(btArray[i]);
                clientClass.start();

                if (isUp) {
                    slideDown_menu(menuView);
                } else {
                    slideUp_menu(menuView);
                }
                isUp = !isUp;
            }
        });

    }

    Handler handler=new Handler(new Handler.Callback() {
        @Override
        public boolean handleMessage(Message msg) {

            switch (msg.what)
            {
                case STATE_LISTENING:
                    // code
                    break;
                case STATE_CONNECTING:
                    connect.setVisibility(View.GONE);
                    fail.setVisibility(View.GONE);
                    connecting.setVisibility(View.VISIBLE);
                    break;
                case STATE_CONNECTED:
                    connect.setVisibility(View.VISIBLE);
                    fail.setVisibility(View.GONE);
                    connecting.setVisibility(View.GONE);
                    check_connect = 5;
                    break;
                case STATE_CONNECTION_FAILED:
                    connect.setVisibility(View.GONE);
                    fail.setVisibility(View.VISIBLE);
                    connecting.setVisibility(View.GONE);
                    break;
                case STATE_MESSAGE_RECEIVED:
                    byte[] readBuff= (byte[]) msg.obj;
                    rev_value =new String(readBuff,0,msg.arg1);
                    mcu();
                    break;
            }
            return true;
        }
    });

    private void mcu() {

        int_value = Integer.parseInt(rev_value);

        String recv_value = Integer.toString(int_value);

        if (int_value > 100 & int_value < 200)
        {
            int final_temp = int_value - 100;
            int fahrenheit = (final_temp * 9/5) + 32;
            String precess_item_value = Integer.toString(final_temp);
            String precess_fahren_value = Integer.toString(fahrenheit);
            temp.setText(precess_item_value);
            fahren.setText(precess_fahren_value);

        }

        if (int_value > 200)
        {
            int final_humidity = int_value - 200;
            if (final_humidity < 101)
            {
                String precess_humidity_value = Integer.toString(final_humidity);
                humidity.setText(precess_humidity_value + "%");
            }else {

                humidity.setText("100%");
            }
        }

        // Receive & Check MCU Port

        if (int_value == 310)
        {
            light_tt = 1;
            light.setChecked(true);
        }else if (int_value == 311)
        {
            light_tt = 1;
            light.setChecked(false);
        }

        if (int_value == 312)
        {
            wifi_tt = 1;
            wifi.setChecked(true);
        }else if (int_value == 313)
        {
            wifi_tt = 1;
            wifi.setChecked(false);
        }

        if (int_value == 314)
        {
            tv_tt = 1;
            tv.setChecked(true);
        }else if (int_value == 315)
        {
            tv_tt = 1;
            tv.setChecked(false);
        }

        if (int_value == 316)
        {
            air_tt = 1;
            air.setChecked(true);
        }else if (int_value == 317)
        {
            air_tt = 1;
            air.setChecked(false);
        }

        if (int_value == 318)
        {
            door_tt = 1;
            door_lock.setChecked(true);
        }else if (int_value == 319)
        {
            door_tt = 1;
            door_lock.setChecked(false);
        }


        if (int_value == 320)
        {
            notify_on.setVisibility(View.VISIBLE);
            notify_off.setVisibility(View.GONE);

        }else if (int_value == 321)
        {
            notify_on.setVisibility(View.GONE);
            notify_off.setVisibility(View.VISIBLE);
        }

        if (int_value == 322)
        {
            ai_on.setVisibility(View.VISIBLE);
            ai_off.setVisibility(View.GONE);

        }else if (int_value == 323)
        {
            ai_on.setVisibility(View.GONE);
            ai_off.setVisibility(View.VISIBLE);
        }

        if (int_value == 324)
        {
            sensor_on.setVisibility(View.VISIBLE);
            sensor_off.setVisibility(View.GONE);

        }else if (int_value == 325)
        {
            sensor_on.setVisibility(View.GONE);
            sensor_off.setVisibility(View.VISIBLE);
        }

        // Fan Function

        if (int_value == 340)
        {
            fan_tt = 1;
            fan_value = "0";
            fan.setChecked(false);
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);

        }else if (int_value == 341)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "10";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 342)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "20";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 343)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "30";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 344)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "40";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 345)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "50";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 346)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "60";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 347)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "70";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 348)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "80";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 349)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "90";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }else if (int_value == 350)
        {
            fan_tt = 1;
            fan.setChecked(true);
            fan_value = "100";
            fan_speed.setText(fan_value +"%");
            int i = Integer.valueOf(fan_value);
            mWaveLoadingView.setProgressValue(i);
        }

        if (int_value == 335)
        {
            Handler handler=new Handler();
            handler.postDelayed(new Runnable() {
                @Override
                public void run() {

                    light_tt = 0; wifi_tt = 0; tv_tt = 0; air_tt = 0; door_tt = 0; fan_tt = 0;
                }
            },1000);
        }
    }

    private void findViewByIdes() {
        listView=(ListView) findViewById(R.id.listview);
    }

    public void dispatchInformations(String mesg) {

        fan_change_value = 10;
        fan_value = mesg;
        int i = Integer.valueOf(mesg);
        mWaveLoadingView.setProgressValue(i);
        fan_speed.setText(mesg + "%");
        if (i != 0)
        {
            fan.setChecked(true);
            switch_control_value = 5;
        }

        if (i == 0)
        {
            fan.setChecked(false);
            switch_control_value = 0;
        }

        fansend_value();

    }

    public String getSpeed() {

        return fan_value;

    }

    public void fansend_value ()
    {
        int value_control_fan_speed = Integer.valueOf(fan_value);
        if (check_connect == 5)
        {
            if (value_control_fan_speed ==0)
            {
                String string = "FAN OFF";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 10 && value_control_fan_speed !=0 || value_control_fan_speed == 10)
            {
                String string = "ONE";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 20 && value_control_fan_speed > 10 || value_control_fan_speed == 20)
            {
                String string = "TWO";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 30 && value_control_fan_speed > 20 || value_control_fan_speed == 30)
            {
                String string = "THREE";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 40 && value_control_fan_speed > 30 || value_control_fan_speed == 40)
            {
                String string = "FOUR";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 50 && value_control_fan_speed > 40 || value_control_fan_speed == 50)
            {
                String string = "FIVE";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 60 && value_control_fan_speed > 50 || value_control_fan_speed == 60)
            {
                String string = "SIX";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 70 && value_control_fan_speed > 60 || value_control_fan_speed == 70)
            {
                String string = "SEVEN";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 80 && value_control_fan_speed > 70 || value_control_fan_speed == 80)
            {
                String string = "EIGHT";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 90 && value_control_fan_speed > 80 || value_control_fan_speed == 90)
            {
                String string = "NINE";

                sendReceive.write(string.getBytes());
            }

            if (value_control_fan_speed < 100 && value_control_fan_speed > 90 || value_control_fan_speed == 100)
            {
                String string = "TEN";

                sendReceive.write(string.getBytes());
            }
        }
    }

    private class ServerClass extends Thread
    {
        private BluetoothServerSocket serverSocket;

        public ServerClass(){
            try {
                serverSocket=bluetoothAdapter.listenUsingRfcommWithServiceRecord(APP_NAME,MY_UUID);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public void run()
        {
            BluetoothSocket socket=null;

            while (socket==null)
            {
                try {
                    Message message=Message.obtain();
                    message.what=STATE_CONNECTING;
                    handler.sendMessage(message);

                    socket=serverSocket.accept();
                } catch (IOException e) {
                    e.printStackTrace();
                    Message message=Message.obtain();
                    message.what=STATE_CONNECTION_FAILED;
                    handler.sendMessage(message);
                }

                if(socket!=null)
                {
                    Message message=Message.obtain();
                    message.what=STATE_CONNECTED;
                    handler.sendMessage(message);

                    sendReceive=new SendReceive(socket);
                    sendReceive.start();

                    break;
                }
            }
        }
    }

    private class ClientClass extends Thread
    {
        private BluetoothDevice device;
        private BluetoothSocket socket;

        public ClientClass (BluetoothDevice device1)
        {
            device=device1;

            try {
                socket=device.createRfcommSocketToServiceRecord(MY_UUID);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }

        public void run()
        {
            try {
                socket.connect();
                Message message=Message.obtain();
                message.what=STATE_CONNECTED;
                handler.sendMessage(message);

                sendReceive=new SendReceive(socket);
                sendReceive.start();

            } catch (IOException e) {
                e.printStackTrace();
                Message message=Message.obtain();
                message.what=STATE_CONNECTION_FAILED;
                handler.sendMessage(message);
            }
        }
    }

    private class SendReceive extends Thread
    {
        private final BluetoothSocket bluetoothSocket;
        private final InputStream inputStream;
        private final OutputStream outputStream;

        public SendReceive (BluetoothSocket socket)
        {
            bluetoothSocket=socket;
            InputStream tempIn=null;
            OutputStream tempOut=null;

            try {
                tempIn=bluetoothSocket.getInputStream();
                tempOut=bluetoothSocket.getOutputStream();
            } catch (IOException e) {
                e.printStackTrace();
            }

            inputStream=tempIn;
            outputStream=tempOut;
        }

        public void run()
        {
            byte[] buffer=new byte[8192];
            int bytes;

            while (true)
            {
                try {
                    bytes=inputStream.read(buffer);
                    handler.obtainMessage(STATE_MESSAGE_RECEIVED,bytes,-1,buffer).sendToTarget();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

        public void write(byte[] bytes)
        {
            try {
                outputStream.write(bytes);
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }

    public void list(View view) {

        Set<BluetoothDevice> bt=bluetoothAdapter.getBondedDevices();
        String[] strings=new String[bt.size()];
        btArray=new BluetoothDevice[bt.size()];
        int index=0;

        if( bt.size()>0)
        {
            for(BluetoothDevice device : bt)
            {
                btArray[index]= device;
                strings[index]=device.getName();
                index++;
            }

            ArrayAdapter<String> arrayAdapter=new ArrayAdapter<String>(getApplicationContext(), R.layout.my_h, R.id.itemText,strings);
            listView.setAdapter(arrayAdapter);
        }

        if (isUp) {
            slideDown_menu(menuView);
        } else {
            slideUp_menu(menuView);
        }
        isUp = !isUp;

    }

    // Menu Main Animation
    public void slideUp_menu(View view){
        view.setVisibility(View.VISIBLE);
        TranslateAnimation animate = new TranslateAnimation(
                600,
                0,
                view.getScaleX(),
                0);
        animate.setDuration(400);
        animate.setFillAfter(true);
        view.startAnimation(animate);


    }

    public void slideDown_menu(View view){
        TranslateAnimation animate = new TranslateAnimation(
                0,
                -800,
                0,
                view.getScaleY());
        animate.setDuration(600);
        animate.setFillAfter(true);
        view.startAnimation(animate);

        Handler handler=new Handler();
        handler.postDelayed(new Runnable() {
            @Override
            public void run() {

                menuView.setVisibility(View.GONE);
            }
        },1500);
    }



    @Override
    public void onBackPressed() {

        back_press++;

        if (back_press == 2)
        {
            finish();
        }

        ///// End Function
    }
}