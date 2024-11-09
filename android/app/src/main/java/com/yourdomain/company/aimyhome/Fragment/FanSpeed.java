package com.yourdomain.company.aimyhome.Fragment;

import android.content.Intent;
import android.graphics.Color;
import android.graphics.drawable.ColorDrawable;
import android.net.Uri;
import android.os.Bundle;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.view.Window;
import android.widget.CheckBox;
import android.widget.ImageView;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.yourdomain.company.aimyhome.MainActivity;
import com.yourdomain.company.aimyhome.R;


public class FanSpeed extends androidx.fragment.app.DialogFragment {

    ImageView close;
    String speed_value = "0";
    TextView speed, cancel, set;
    SeekBar seekBar;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        super.onCreateView(inflater, container, savedInstanceState);
        final View view = inflater.inflate(R.layout.fan_speed_fragment,container,false);


        MainActivity activity = (MainActivity) getActivity();
        String get_speed = activity.getSpeed();

        if (getDialog() != null && getDialog().getWindow() != null) {
            getDialog().getWindow().setBackgroundDrawable(new ColorDrawable(Color.TRANSPARENT));
            getDialog().getWindow().requestFeature(Window.FEATURE_NO_TITLE);
        }

        speed = (TextView)view.findViewById(R.id.speed);
        cancel = (TextView)view.findViewById(R.id.cancel);
        set = (TextView)view.findViewById(R.id.set);
        close = (ImageView)view.findViewById(R.id.close);
        seekBar = (SeekBar)view.findViewById(R.id.seekbar);


        speed.setText(get_speed + "%");
        speed_value = get_speed;
        int i = Integer.valueOf(get_speed);
        seekBar.setProgress(i);

        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {

                speed_value = String.valueOf(progress);
                speed.setText(speed_value + "%");
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });


        close.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                dismiss();

            }
        });

        cancel.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                dismiss();

            }
        });

        set.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {

                ((MainActivity)getActivity()).dispatchInformations(speed_value);
                dismiss();

            }
        });

        return view;
    }

}
