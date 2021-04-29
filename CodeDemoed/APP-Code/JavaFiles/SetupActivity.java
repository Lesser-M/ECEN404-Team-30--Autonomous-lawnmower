package lawnmower.app;

import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;
import android.widget.Toast;

import androidx.appcompat.app.AppCompatActivity;
import androidx.lifecycle.ViewModelProvider;
import androidx.recyclerview.widget.RecyclerView;

import com.google.android.material.dialog.MaterialAlertDialogBuilder;

import java.util.ArrayList;
import java.util.List;

import lawnmower.app.base.recycler.BaseRecyclerAdapter;
import lawnmower.app.base.recycler.BaseRecyclerViewHolder;
import lawnmower.app.base.recycler.RecyclerActionListener;
import lawnmower.app.holder.LawnViewHolder;
import lawnmower.app.model.Lawn;
import lawnmower.app.viewmodel.LawnViewModel;

public class SetupActivity extends AppCompatActivity {
    public View btnAdd;
    public TextView tvEstimatedTime, tvBattery;
    public Button btnGoHome, btnStart, btnStop;

    public View btnDefault;
    public TextView tvDefaultValue;
    public LawnViewModel lawnViewModel;
    public BaseRecyclerAdapter<Lawn> lawnAdapter;
    public Lawn defaultValue;

    private final RecyclerActionListener lawnActionListener = new RecyclerActionListener() {
        @Override
        public void onViewClick(int position, View view, BaseRecyclerViewHolder viewHolder) {
            Lawn lawn = (Lawn) lawnAdapter.getItemAt(position);
            LawnViewHolder holder = (LawnViewHolder) viewHolder;
            if (view == holder.itemView) {
                viewLawn(lawn);
            } else if (view == holder.btnRemove) {
                removeLawn(lawn);
            }
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_setup);

        lawnViewModel = new ViewModelProvider(this).get(LawnViewModel.class);
        btnAdd = findViewById(R.id.btnAdd);
        btnGoHome = findViewById(R.id.btnHome);
        btnStart = findViewById(R.id.btnStart);
        btnStop = findViewById(R.id.btnStop);
        tvEstimatedTime = findViewById(R.id.tvEstimateTime);
        tvBattery = findViewById(R.id.tvBattery);
        btnDefault = findViewById(R.id.btnDefault);
        tvDefaultValue = findViewById(R.id.tvDefaultValue);

        btnGoHome = findViewById(R.id.btnHome);
        btnGoHome.setOnClickListener(view -> goHome());
        btnStart.setOnClickListener(view -> doStart());
        btnStop.setOnClickListener(view -> doStop());
        btnAdd.setOnClickListener(v -> {
            Intent intent = new Intent(SetupActivity.this, AddLawnActivity.class);
            startActivity(intent);
        });

        btnDefault.setOnClickListener(this::pickDefaultLawn);

        RecyclerView rvLawn = findViewById(R.id.rvLawn);
        lawnAdapter = new BaseRecyclerAdapter<>(lawnActionListener);
        rvLawn.setAdapter(lawnAdapter);


        lawnViewModel.getDefaultLawn().observe(this, this::observerDefaultLawn);
        lawnViewModel.getLawns().observe(this, this::observerLawns);
        lawnViewModel.getBattery().observe(this, this::observerBattery);
        lawnViewModel.getEstimatedTime().observe(this, this::observerEstimateTime);

        lawnViewModel.getHomeStatus().observe(this, this::observerHomeStatus);
        lawnViewModel.getStartStatus().observe(this, this::observerStartStatus);
    }

    private void doStop() {
        lawnViewModel.setStartStatus(false);
    }

    private List<CharSequence> getLawnNameList() {
        List<Lawn> lawns = lawnAdapter.getData();
        List<CharSequence> lawnNames = new ArrayList<>();
        for (Lawn lawn : lawns) {
            lawnNames.add(lawn.name);
        }
        return lawnNames;
    }

    private Lawn getLawnByName(String name) {
        List<Lawn> lawns = lawnAdapter.getData();
        for (Lawn lawn : lawns) {
            if (name.equals(lawn.name)) return lawn;
        }
        return null;
    }

    private void pickDefaultLawn(View view) {
        List<CharSequence> nameList = getLawnNameList();
        CharSequence currentDefault = tvDefaultValue.getText();
        int selectIndex = nameList.indexOf(currentDefault);

        if (nameList.isEmpty()) {
            Toast.makeText(this, "You don't have any lawn!", Toast.LENGTH_SHORT).show();
            return;
        }
        new MaterialAlertDialogBuilder(this)
                .setSingleChoiceItems(
                        nameList.toArray(new CharSequence[0]), selectIndex, new DialogInterface.OnClickListener() {
                            @Override
                            public void onClick(DialogInterface dialogInterface, int i) {
                                if (i >= 0 && i < nameList.size()) {
                                    defaultValue = getLawnByName(nameList.get(i).toString());
                                }
                            }
                        }
                )
                .setPositiveButton("SELECT", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        lawnViewModel.addDefaultLawn(defaultValue);
                        dialogInterface.dismiss();
                    }
                })
                .setNegativeButton("CANCEL", new DialogInterface.OnClickListener() {
                    @Override
                    public void onClick(DialogInterface dialogInterface, int i) {
                        dialogInterface.dismiss();
                    }
                })
                .show();
    }

    private void observerDefaultLawn(Lawn defaultValue) {
        this.defaultValue = defaultValue;

        if (defaultValue == null) {
            tvDefaultValue.setText("Tap to select a default lawn");
            return;
        }

        if (!TextUtils.isEmpty(defaultValue.name)) {
            tvDefaultValue.setText(defaultValue.name);
        } else {
            tvDefaultValue.setText("Tap to select a default lawn");
        }
    }

    private void observerStartStatus(Boolean started) {
        if (started != null) {
            btnStart.setEnabled(!started);
            btnStop.setEnabled(started);
        }
    }

    private void doStart() {
        lawnViewModel.setStartStatus(true);
    }

    private void observerHomeStatus(Boolean isHome) {
        if (isHome != null) {
            btnGoHome.setEnabled(!isHome);
        }
    }

    private void goHome() {
        lawnViewModel.setHomeStatus(true);
    }

    private void observerEstimateTime(String estimateTime) {
        if (!TextUtils.isEmpty(estimateTime)) {
            tvEstimatedTime.setVisibility(View.VISIBLE);
            tvEstimatedTime.setText(String.format("Estimated time remaining: %s", estimateTime));
        } else {
            tvEstimatedTime.setVisibility(View.GONE);
        }
    }

    private void observerBattery(String battery) {
        if (!TextUtils.isEmpty(battery)) {
            tvBattery.setVisibility(View.VISIBLE);
            tvBattery.setText(String.format("Battery: %s", battery));
        } else {
            tvBattery.setVisibility(View.GONE);
        }
    }

    private void observerLawns(List<Lawn> lawns) {
        if (lawns != null) {
            Log.d("js.poul", "Lawn: " + lawns.size());
            lawnAdapter.update(lawns);

            // Check if exists default lawn ??
            if (defaultValue != null) {
                boolean found = false;
                for (Lawn lawn : lawns) {
                    if (lawn.name.equals(defaultValue.name)) {
                        lawnViewModel.addDefaultLawn(lawn);
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    lawnViewModel.removeDefaultLawn();
                }
            }
        } else {
            lawnViewModel.removeDefaultLawn();
        }
    }

    private void viewLawn(Lawn lawn) {
        Intent intent = new Intent(this, AddLawnActivity.class);
        intent.putExtra(AddLawnActivity.KEY_LAWN_NAME, lawn.name);
        startActivity(intent);
    }

    private void removeLawn(Lawn lawn) {
        lawnViewModel.removeLawn(lawn);
        if (lawn.name != null && lawn.name.equals(defaultValue.toString())) {
            lawnViewModel.removeDefaultLawn();
        }
    }
}