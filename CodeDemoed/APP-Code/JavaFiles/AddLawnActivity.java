package lawnmower.app;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.location.Address;
import android.location.Geocoder;
import android.location.Location;
import android.os.Bundle;
import android.text.TextUtils;
import android.view.View;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

import androidx.activity.result.ActivityResultCallback;
import androidx.activity.result.ActivityResultLauncher;
import androidx.activity.result.contract.ActivityResultContracts;
import androidx.fragment.app.FragmentActivity;
import androidx.lifecycle.ViewModelProvider;

import com.google.android.gms.location.FusedLocationProviderClient;
import com.google.android.gms.location.LocationServices;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;
import com.google.android.gms.tasks.OnSuccessListener;
import com.google.android.gms.tasks.Task;
import com.google.android.material.textfield.TextInputLayout;
import com.google.android.material.timepicker.MaterialTimePicker;
import com.google.android.material.timepicker.TimeFormat;

import org.threeten.bp.DayOfWeek;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

import lawnmower.app.model.Lawn;
import lawnmower.app.model.LawnLocations;
import lawnmower.app.model.LawnPosition;
import lawnmower.app.model.LawnSchedule;
import lawnmower.app.viewmodel.LawnViewModel;

public class AddLawnActivity extends FragmentActivity implements OnMapReadyCallback, GoogleMap.OnMapLongClickListener, GoogleMap.OnMarkerDragListener {

    public static final String KEY_LAWN_NAME = "key_lawn_name";

    private GoogleMap mMap;
    private Geocoder geocoder;
    private FusedLocationProviderClient fusedLocationProviderClient;
    private Polyline lawnPath;

    private TextView tvStartTime;
    private TextInputLayout inputName;
    private EditText edtName;
    private List<CheckBox> checkBoxes;

    private LawnViewModel lawnViewModel;
    private final Set<Marker> lawnMarkers = new LinkedHashSet<>();
    private final LawnSchedule lawnSchedule = new LawnSchedule();
    private final PermissionManager permissionManager = PermissionManager.getInstance();
    private final ActivityResultLauncher<String[]> permissionLauncher = registerForActivityResult(new ActivityResultContracts.RequestMultiplePermissions(), new ActivityResultCallback<Map<String, Boolean>>() {
        @Override
        public void onActivityResult(Map<String, Boolean> result) {
            for (Map.Entry<String, Boolean> entry : result.entrySet()) {
                if (!entry.getValue()) return;
            }
            initMap();
        }
    });

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_add_lawn);

        lawnViewModel = new ViewModelProvider(this).get(LawnViewModel.class);
        if (!PermissionManager.getInstance().hasPermissions(this)) {
            requestPermissions();
        } else {
            initMap();
        }
    }

    private void requestPermissions() {
        permissionLauncher.launch(permissionManager.getAppPermissions());
    }

    private void initMap() {
        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
        geocoder = new Geocoder(this);
        fusedLocationProviderClient = LocationServices.getFusedLocationProviderClient(this);

        tvStartTime = findViewById(R.id.tvStartTime);
        tvStartTime.setOnClickListener(this::onTimeSelected);
        inputName = findViewById(R.id.inputName);
        edtName = findViewById(R.id.edtName);

        checkBoxes = new ArrayList<>();
        CheckBox mo = findViewById(R.id.mo);
        CheckBox tu = findViewById(R.id.tu);
        CheckBox we = findViewById(R.id.we);
        CheckBox th = findViewById(R.id.th);
        CheckBox fr = findViewById(R.id.fr);
        CheckBox sa = findViewById(R.id.sa);
        CheckBox su = findViewById(R.id.su);

        checkBoxes.add(mo);
        checkBoxes.add(tu);
        checkBoxes.add(we);
        checkBoxes.add(th);
        checkBoxes.add(fr);
        checkBoxes.add(sa);
        checkBoxes.add(su);


        findViewById(R.id.btnSave).setOnClickListener(this::onSaveLawn);

        String lawnName = getIntent().getStringExtra(KEY_LAWN_NAME);
        if (!TextUtils.isEmpty(lawnName)) {
            lawnViewModel.getLawnByName(lawnName).observe(this, this::observeLawn);
        }
    }

    private void observeLawn(Lawn lawn) {
        if (lawn != null) {
            showLawn(lawn);
        }
    }

    private void showLawn(Lawn lawn) {
        // Set name
        inputName.setEnabled(false);
        edtName.setText(lawn.name);

        // Set positions
        for (int i = 0; i < lawn.getMapPositions().size(); i++) {
            LatLng position = lawn.getMapPositions().get(i);
            if (i == 0) {
                mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(position, 15));
            }

            Address address = fromLocation(position.latitude, position.longitude);
            if (address != null) {
                String streetAddress = address.getAddressLine(0);
                Marker newLawnMarker = mMap.addMarker(new MarkerOptions()
                        .position(position)
                        .title(String.valueOf(i + 1))
                        .snippet(streetAddress)
                        .draggable(true)
                );
                newLawnMarker.showInfoWindow();
                lawnMarkers.add(newLawnMarker);
            }
        }
        lawnPath.setPoints(lawn.getMapPositions());

        // Set schedule
        lawnSchedule.startHour = lawn.schedule.startHour;
        lawnSchedule.startMinute = lawn.schedule.startMinute;
        tvStartTime.setText(String.format(Locale.getDefault(), "%d:%d", lawn.schedule.startHour, lawn.schedule.startMinute));

        for (DayOfWeek dayOfWeek : lawn.schedule.daysInWeek) {
            int checkBoxPosition = dayOfWeek.getValue();
            checkBoxes.get(checkBoxPosition - 1).setChecked(true);
        }
    }

    private void onTimeSelected(View view) {
        MaterialTimePicker timePicker = new MaterialTimePicker.Builder()
                .setTimeFormat(TimeFormat.CLOCK_24H)
                .setHour(12)
                .setMinute(0)
                .build();

        timePicker.addOnPositiveButtonClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                int hour = timePicker.getHour();
                int minute = timePicker.getMinute();
                lawnSchedule.startHour = hour;
                lawnSchedule.startMinute = minute;
                tvStartTime.setText(String.format(Locale.getDefault(), "%2d:%2d", hour, minute));
            }
        });

        timePicker.show(getSupportFragmentManager(), "");
    }

    private void onSaveLawn(View view) {
        TextInputLayout inputName = findViewById(R.id.inputName);
        String name = inputName.getEditText().getEditableText().toString();
        setLawnSchedule();

        if (checkValid()) {
            LawnLocations lawnLocations = new LawnLocations();
            lawnLocations.positions = getLawnLocations();
            lawnViewModel.saveLawn(new Lawn(name, lawnLocations, lawnSchedule));
            finish();
        } else {
            Toast.makeText(this, "Please input all information!", Toast.LENGTH_SHORT).show();
        }
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;

        mMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
        mMap.setOnMapLongClickListener(this);
        mMap.setOnMarkerDragListener(this);

        enableUserLocation();
        zoomToUserLocation();

        lawnPath = mMap.addPolyline(new PolylineOptions().color(Color.RED).width(6f));
    }

    @SuppressLint("MissingPermission")
    private void enableUserLocation() {
        mMap.setMyLocationEnabled(true);
    }

    @SuppressLint("MissingPermission")
    private void zoomToUserLocation() {
        Task<Location> locationTask = fusedLocationProviderClient.getLastLocation();
        locationTask.addOnSuccessListener(new OnSuccessListener<Location>() {
            @Override
            public void onSuccess(Location location) {
                if (location != null) {
                    LatLng latLng = new LatLng(location.getLatitude(), location.getLongitude());
                    mMap.moveCamera(CameraUpdateFactory.newLatLngZoom(latLng, 15));
                }
            }
        });
    }

    @Override
    public void onMapLongClick(LatLng latLng) {
        Address address = fromLocation(latLng.latitude, latLng.longitude);
        if (address != null) {
            String streetAddress = address.getAddressLine(0);
            Marker newLawnMarker = mMap.addMarker(new MarkerOptions()
                    .position(latLng)
                    .title(String.valueOf(lawnMarkers.size() + 1))
                    .snippet(streetAddress)
                    .draggable(true)
            );
            newLawnMarker.showInfoWindow();
            lawnMarkers.add(newLawnMarker);
            lawnPath.setPoints(getLawnPath());
        }
    }

    @Override
    public void onMarkerDragStart(Marker marker) {

    }

    @Override
    public void onMarkerDrag(Marker marker) {
    }

    @Override
    public void onMarkerDragEnd(Marker marker) {
        LatLng latLng = marker.getPosition();
        Address address = fromLocation(latLng.latitude, latLng.longitude);
        if (address != null) {
            String streetAddress = address.getAddressLine(0);
            marker.setTitle(streetAddress);
        }
        lawnPath.setPoints(getLawnPath());
    }

    private Address fromLocation(double latitude, double longitude) {
        try {
            List<Address> addresses = geocoder.getFromLocation(latitude, longitude, 1);
            return addresses != null && !addresses.isEmpty() ? addresses.get(0) : null;
        } catch (IOException e) {
            return null;
        }
    }

    private List<LatLng> getLawnPath() {
        List<LatLng> path = new ArrayList<>();
        for (Marker marker : lawnMarkers) {
            LatLng position = marker.getPosition();
            path.add(position);
        }
        return path;
    }

    private List<LawnPosition> getLawnLocations() {
        List<LawnPosition> path = new ArrayList<>();
        for (Marker marker : lawnMarkers) {
            LatLng position = marker.getPosition();
            LawnPosition lawnPosition = new LawnPosition();
            lawnPosition.latitude = position.latitude;
            lawnPosition.longitude = position.longitude;
            path.add(lawnPosition);
        }
        return path;
    }

    private void setLawnSchedule() {
        List<DayOfWeek> dayOfWeeks = new ArrayList<>();
        for (int i = 1; i <= checkBoxes.size(); i++) {
            CheckBox checkBox = checkBoxes.get(i - 1);
            if (checkBox.isChecked()) {
                DayOfWeek dow = DayOfWeek.of(i);
                dayOfWeeks.add(dow);
            }
        }

        lawnSchedule.daysInWeek = dayOfWeeks;
    }

    private boolean checkValid() {
        String name = inputName.getEditText().getEditableText().toString();

        return (!TextUtils.isEmpty(name) && lawnSchedule.checkValid() && !getLawnPath().isEmpty());
    }
}