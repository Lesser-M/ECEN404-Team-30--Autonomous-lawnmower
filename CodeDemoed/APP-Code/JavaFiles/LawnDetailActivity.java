package lawnmower.app;

import android.graphics.Color;
import android.location.Address;
import android.location.Geocoder;
import android.os.Bundle;

import androidx.fragment.app.FragmentActivity;
import androidx.lifecycle.ViewModelProvider;

import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polyline;
import com.google.android.gms.maps.model.PolylineOptions;

import java.io.IOException;
import java.util.List;

import lawnmower.app.model.Lawn;
import lawnmower.app.viewmodel.LawnViewModel;

public class LawnDetailActivity extends FragmentActivity implements OnMapReadyCallback {

    public static final String KEY_LAWN_NAME = "key_lawn_name";
    private GoogleMap mMap;
    private Polyline lawnPath;
    private LawnViewModel lawnViewModel;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_lawn_detail);

        SupportMapFragment mapFragment = (SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.map);
        mapFragment.getMapAsync(this);
        lawnViewModel = new ViewModelProvider(this).get(LawnViewModel.class);
    }

    private void observeLawn(Lawn lawn) {
        if (lawn != null) {
            showLawn(lawn);
        }
    }

    @Override
    public void onMapReady(GoogleMap googleMap) {
        mMap = googleMap;
        mMap.setMapType(GoogleMap.MAP_TYPE_NORMAL);
        lawnPath = mMap.addPolyline(new PolylineOptions().color(Color.RED).width(6f));

        String lawnName = getIntent().getStringExtra(KEY_LAWN_NAME);
        lawnViewModel.getLawnByName(lawnName).observe(this, this::observeLawn);
    }

    private void showLawn(Lawn lawn) {
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
            }
        }

        lawnPath.setPoints(lawn.getMapPositions());
    }

    private Address fromLocation(double latitude, double longitude) {
        try {
            Geocoder geocoder = new Geocoder(this);
            List<Address> addresses = geocoder.getFromLocation(latitude, longitude, 1);
            return addresses != null && !addresses.isEmpty() ? addresses.get(0) : null;
        } catch (IOException e) {
            return null;
        }
    }

}
