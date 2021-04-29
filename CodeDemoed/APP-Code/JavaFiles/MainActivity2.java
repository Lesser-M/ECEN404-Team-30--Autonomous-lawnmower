package lawnmower.app;

import android.content.Intent;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.RadioButton;

import androidx.appcompat.app.AppCompatActivity;



public class MainActivity2 extends AppCompatActivity {
        public RadioButton setButton;
        public Button setUpButton;


        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            setContentView(R.layout.activity2);

            setButton = (RadioButton) findViewById(R.id.settingsButton);
            setButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    Intent intent1 = new Intent(MainActivity2.this,SettingsActivity.class);
                    startActivity(intent1);
                }
            });

            setUpButton = (Button) findViewById(R.id.setupButton);
            setUpButton.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v) {
                    Intent intent2 = new Intent(MainActivity2.this,SetupActivity.class);
                    startActivity(intent2);
                }
            });

        }


}