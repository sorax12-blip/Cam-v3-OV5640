package com.openai.camv3;

import android.Manifest;
import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.BluetoothProfile;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.ContentValues;
import android.content.Context;
import android.content.pm.PackageManager;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.Looper;
import android.provider.MediaStore;
import android.util.Base64;
import android.webkit.JavascriptInterface;
import android.webkit.WebChromeClient;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Toast;

import org.json.JSONObject;

import java.io.File;
import java.io.FileOutputStream;
import java.io.OutputStream;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.UUID;

public class MainActivity extends Activity {
    private static final String DEVICE_NAME = "ESP32 Cam HD";
    private static final UUID SERVICE_UUID = UUID.fromString("b6a30001-6e8c-4b74-a4c2-3b33d2c7a001");
    private static final UUID COMMAND_UUID = UUID.fromString("b6a30002-6e8c-4b74-a4c2-3b33d2c7a001");
    private static final UUID STATUS_UUID = UUID.fromString("b6a30003-6e8c-4b74-a4c2-3b33d2c7a001");
    private static final UUID DATA_UUID = UUID.fromString("b6a30004-6e8c-4b74-a4c2-3b33d2c7a001");
    private static final UUID CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
    private static final int REQ_BT = 20;

    private WebView webView;
    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic commandChar, statusChar, dataChar;
    private final Handler handler = new Handler(Looper.getMainLooper());
    private boolean connectedReady = false;
    private int notifyStage = 0;
    private final ArrayDeque<byte[]> commandQueue = new ArrayDeque<>();
    private boolean commandWriteInProgress = false;

    @Override public void onCreate(Bundle state) {
        super.onCreate(state);
        BluetoothManager manager = (BluetoothManager)getSystemService(Context.BLUETOOTH_SERVICE);
        adapter = manager.getAdapter();

        webView = new WebView(this);
        webView.setOnApplyWindowInsetsListener((v, insets) -> {
            int top = insets.getSystemWindowInsetTop();
            int bottom = insets.getSystemWindowInsetBottom();
            v.setPadding(0, top, 0, bottom);
            return insets;
        });
        setContentView(webView);
        WebSettings ws = webView.getSettings();
        ws.setJavaScriptEnabled(true);
        ws.setDomStorageEnabled(true);
        ws.setAllowFileAccess(true);
        ws.setAllowContentAccess(false);
        webView.setWebChromeClient(new WebChromeClient());
        webView.setWebViewClient(new WebViewClient());
        webView.addJavascriptInterface(new JsBridge(), "AndroidBridge");
        webView.loadUrl("file:///android_asset/index.html");
    }

    private boolean hasBtPermissions() {
        if (Build.VERSION.SDK_INT >= 31) {
            return checkSelfPermission(Manifest.permission.BLUETOOTH_SCAN) == PackageManager.PERMISSION_GRANTED &&
                   checkSelfPermission(Manifest.permission.BLUETOOTH_CONNECT) == PackageManager.PERMISSION_GRANTED;
        }
        return checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED;
    }

    private void askBtPermissions() {
        if (Build.VERSION.SDK_INT >= 31) {
            requestPermissions(new String[]{Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT}, REQ_BT);
        } else {
            requestPermissions(new String[]{Manifest.permission.ACCESS_FINE_LOCATION}, REQ_BT);
        }
    }

    @Override public void onRequestPermissionsResult(int req, String[] perms, int[] grants) {
        super.onRequestPermissionsResult(req, perms, grants);
        if (req == REQ_BT) {
            if (hasBtPermissions()) startScan();
            else jsMessage("Bluetooth permission is required to connect to the camera.");
        }
    }

    private void connectCamera() {
        if (adapter == null || !adapter.isEnabled()) { jsMessage("Turn Android Bluetooth on first."); return; }
        if (!hasBtPermissions()) { askBtPermissions(); return; }
        startScan();
    }

    @SuppressWarnings("MissingPermission")
    private void startScan() {
        disconnectGatt();
        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) { jsMessage("BLE scanner unavailable."); return; }
        jsMessage("Scanning for " + DEVICE_NAME + "…");
        ScanFilter filter = new ScanFilter.Builder().setDeviceName(DEVICE_NAME).build();
        ScanSettings settings = new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
        scanner.startScan(Collections.singletonList(filter), settings, scanCallback);
        handler.postDelayed(() -> {
            try { if (scanner != null) scanner.stopScan(scanCallback); } catch (Exception ignored) {}
            // Silent timeout: the personal UI does not need a persistent "camera not found" message.
        }, 12000);
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @Override public void onScanResult(int callbackType, ScanResult result) {
            BluetoothDevice d = result.getDevice();
            handler.post(() -> connectDevice(d));
        }
        @Override public void onScanFailed(int errorCode) { jsMessage("BLE scan failed: " + errorCode); }
    };

    @SuppressWarnings("MissingPermission")
    private void connectDevice(BluetoothDevice device) {
        try { if (scanner != null) scanner.stopScan(scanCallback); } catch (Exception ignored) {}
        scanner = null;
        jsMessage("Connecting…");
        if (Build.VERSION.SDK_INT >= 23) gatt = device.connectGatt(this, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
        else gatt = device.connectGatt(this, false, gattCallback);
    }

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                jsMessage("Connected; discovering SD browser service…");
                try { g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH); } catch (Exception ignored) {}
                g.discoverServices();
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                connectedReady = false; notifyStage = 0; commandChar = statusChar = dataChar = null;
                js("window.onNativeDisconnected&&window.onNativeDisconnected()");
                try { g.close(); } catch (Exception ignored) {}
                if (gatt == g) gatt = null;
            }
        }

        @Override public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService svc = g.getService(SERVICE_UUID);
            if (svc == null) { jsMessage("Camera BLE service not found. Update the ESP32 firmware."); return; }
            commandChar = svc.getCharacteristic(COMMAND_UUID);
            statusChar = svc.getCharacteristic(STATUS_UUID);
            dataChar = svc.getCharacteristic(DATA_UUID);
            if (commandChar == null || statusChar == null || dataChar == null) { jsMessage("Camera BLE characteristics are incomplete."); return; }
            if (Build.VERSION.SDK_INT >= 21) g.requestMtu(247); else enableNextNotification();
        }

        @Override public void onMtuChanged(BluetoothGatt g, int mtu, int status) { enableNextNotification(); }

        @Override public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor descriptor, int status) { enableNextNotification(); }

        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c) { handleNotification(c.getUuid(), c.getValue()); }

        @Override public void onCharacteristicChanged(BluetoothGatt g, BluetoothGattCharacteristic c, byte[] value) { handleNotification(c.getUuid(), value); }

        @Override public void onCharacteristicWrite(BluetoothGatt g, BluetoothGattCharacteristic c, int status) {
            if (!COMMAND_UUID.equals(c.getUuid())) return;
            handler.post(() -> {
                if (!commandQueue.isEmpty()) commandQueue.pollFirst();
                commandWriteInProgress = false;
                if (status != BluetoothGatt.GATT_SUCCESS) jsMessage("BLE command write failed: " + status);
                drainCommandQueue();
            });
        }
    };

    @SuppressWarnings("MissingPermission")
    private void enableNextNotification() {
        BluetoothGattCharacteristic c;
        if (notifyStage == 0) c = statusChar;
        else if (notifyStage == 1) c = dataChar;
        else {
            if (!connectedReady) {
                connectedReady = true;
                js("window.onNativeConnected&&window.onNativeConnected()");
            }
            return;
        }
        notifyStage++;
        if (gatt == null || c == null) return;
        gatt.setCharacteristicNotification(c, true);
        BluetoothGattDescriptor d = c.getDescriptor(CCCD_UUID);
        if (d == null) { enableNextNotification(); return; }
        if (Build.VERSION.SDK_INT >= 33) gatt.writeDescriptor(d, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);
        else { d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE); gatt.writeDescriptor(d); }
    }

    private void handleNotification(UUID uuid, byte[] value) {
        if (value == null) return;
        if (STATUS_UUID.equals(uuid)) {
            String s = new String(value, StandardCharsets.UTF_8);
            js("window.onNativeStatus&&window.onNativeStatus(" + JSONObject.quote(s) + ")");
        } else if (DATA_UUID.equals(uuid)) {
            String rawB64 = Base64.encodeToString(value, Base64.NO_WRAP);
            if (value.length >= 4) {
                int offset = ByteBuffer.wrap(value, 0, 4).order(ByteOrder.LITTLE_ENDIAN).getInt();
                byte[] payload = new byte[value.length - 4];
                System.arraycopy(value, 4, payload, 0, payload.length);
                String payloadB64 = Base64.encodeToString(payload, Base64.NO_WRAP);
                js("if(window.onNativeDataRaw){window.onNativeDataRaw(" + JSONObject.quote(rawB64) + ");}" +
                   "else if(window.onNativeData){window.onNativeData(" + (offset & 0xffffffffL) + "," + JSONObject.quote(payloadB64) + ");}");
            } else {
                js("window.onNativeDataRaw&&window.onNativeDataRaw(" + JSONObject.quote(rawB64) + ")");
            }
        }
    }

    @SuppressWarnings("MissingPermission")
    private void sendCommand(String text) {
        if (!connectedReady || gatt == null || commandChar == null) { jsMessage("Camera is not connected yet."); return; }
        byte[] value = text.getBytes(StandardCharsets.UTF_8);
        if (Build.VERSION.SDK_INT >= 33) {
            gatt.writeCharacteristic(commandChar, value, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        } else {
            commandChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
            commandChar.setValue(value);
            gatt.writeCharacteristic(commandChar);
        }
    }

    @SuppressWarnings("MissingPermission")
    private void disconnectGatt() {
        connectedReady = false; notifyStage = 0; commandQueue.clear(); commandWriteInProgress = false;
        try { if (scanner != null) scanner.stopScan(scanCallback); } catch (Exception ignored) {}
        scanner = null;
        if (gatt != null) { try { gatt.disconnect(); gatt.close(); } catch (Exception ignored) {} gatt = null; }
        commandChar = statusChar = dataChar = null;
    }

    private void savePicture(String filename, String base64) {
        try {
            byte[] data = Base64.decode(base64, Base64.DEFAULT);
            if (Build.VERSION.SDK_INT >= 29) {
                ContentValues values = new ContentValues();
                values.put(MediaStore.Downloads.DISPLAY_NAME, filename);
                values.put(MediaStore.Downloads.MIME_TYPE, "image/jpeg");
                values.put(MediaStore.Downloads.RELATIVE_PATH, Environment.DIRECTORY_DOWNLOADS + "/ESP32 Cam HD");
                Uri uri = getContentResolver().insert(MediaStore.Downloads.EXTERNAL_CONTENT_URI, values);
                if (uri == null) throw new Exception("Could not create Downloads file");
                try (OutputStream out = getContentResolver().openOutputStream(uri)) { out.write(data); }
            } else {
                File dir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_DOWNLOADS);
                if (!dir.exists()) dir.mkdirs();
                try (FileOutputStream out = new FileOutputStream(new File(dir, filename))) { out.write(data); }
            }
            jsMessage("Saved " + filename + " to Downloads/ESP32 Cam HD");
        } catch (Exception e) { jsMessage("Save failed: " + e.getMessage()); }
    }

    private void js(String code) { handler.post(() -> webView.evaluateJavascript(code, null)); }
    private void jsMessage(String text) { js("window.onNativeMessage&&window.onNativeMessage(" + JSONObject.quote(text) + ")"); }

    public class JsBridge {
        @JavascriptInterface public void connect() { handler.post(() -> connectCamera()); }
        @JavascriptInterface public void disconnect() { handler.post(() -> { disconnectGatt(); js("window.onNativeDisconnected&&window.onNativeDisconnected()"); }); }
        @JavascriptInterface public void sendCommand(String command) { handler.post(() -> MainActivity.this.sendCommand(command)); }
        @JavascriptInterface public void saveFile(String filename, String base64) { new Thread(() -> savePicture(filename, base64)).start(); }
    }

    @Override protected void onDestroy() { disconnectGatt(); if (webView != null) webView.destroy(); super.onDestroy(); }
}
