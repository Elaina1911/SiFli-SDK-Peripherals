package com.openai.m3u8zipmp4;

import android.Manifest;
import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.database.Cursor;
import android.graphics.Color;
import android.net.Uri;
import android.os.Build;
import android.os.Bundle;
import android.provider.OpenableColumns;
import android.text.InputType;
import android.view.Gravity;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.EditText;
import android.widget.LinearLayout;
import android.widget.ProgressBar;
import android.widget.ScrollView;
import android.widget.TextView;
import android.widget.Toast;

import java.util.Locale;

public class MainActivity extends Activity {
    private static final int PICK_ZIP = 501;
    private static final int MODE_URL = 0;
    private static final int MODE_ZIP = 1;

    private EditText urlInput, nameInput, refererInput, cookieInput, userAgentInput;
    private TextView status, selectedZipText, modeUrl, modeZip;
    private ProgressBar progress;
    private Button startButton, cancelButton, advancedButton, chooseZipButton, openButton;
    private LinearLayout advancedBox, urlBox, zipBox;
    private int mode = MODE_URL;
    private Uri selectedZipUri;
    private String selectedZipName = "";
    private Uri resultUri;

    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        @Override public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            if (ConverterService.ACTION_PROGRESS.equals(action)) {
                int value = intent.getIntExtra("progress", 0);
                progress.setIndeterminate(value < 0);
                if (value >= 0) progress.setProgress(value);
                status.setText(intent.getStringExtra("message"));
                status.setTextColor(Color.parseColor("#667085"));
                setRunning(true);
            } else if (ConverterService.ACTION_DONE.equals(action)) {
                boolean ok = intent.getBooleanExtra("ok", false);
                progress.setIndeterminate(false);
                progress.setProgress(ok ? 100 : 0);
                status.setText(intent.getStringExtra("message"));
                status.setTextColor(Color.parseColor(ok ? "#067647" : "#B42318"));
                String uri = intent.getStringExtra("uri");
                resultUri = uri == null || uri.isEmpty() ? null : Uri.parse(uri);
                openButton.setVisibility(ok && resultUri != null ? View.VISIBLE : View.GONE);
                setRunning(false);
            }
        }
    };

    @Override protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_ADJUST_RESIZE);
        requestNeededPermissions();
        setContentView(buildUi());
        IntentFilter filter = new IntentFilter();
        filter.addAction(ConverterService.ACTION_PROGRESS);
        filter.addAction(ConverterService.ACTION_DONE);
        if (Build.VERSION.SDK_INT >= 33) registerReceiver(receiver, filter, Context.RECEIVER_NOT_EXPORTED);
        else registerReceiver(receiver, filter);
    }

    private void requestNeededPermissions() {
        if (Build.VERSION.SDK_INT >= 33 && checkSelfPermission(Manifest.permission.POST_NOTIFICATIONS) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.POST_NOTIFICATIONS}, 100);
        }
        if (Build.VERSION.SDK_INT <= 28 && checkSelfPermission(Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            requestPermissions(new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE}, 101);
        }
    }

    private View buildUi() {
        int pad = dp(18);
        ScrollView scroll = new ScrollView(this);
        scroll.setFillViewport(true);
        LinearLayout root = new LinearLayout(this);
        root.setOrientation(LinearLayout.VERTICAL);
        root.setPadding(pad, dp(24), pad, dp(36));
        root.setBackgroundColor(Color.parseColor("#F4F6F8"));
        scroll.addView(root, new ScrollView.LayoutParams(-1, -2));

        root.addView(text("🔒 全程在手机本地下载、解压与转换", 13, "#667085"));
        TextView title = text("M3U8 / ZIP 转 MP4", 29, "#111827");
        title.setTypeface(null, 1);
        title.setPadding(0, dp(10), 0, dp(6));
        root.addView(title);
        TextView sub = text("可以粘贴 m3u8 链接，也可以直接导入包含播放列表和分片的 ZIP。", 15, "#667085");
        sub.setLineSpacing(0, 1.35f);
        root.addView(sub);

        LinearLayout card = new LinearLayout(this);
        card.setOrientation(LinearLayout.VERTICAL);
        card.setPadding(dp(18), dp(18), dp(18), dp(18));
        card.setBackground(roundRect("#FFFFFF", 20));
        LinearLayout.LayoutParams cp = new LinearLayout.LayoutParams(-1, -2);
        cp.topMargin = dp(20);
        root.addView(card, cp);

        LinearLayout selector = new LinearLayout(this);
        selector.setOrientation(LinearLayout.HORIZONTAL);
        selector.setPadding(dp(4), dp(4), dp(4), dp(4));
        selector.setBackground(roundRect("#EAECF0", 14));
        modeUrl = modeButton("M3U8 链接");
        modeZip = modeButton("ZIP 压缩包");
        selector.addView(modeUrl, new LinearLayout.LayoutParams(0, dp(46), 1));
        selector.addView(modeZip, new LinearLayout.LayoutParams(0, dp(46), 1));
        card.addView(selector);
        modeUrl.setOnClickListener(v -> setMode(MODE_URL));
        modeZip.setOnClickListener(v -> setMode(MODE_ZIP));

        urlBox = new LinearLayout(this);
        urlBox.setOrientation(LinearLayout.VERTICAL);
        urlBox.addView(labelWithTop("M3U8 链接"));
        urlInput = edit("https://example.com/video/index.m3u8", InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        urlBox.addView(urlInput);
        advancedButton = button("高级设置（Referer / Cookie）", false);
        advancedButton.setOnClickListener(v -> toggleAdvanced());
        urlBox.addView(advancedButton);
        advancedBox = new LinearLayout(this);
        advancedBox.setOrientation(LinearLayout.VERTICAL);
        advancedBox.setVisibility(View.GONE);
        advancedBox.addView(labelWithTop("Referer（可选）"));
        refererInput = edit("https://来源网页/", InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        advancedBox.addView(refererInput);
        advancedBox.addView(labelWithTop("Cookie（可选）"));
        cookieInput = edit("name=value; ...", InputType.TYPE_CLASS_TEXT);
        advancedBox.addView(cookieInput);
        advancedBox.addView(labelWithTop("User-Agent"));
        userAgentInput = edit("Mozilla/5.0 (Linux; Android) AppleWebKit/537.36 Chrome Mobile Safari/537.36", InputType.TYPE_CLASS_TEXT);
        userAgentInput.setText("Mozilla/5.0 (Linux; Android) AppleWebKit/537.36 Chrome Mobile Safari/537.36");
        advancedBox.addView(userAgentInput);
        urlBox.addView(advancedBox);
        card.addView(urlBox);

        zipBox = new LinearLayout(this);
        zipBox.setOrientation(LinearLayout.VERTICAL);
        zipBox.setVisibility(View.GONE);
        zipBox.addView(labelWithTop("ZIP 压缩包"));
        chooseZipButton = button("选择 ZIP 文件", false);
        chooseZipButton.setOnClickListener(v -> chooseZip());
        zipBox.addView(chooseZipButton);
        selectedZipText = text("尚未选择文件。", 14, "#667085");
        selectedZipText.setPadding(dp(4), dp(10), dp(4), 0);
        selectedZipText.setLineSpacing(0, 1.3f);
        zipBox.addView(selectedZipText);
        card.addView(zipBox);

        card.addView(labelWithTop("输出文件名（可选）"));
        nameInput = edit("视频名称", InputType.TYPE_CLASS_TEXT);
        card.addView(nameInput);

        startButton = button("开始下载并转换", true);
        startButton.setOnClickListener(v -> startJob());
        card.addView(startButton);
        cancelButton = button("取消任务", false);
        cancelButton.setVisibility(View.GONE);
        cancelButton.setOnClickListener(v -> {
            Intent i = new Intent(this, ConverterService.class);
            i.setAction(ConverterService.ACTION_CANCEL);
            startService(i);
        });
        card.addView(cancelButton);
        openButton = button("打开转换后的视频", false);
        openButton.setVisibility(View.GONE);
        openButton.setOnClickListener(v -> openResult());
        card.addView(openButton);

        progress = new ProgressBar(this, null, android.R.attr.progressBarStyleHorizontal);
        progress.setMax(100);
        progress.setProgress(0);
        LinearLayout.LayoutParams pp = new LinearLayout.LayoutParams(-1, dp(8));
        pp.topMargin = dp(16);
        card.addView(progress, pp);
        status = text("请选择一种转换方式。", 14, "#667085");
        status.setPadding(0, dp(12), 0, 0);
        status.setLineSpacing(0, 1.3f);
        card.addView(status);

        TextView notes = text("链接模式支持普通 HLS 点播、主播放列表、相对地址和 AES-128。ZIP 模式会读取包内 m3u8，并自动匹配数字分片；没有播放列表时按自然顺序合并。SAMPLE-AES、Widevine 等 DRM 不会绕过。", 13, "#667085");
        notes.setLineSpacing(0, 1.45f);
        LinearLayout.LayoutParams np = new LinearLayout.LayoutParams(-1, -2);
        np.topMargin = dp(18);
        root.addView(notes, np);

        setMode(MODE_URL);
        return scroll;
    }

    private void setMode(int newMode) {
        mode = newMode;
        boolean url = mode == MODE_URL;
        urlBox.setVisibility(url ? View.VISIBLE : View.GONE);
        zipBox.setVisibility(url ? View.GONE : View.VISIBLE);
        modeUrl.setBackground(url ? roundRect("#FFFFFF", 11) : transparent());
        modeZip.setBackground(url ? transparent() : roundRect("#FFFFFF", 11));
        modeUrl.setTextColor(Color.parseColor(url ? "#111827" : "#667085"));
        modeZip.setTextColor(Color.parseColor(url ? "#667085" : "#111827"));
        startButton.setText(url ? "开始下载并转换" : "开始解压并转换");
        status.setText(url ? "请输入 m3u8 链接。" : "请选择 ZIP 压缩包。");
        status.setTextColor(Color.parseColor("#667085"));
        progress.setProgress(0);
        openButton.setVisibility(View.GONE);
    }

    private void chooseZip() {
        Intent i = new Intent(Intent.ACTION_OPEN_DOCUMENT);
        i.addCategory(Intent.CATEGORY_OPENABLE);
        i.setType("application/zip");
        i.putExtra(Intent.EXTRA_MIME_TYPES, new String[]{"application/zip", "application/x-zip-compressed", "application/octet-stream"});
        i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION);
        startActivityForResult(i, PICK_ZIP);
    }

    @Override protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != PICK_ZIP || resultCode != RESULT_OK || data == null || data.getData() == null) return;
        selectedZipUri = data.getData();
        try {
            int flags = data.getFlags() & (Intent.FLAG_GRANT_READ_URI_PERMISSION | Intent.FLAG_GRANT_WRITE_URI_PERMISSION);
            getContentResolver().takePersistableUriPermission(selectedZipUri, flags & Intent.FLAG_GRANT_READ_URI_PERMISSION);
        } catch (Exception ignored) {}
        selectedZipName = queryDisplayName(selectedZipUri);
        long size = querySize(selectedZipUri);
        selectedZipText.setText("已选择：" + selectedZipName + (size >= 0 ? "\n大小：" + formatBytes(size) : ""));
        selectedZipText.setTextColor(Color.parseColor("#344054"));
        status.setText("已选择 ZIP，点击开始转换。");
    }

    private void startJob() {
        Intent i = new Intent(this, ConverterService.class);
        i.setAction(ConverterService.ACTION_START);
        i.putExtra("mode", mode == MODE_URL ? "url" : "zip");
        i.putExtra("name", nameInput.getText().toString().trim());
        if (mode == MODE_URL) {
            String url = urlInput.getText().toString().trim();
            if (!(url.startsWith("http://") || url.startsWith("https://"))) {
                Toast.makeText(this, "请输入有效的 http/https m3u8 地址", Toast.LENGTH_LONG).show();
                return;
            }
            i.putExtra("url", url);
            i.putExtra("referer", refererInput.getText().toString().trim());
            i.putExtra("cookie", cookieInput.getText().toString().trim());
            i.putExtra("ua", userAgentInput.getText().toString().trim());
        } else {
            if (selectedZipUri == null) {
                Toast.makeText(this, "请先选择 ZIP 压缩包", Toast.LENGTH_LONG).show();
                return;
            }
            i.putExtra("zipUri", selectedZipUri.toString());
            i.putExtra("zipName", selectedZipName);
            i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
        }
        resultUri = null;
        openButton.setVisibility(View.GONE);
        status.setTextColor(Color.parseColor("#667085"));
        if (Build.VERSION.SDK_INT >= 26) startForegroundService(i); else startService(i);
        setRunning(true);
        progress.setIndeterminate(true);
        status.setText(mode == MODE_URL ? "正在读取播放列表…" : "正在读取 ZIP…");
    }

    private void setRunning(boolean running) {
        startButton.setEnabled(!running);
        modeUrl.setEnabled(!running);
        modeZip.setEnabled(!running);
        chooseZipButton.setEnabled(!running);
        cancelButton.setVisibility(running ? View.VISIBLE : View.GONE);
    }

    private void openResult() {
        if (resultUri == null) return;
        try {
            Intent i = new Intent(Intent.ACTION_VIEW);
            i.setDataAndType(resultUri, "video/mp4");
            i.addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION);
            startActivity(i);
        } catch (Exception e) {
            Toast.makeText(this, "找不到可打开 MP4 的应用", Toast.LENGTH_LONG).show();
        }
    }

    private void toggleAdvanced() {
        boolean show = advancedBox.getVisibility() != View.VISIBLE;
        advancedBox.setVisibility(show ? View.VISIBLE : View.GONE);
        advancedButton.setText(show ? "收起高级设置" : "高级设置（Referer / Cookie）");
    }

    private String queryDisplayName(Uri uri) {
        try (Cursor c = getContentResolver().query(uri, new String[]{OpenableColumns.DISPLAY_NAME}, null, null, null)) {
            if (c != null && c.moveToFirst()) return c.getString(0);
        } catch (Exception ignored) {}
        String last = uri.getLastPathSegment();
        return last == null ? "input.zip" : last;
    }

    private long querySize(Uri uri) {
        try (Cursor c = getContentResolver().query(uri, new String[]{OpenableColumns.SIZE}, null, null, null)) {
            if (c != null && c.moveToFirst() && !c.isNull(0)) return c.getLong(0);
        } catch (Exception ignored) {}
        return -1;
    }

    private static String formatBytes(long n) {
        if (n < 1024) return n + " B";
        if (n < 1024L * 1024) return String.format(Locale.US, "%.1f KB", n / 1024.0);
        if (n < 1024L * 1024 * 1024) return String.format(Locale.US, "%.1f MB", n / 1048576.0);
        return String.format(Locale.US, "%.2f GB", n / 1073741824.0);
    }

    private TextView modeButton(String s) {
        TextView v = text(s, 15, "#667085");
        v.setTypeface(null, 1);
        v.setGravity(Gravity.CENTER);
        return v;
    }
    private TextView labelWithTop(String s) { TextView v = text(s, 14, "#344054"); v.setTypeface(null, 1); v.setPadding(0, dp(16), 0, dp(7)); return v; }
    private TextView text(String s, int sp, String color) { TextView v = new TextView(this); v.setText(s); v.setTextSize(sp); v.setTextColor(Color.parseColor(color)); return v; }
    private EditText edit(String hint, int type) { EditText e = new EditText(this); e.setHint(hint); e.setTextSize(15); e.setSingleLine(false); e.setMinLines(1); e.setMaxLines(3); e.setInputType(type); e.setPadding(dp(12), dp(10), dp(12), dp(10)); e.setBackground(roundRectStroke("#FAFAFA", "#D0D5DD", 12)); return e; }
    private Button button(String s, boolean primary) { Button b = new Button(this); b.setText(s); b.setTextSize(15); b.setAllCaps(false); b.setTypeface(null, 1); b.setGravity(Gravity.CENTER); b.setPadding(dp(10), dp(12), dp(10), dp(12)); b.setTextColor(Color.parseColor(primary ? "#FFFFFF" : "#111827")); b.setBackground(primary ? roundRect("#111827", 14) : roundRectStroke("#FFFFFF", "#D0D5DD", 14)); LinearLayout.LayoutParams p = new LinearLayout.LayoutParams(-1, -2); p.topMargin = dp(12); b.setLayoutParams(p); return b; }
    private android.graphics.drawable.Drawable transparent() { android.graphics.drawable.GradientDrawable g = new android.graphics.drawable.GradientDrawable(); g.setColor(Color.TRANSPARENT); return g; }
    private android.graphics.drawable.Drawable roundRect(String color, int r) { android.graphics.drawable.GradientDrawable g = new android.graphics.drawable.GradientDrawable(); g.setColor(Color.parseColor(color)); g.setCornerRadius(dp(r)); return g; }
    private android.graphics.drawable.Drawable roundRectStroke(String color, String stroke, int r) { android.graphics.drawable.GradientDrawable g = (android.graphics.drawable.GradientDrawable) roundRect(color, r); g.setStroke(dp(1), Color.parseColor(stroke)); return g; }
    private int dp(int v) { return Math.round(v * getResources().getDisplayMetrics().density); }

    @Override protected void onDestroy() {
        try { unregisterReceiver(receiver); } catch (Exception ignored) {}
        super.onDestroy();
    }
}
