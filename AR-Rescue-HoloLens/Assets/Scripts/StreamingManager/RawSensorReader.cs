using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

#if !UNITY_EDITOR && UNITY_METRO
using Windows.Media;
using Windows.Media.Capture;
using Windows.Media.Capture.Frames;
using System.Threading.Tasks;
using System;
using Windows.Media.MediaProperties;
using Windows.Graphics.Imaging;
using System.Threading;
using System.Linq;
using System.Collections.Concurrent;
using System.Runtime.InteropServices.WindowsRuntime;
#endif

public class RawSensorReader : MonoBehaviour
{
    public int id;
    public int group_id;

    private enum CaptureStatus
    {
        Clean,
        Initialized,
        Running
    }
    private CaptureStatus captureStatus = CaptureStatus.Clean;


#if !UNITY_EDITOR && UNITY_METRO
    private SoftwareBitmap upBitmap = null;
    private MediaCapture mediaCapture;

    private MediaFrameReader frameReader = null;

    private int videoWidth = 0;
    private int videoHeight = 0;

    private async Task<bool> InitializeMediaCaptureAsync() {
        if (captureStatus != CaptureStatus.Clean) {
            Debug.Log(": InitializeMediaCaptureAsync() fails because of incorrect status");
            return false;
        }

        if (mediaCapture != null) {
            return false;
        }

        var allGroups = await MediaFrameSourceGroup.FindAllAsync();
        foreach (var group in allGroups) {
            Debug.Log(group.DisplayName + ", " + group.Id);
        }
        
        if (allGroups.Count <= 0) {
            Debug.Log(": InitializeMediaCaptureAsync() fails because there is no MediaFrameSourceGroup");
            return false;
        }
        
        // Initialize mediacapture with the source group.
        mediaCapture = new MediaCapture();
        var settings = new MediaCaptureInitializationSettings {
            SourceGroup = allGroups[group_id],
            // This media capture can share streaming with other apps.
            SharingMode = MediaCaptureSharingMode.SharedReadOnly,
            // Only stream video and don't initialize audio capture devices.
            StreamingCaptureMode = StreamingCaptureMode.Video,
            // Set to CPU to ensure frames always contain CPU SoftwareBitmap images
            // instead of preferring GPU D3DSurface images.
            MemoryPreference = MediaCaptureMemoryPreference.Cpu
        };

        await mediaCapture.InitializeAsync(settings);
        Debug.Log(": MediaCapture is successfully initialized in shared mode.");

        MediaFrameSource targetFrameSource = mediaCapture.FrameSources.Values.ElementAt(id);
        MediaFrameFormat targetResFormat = targetFrameSource.SupportedFormats[0];
        try {
            await targetFrameSource.SetFormatAsync(targetResFormat);
            frameReader = await mediaCapture.CreateFrameReaderAsync(targetFrameSource, targetResFormat.Subtype);
            frameReader.FrameArrived += OnFrameArrived;
            videoWidth = Convert.ToInt32(targetResFormat.VideoFormat.Width);
            videoHeight = Convert.ToInt32(targetResFormat.VideoFormat.Height);
            Debug.Log(": FrameReader is successfully initialized, " + videoWidth + "x" + videoHeight + 
                ", Framerate: " + targetResFormat.FrameRate.Numerator + "/" + targetResFormat.FrameRate.Denominator + 
                ", Major type: " + targetResFormat.MajorType + ", Subtype: " + targetResFormat.Subtype);
        }
        catch (Exception e) {
            Debug.Log(": FrameReader is not initialized");
            Debug.Log(": Exception: " + e);
            return false;
        }
        
        captureStatus = CaptureStatus.Initialized;
        return true;
    }
    
    private async Task<bool> StartFrameReaderAsync() {
        Debug.Log(" StartFrameReaderAsync() thread ID is " + Thread.CurrentThread.ManagedThreadId);
        if (captureStatus != CaptureStatus.Initialized) {
            Debug.Log(": StartFrameReaderAsync() fails because of incorrect status");
            return false;
        }
        
        MediaFrameReaderStartStatus status = await frameReader.StartAsync();
        if (status == MediaFrameReaderStartStatus.Success) {
            Debug.Log(": StartFrameReaderAsync() is successful");
            captureStatus = CaptureStatus.Running;
            return true;
        }
        else {
            Debug.Log(": StartFrameReaderAsync() is successful, status = " + status);
            return false;
        }
    }

    private async Task<bool> StopFrameReaderAsync() {
        if (captureStatus != CaptureStatus.Running) {
            Debug.Log(": StopFrameReaderAsync() fails because of incorrect status");
            return false;
        }
        await frameReader.StopAsync();
        captureStatus = CaptureStatus.Initialized;
        upBitmap = null;
        Debug.Log(": StopFrameReaderAsync() is successful");
        return true;
    }

    private bool onFrameArrivedProcessing = false;
    
    private unsafe void OnFrameArrived(MediaFrameReader sender, MediaFrameArrivedEventArgs args) {
        // TryAcquireLatestFrame will return the latest frame that has not yet been acquired.
        // This can return null if there is no such frame, or if the reader is not in the
        // "Started" state. The latter can occur if a FrameArrived event was in flight
        // when the reader was stopped.
        if (onFrameArrivedProcessing) {
            Debug.Log(" OnFrameArrived() is still processing");
            return;
        }
        onFrameArrivedProcessing = true;
        using (var frame = sender.TryAcquireLatestFrame()) {
            if (frame != null) {
                Debug.Log("frame received");
                var softwareBitmap = SoftwareBitmap.Convert(frame.VideoMediaFrame.SoftwareBitmap, 
                    BitmapPixelFormat.Rgba8, BitmapAlphaMode.Ignore);

                Interlocked.Exchange(ref upBitmap, softwareBitmap);
            }
        }
        onFrameArrivedProcessing = false;
    }
    
    async void InitializeMediaCaptureAsyncWrapper() {
        await InitializeMediaCaptureAsync();
    }

    async void StartFrameReaderAsyncWrapper() {
        await StartFrameReaderAsync();
    }

    async void StopFrameReaderAsyncWrapper() {
        await StopFrameReaderAsync();
    }

    // Update is called once per frame
    void Update() {

    }

    void Start() {
        captureStatus = CaptureStatus.Clean;
        InitializeMediaCaptureAsyncWrapper();
    }

    void OnApplicationQuit() {
        if (captureStatus == CaptureStatus.Running) {
            StopFrameReaderAsyncWrapper();
        }
    }

    public void OnTapped() {
        if (captureStatus == CaptureStatus.Initialized) {
            StartFrameReaderAsyncWrapper();
        }
        else if (captureStatus == CaptureStatus.Running) {
            StopFrameReaderAsyncWrapper();
        }
    }

    public byte[] read() {
        if(upBitmap == null)
            return null;

        SoftwareBitmap tmp_bitmap = null;
        Interlocked.Exchange(ref tmp_bitmap, upBitmap);
        
        byte[] img_bin = new byte[videoWidth * videoHeight * 4];
        
        tmp_bitmap.CopyToBuffer(img_bin.AsBuffer());
        tmp_bitmap.Dispose();
        
        return img_bin;
    }

    public int GetWidth()
    {
        return videoWidth;
    }

    public int GetHeight()
    {
        return videoHeight;
    }

#endif
}