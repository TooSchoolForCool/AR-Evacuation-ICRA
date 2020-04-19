using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;
using System.Linq;
using HoloToolkit.Unity;


public class CamSensorStreamer : MonoBehaviour {
    private TcpSender tcp_sender_;
    private RawSensorReader sensor_;

    private float pub_frequency_ = 0.3f;

    // Use this for initialization
    void Start () {
        tcp_sender_ = GetComponent<TcpSender>();
        sensor_ = GetComponent<RawSensorReader>();

        // send camera tf in 0 second every pub_frequency_ second
        InvokeRepeating("SendSensorStream", 0.0f, pub_frequency_);
    }

    // Update is called once per frame
    void Update () {
		
	}

    public void OnTapped()
    {
#if !UNITY_EDITOR && UNITY_METRO
        sensor_.OnTapped();
#endif
    }

    private void SendSensorStream()
    {
#if !UNITY_EDITOR && UNITY_METRO
        byte[] img_bin = sensor_.read();
            
        if(img_bin == null)
            return;
            
        int w = sensor_.GetWidth();
        int h = sensor_.GetHeight();
            
        byte[] w_bin = BitConverter.GetBytes(w);
        byte[] h_bin = BitConverter.GetBytes(h);

        byte[] data = w_bin.Concat(h_bin).ToArray();
        data = data.Concat(img_bin).ToArray();

        tcp_sender_.SendData(data);
#endif
    }
}
