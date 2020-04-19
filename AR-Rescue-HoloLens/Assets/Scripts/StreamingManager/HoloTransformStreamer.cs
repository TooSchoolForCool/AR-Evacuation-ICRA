using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using System.Linq;
using HoloToolkit.Unity;

public class HoloTransformStreamer : MonoBehaviour
{
    private TcpSender tcp_sender_;
    private Serializer serializer_;

    private float pub_frequency_ = 0.3f;

    // Use this for initialization
    void Start()
    {
        tcp_sender_ = GetComponent<TcpSender>();
        serializer_ = GetComponent<Serializer>();

        // send camera tf in 0 second every pub_frequency_ second
        InvokeRepeating("SendCamTransform", 0.0f, pub_frequency_);
    }

    // Update is called once per frame
    void Update()
    {

    }

    void SendCamTransform()
    {
#if !UNITY_EDITOR
        Vector3 cam_pos = CameraCache.Main.transform.position;
        Vector3 cam_rot = CameraCache.Main.transform.eulerAngles;

        byte[] pos_bin = serializer_.Serialize(cam_pos);
        byte[] rot_bin = serializer_.Serialize(cam_rot);

        byte[] data = pos_bin.Concat(rot_bin).ToArray();
        tcp_sender_.SendData(data);
#endif
    }
}
