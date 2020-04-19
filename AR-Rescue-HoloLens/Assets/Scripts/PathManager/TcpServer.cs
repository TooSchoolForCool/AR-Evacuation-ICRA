using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

#if !UNITY_EDITOR
using System.Collections.Generic;
using Windows.Networking;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using Windows.Foundation;
#endif

public class TcpServer : MonoBehaviour
{
#if !UNITY_EDITOR
        private StreamSocketListener listener_;
        private String port_;

        private Queue<Vector3[]> queue_ = new Queue<Vector3[]>();
#endif

    // Use this for initialization
    void Start()
    {
#if !UNITY_EDITOR
        listener_ = new StreamSocketListener();
        port_ = "12346";
        listener_.ConnectionReceived += ListenerConnectionReceived;
        listener_.Control.KeepAlive = false;

        Launch();
#endif
    }

#if !UNITY_EDITOR
    private async void Launch()
    {
        Debug.Log("Listener started");
        try
        {
            await listener_.BindServiceNameAsync(port_);
        }
        catch (Exception e)
        {
            Debug.Log("Error: " + e.Message);
        }

        Debug.Log("Listening");
    }

    private async void ListenerConnectionReceived(StreamSocketListener sender, 
        StreamSocketListenerConnectionReceivedEventArgs args)
    {
        Debug.Log("Connection received");

        try
        {
            using (var reader = new DataReader(args.Socket.InputStream))
            {
                reader.InputStreamOptions = InputStreamOptions.Partial;

                await reader.LoadAsync(sizeof(uint));
                uint dsize = reader.ReadUInt32();

                await reader.LoadAsync(dsize);

                uint len = dsize / sizeof(uint) / 3;
                Vector3[] path = new Vector3[len];
                
                for(int i = 0; i < len; i++)
                {
                    path[i].x = reader.ReadSingle();
                    path[i].y = reader.ReadSingle();
                    path[i].z = reader.ReadSingle();
                }

                queue_.Enqueue(path);
                Debug.Log("received: " + dsize + ", " + len);
                Debug.Log(path[0].ToString());
            }
        }
        catch (Exception e)
        {
            Debug.Log("[ERROR] Disconnected" + e);
        }

        // Debug.Log("Sender disposed");
    }

    public bool IsEmpty(){
        return queue_.Count == 0;
    }

    public Vector3[] GetPath(){
        // skip old data
        while(queue_.Count > 3)
            queue_.Dequeue();

        return queue_.Dequeue();
    }
#endif

    void Update()
    {

    }
}
