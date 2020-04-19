using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DrawPath : MonoBehaviour
{
    [SerializeField]
    private GameObject line_generator_prefab_;

    private GameObject line_generator;
    private LineRenderer line_rend;

    private TcpServer tcp_server_;

    // Use this for initialization
    void Start()
    {
        tcp_server_ = GetComponent<TcpServer>();

        line_generator = Instantiate(line_generator_prefab_);
        line_rend = line_generator.GetComponent<LineRenderer>();
        line_rend.textureMode = LineTextureMode.Tile;

        // send camera tf in 0 second every x second
        InvokeRepeating("SpawnPath", 0.0f, 0.2f);
    }

    private void SpawnPath()
    {
        // line_rend.positionCount = 3;

        // line_rend.SetPosition(0, new Vector3(0, -1.5f, 0));
        // line_rend.SetPosition(1, new Vector3(0, -1.5f, 2));
        // line_rend.SetPosition(2, new Vector3(2, -1.5f, 2));

#if !UNITY_EDITOR
        if( tcp_server_.IsEmpty() )
            return;

        Vector3[] path = tcp_server_.GetPath();

        line_rend.positionCount = path.Length;
        line_rend.SetPositions(path);
#endif
    }
}
