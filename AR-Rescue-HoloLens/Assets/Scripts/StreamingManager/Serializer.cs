using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Serializer : MonoBehaviour {

    public byte[] Serialize(Vector3 vec)
    {
        byte[] data = new byte[sizeof(float) * 3];

        for (int i = 0; i < 3; i++)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(vec[i]), 0, data,
                i * sizeof(float), sizeof(float));
        }

        // transform.position (x, y, z)
        return data;
    }

    public byte[] Serialize(Quaternion quat)
    {
        byte[] data = new byte[sizeof(float) * 4];

        for (int i = 0; i < 4; i++)
        {
            Buffer.BlockCopy(BitConverter.GetBytes(quat[i]), 0, data,
                i * sizeof(float), sizeof(float));
        }

        // transform.quaternion (x, y, z, w)
        return data;
    }
}
