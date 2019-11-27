using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class ReadRotation : MonoBehaviour
{
    public float w, x, y, z;    

    private Rigidbody rb;
    private Quaternion r0;

    private IPEndPoint ip;
    private Thread data_reader;

    public int port;
    // Start is called before the first frame update
    void Start(){
        ip = new IPEndPoint(IPAddress.Any, port);

        this.rb = this.GetComponent<Rigidbody>();
        this.w = this.transform.rotation.w;
        this.x = this.transform.rotation.x;
        this.y = this.transform.rotation.y;
        this.z = this.transform.rotation.z;
        r0 = new Quaternion();
        r0.x = this.transform.rotation.x;
        r0.y = this.transform.rotation.y;
        r0.z = this.transform.rotation.z;
        r0.w = this.transform.rotation.w;

        this.data_reader = new Thread(new ThreadStart(ReadRemoteIMU));  
        this.data_reader.Start();
    }

    void OnDestroy(){
        this.data_reader.Abort();        
    }

    // Update is called once per frame
    void Update(){
        // this.transform.rotation = new Quaternion(this.y, -this.z, -this.x, this.w);
        this.transform.rotation = new Quaternion(this.y, -this.z, -this.x, this.w) * r0;
        // this.rb.rotation = new Quaternion(-this.x, -this.z, -this.y, this.w);
        // this.transform.rotation = new Quaternion(-this.x, -this.z, -this.y, this.w);        
    }

    public void ReadRemoteIMU(){
        byte[] bytes;
        float x, y, z, w;
        int bytesRec;
        char type;
        UdpClient client = new UdpClient(port);
        while (true){         
            bytes = client.Receive(ref ip);
            this.w = (((float)(((short)(bytes[0] << 8)) | bytes[1]))) / 16384.0f;
            this.x = (((float)(((short)(bytes[2] << 8)) | bytes[3]))) / 16384.0f;
            this.y = (((float)(((short)(bytes[4] << 8)) | bytes[5]))) / 16384.0f;
            this.z = (((float)(((short)(bytes[6] << 8)) | bytes[7]))) / 16384.0f;
        }        
    }
}
