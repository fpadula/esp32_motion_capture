using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System.Threading;

public class ReadRotation : MonoBehaviour
{
    public enum Quat_ref{ w, x, y, z}
    public enum Dir{ none = 0, normal=1, inverse=-1}
    public Quat_ref w_ref,x_ref,y_ref,z_ref;
    public Dir w_dir,x_dir,y_dir,z_dir;    
    public float w, x, y, z, lerp;

    private Rigidbody rb;    

    private IPEndPoint ip;
    private Thread data_reader;
    private bool read_data;
    private float[] q; 
    private Quaternion r0; 


    public int port;
    // Start is called before the first frame update
    void Start(){
        this.ip = new IPEndPoint(IPAddress.Any, port);

        this.rb = this.GetComponent<Rigidbody>();                
        r0 = new Quaternion();
        r0.w = this.transform.rotation.w;
        r0.x = this.transform.rotation.x;
        r0.y = this.transform.rotation.y;
        r0.z = this.transform.rotation.z;
        this.data_reader = new Thread(new ThreadStart(ReadRemoteIMU));  
        this.data_reader.Start();
        this.read_data = true;

        this.q = new float[4];
    }

    void OnDestroy(){
        this.read_data = false;
    }

    // Update is called once per frame
    void Update(){
        q[(int)w_ref] = this.w*(int)w_dir;
        q[(int)x_ref] = this.x*(int)x_dir;
        q[(int)y_ref] = this.y*(int)y_dir;
        q[(int)z_ref] = this.z*(int)z_dir;

        Quaternion target = r0 * new Quaternion(q[0], q[1], q[2], q[3]);

        this.transform.rotation = Quaternion.Lerp(this.transform.rotation, target, lerp);        
        // this.transform.rotation = new Quaternion(this.w, this.x, this.y, this.z) * r0;        
    }

    public void ReadRemoteIMU(){
        byte[] bytes;                
        UdpClient client = new UdpClient(port);        
        while (this.read_data){         
            bytes = client.Receive(ref ip);           
            this.x = System.BitConverter.ToSingle(bytes, 0);
            this.y = System.BitConverter.ToSingle(bytes, 4);
            this.z = System.BitConverter.ToSingle(bytes, 8);
            this.w = System.BitConverter.ToSingle(bytes, 12);         
        }        
    }
}
