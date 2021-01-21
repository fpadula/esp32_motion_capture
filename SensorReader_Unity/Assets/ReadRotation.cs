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

    public float w, x, y, z;    
    private float w_acc, x_acc, y_acc, z_acc;    

    private Rigidbody rb;    

    private IPEndPoint ip;
    private Thread data_reader;
    private bool read_data;
    private float[] q; 
    private Quaternion r0; 
    public int packet_count, packets_to_avg;


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
   

        this.w_acc = 0;
        this.x_acc = 0;
        this.y_acc = 0;
        this.z_acc = 0;

        this.q = new float[4];
        this.packet_count = 0;
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
 
        this.transform.rotation = r0 * new Quaternion(q[0], q[1], q[2], q[3]);        
        // this.transform.rotation = new Quaternion(this.w, this.x, this.y, this.z) * r0;        
    }

    public void ReadRemoteIMU(){
        byte[] bytes;                
        UdpClient client = new UdpClient(port);        
        while (this.read_data){         
            bytes = client.Receive(ref ip);                                    
            this.w_acc += ((float)((short)((bytes[0] << 8) | bytes[1]))) / 16384.0f;
            this.x_acc += ((float)((short)((bytes[2] << 8) | bytes[3]))) / 16384.0f;
            this.y_acc += ((float)((short)((bytes[4] << 8) | bytes[5]))) / 16384.0f;
            this.z_acc += ((float)((short)((bytes[6] << 8) | bytes[7]))) / 16384.0f;

            this.packet_count++;
            if(this.packet_count >= this.packets_to_avg){
                this.packet_count = 0;
                this.w = this.w_acc/this.packets_to_avg;
                this.x = this.x_acc/this.packets_to_avg;
                this.y = this.y_acc/this.packets_to_avg;
                this.z = this.z_acc/this.packets_to_avg;
                this.w_acc = 0;
                this.x_acc = 0;
                this.y_acc = 0;
                this.z_acc = 0;
            }            
        }        
    }
}
