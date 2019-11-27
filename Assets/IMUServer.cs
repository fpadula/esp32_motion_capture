using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Net;
using System.Net.Sockets;
using System;
using System.Text;
using System.Threading;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class IMUServer : MonoBehaviour
{    
    private IPEndPoint localEndPoint;
    private bool accept_connections;    
    public GameObject A, B;
    private ReadRotation A_readrotation, B_readrotation, current;
    private Thread t_accept_connections;
    private Stack<Thread> imu_readers_t;

    void Start()
    {
        imu_readers_t = new Stack<Thread>();

        localEndPoint = new IPEndPoint(IPAddress.Parse("0.0.0.0"), 8090);
        accept_connections = true;
        t_accept_connections = new Thread(new ThreadStart(StartServer));        

        t_accept_connections.Start();        

        A_readrotation = A.GetComponent<ReadRotation>();
        B_readrotation = B.GetComponent<ReadRotation>();
    }

    void OnDestroy()
    {
        accept_connections = false;
        // t.Join();
        while(imu_readers_t.Count !=0){
            imu_readers_t.Pop().Abort();
        }
    }
    
    public void ReadRemoteIMU(Socket handler){
        byte[] bytes = new Byte[9];
        float x, y, z, w;
        int bytesRec;
        char type;
                
        while (true)
        {
            if (handler.Poll(1000, SelectMode.SelectRead) && (handler.Available == 0))
            { // Connection not available!
                break;
            }
            // Reading command
            bytesRec = handler.Receive(bytes);
            w = (((float)(((short)(bytes[0] << 8)) | bytes[1]))) / 16384.0f;
            x = (((float)(((short)(bytes[2] << 8)) | bytes[3]))) / 16384.0f;
            y = (((float)(((short)(bytes[4] << 8)) | bytes[5]))) / 16384.0f;
            z = (((float)(((short)(bytes[6] << 8)) | bytes[7]))) / 16384.0f;
            type = (char)bytes[8];            
            if (type == 'a')
            {
                current = A_readrotation;
            }
            else
            {
                current = B_readrotation;
            }
            current.x = x;
            current.y = y;
            current.z = z;
            current.w = w;

        }
        handler.Shutdown(SocketShutdown.Both);
        handler.Close();
    }

    public void StartServer()
    {
        Socket handler;
        //make the server socket, again using TCP as the transmission protocol
        // Socket listener = new Socket(localEndPoint.AddressFamily, SocketType.Stream, ProtocolType.Tcp);
        Socket listener = new Socket(localEndPoint.AddressFamily, SocketType.Dgram, ProtocolType.Udp);
        Thread t;
        //tell the socket that it belongs to our machine by binding it to our IP
        listener.Bind(localEndPoint);

        //listen for a max of 10 new connections
        listener.Listen(1000);

        //wait for a new connection (this is a blocking method)
        Debug.Log("Waiting for a connection...");
        while(accept_connections){
            handler = listener.Accept();
            Debug.Log("Client Connected!");
            // imu_readers_t.Push(new Thread(new ThreadStart(ReadRemoteIMU)));
            t = new Thread(() => ReadRemoteIMU(handler));
            t.Start();
            imu_readers_t.Push(t);
        }
    }
}