using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ReadRotation : MonoBehaviour
{
    public float w, x, y, z;
    // public float wc, xc, yc, zc;

    private Rigidbody rb;
    private Quaternion r0;
    // Start is called before the first frame update
    void Start()
    {
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
    }

    // Update is called once per frame
    void Update()
    {
        // this.transform.rotation = new Quaternion(this.y, -this.z, -this.x, this.w);
        this.transform.rotation = new Quaternion(this.y, -this.z, -this.x, this.w) * r0;
        // this.rb.rotation = new Quaternion(-this.x, -this.z, -this.y, this.w);
        // this.transform.rotation = new Quaternion(-this.x, -this.z, -this.y, this.w);        
    }
}
