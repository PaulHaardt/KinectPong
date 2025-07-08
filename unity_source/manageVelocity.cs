using UnityEngine;  
using System.Threading.Tasks;

public class manageVelocity : MonoBehaviour
{
    public Rigidbody2D rb;
    bool isFrozen = false;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = new Vector3(50,50,0);
    }

    void Update()
    {
        if (isFrozen)
        {
            rb.linearVelocity = new Vector3(0,0,0);
            return;
        }
        float x = rb.linearVelocity.x;
        float y = rb.linearVelocity.y;
        if (x < 0)
        {
            x = Mathf.Clamp(x, -300, -20);
        }
        else
        {
            x = Mathf.Clamp(x, 20, 300);
        }
        if (y < 0)
        {
            y = Mathf.Clamp(y, -300, -20);
        }
        else
        {
            y = Mathf.Clamp(y, 20, 300);
        }
        rb.linearVelocity = new Vector3(x, y, 0);
    }

    async void OnTriggerEnter2D(Collider2D collider)
    {
        if (collider.gameObject.tag == "Left")
        {
            transform.position = new Vector3(160,120,0);
            isFrozen = true;
            await Task.Delay(1000);
            isFrozen = false;
            rb.linearVelocity = new Vector3(50,50,0);
        }
        if (collider.gameObject.tag == "Right")
        {
            transform.position = new Vector3(160,120,0);
            isFrozen = true;
            await Task.Delay(1000);
            isFrozen = false;
            rb.linearVelocity = new Vector3(-50,-50,0);

        }
    }
}
