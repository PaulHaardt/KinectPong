using UnityEngine;

public class Aura : MonoBehaviour
{
    public GameObject paddle;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = paddle.transform.position;
        // Let's make it spin back and forth
        float angle = Mathf.Sin(Time.time * 2f) * 15f; // Oscillate between -15 and 15 degrees
        transform.rotation = Quaternion.Euler(0, 0, angle);
        // Let's make it scale up and down
        float scale = 1 + Mathf.Sin(Time.time * 2f) * 0.1f; // Scale between 0.9 and 1.1
    }
}
