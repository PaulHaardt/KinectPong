using UnityEngine;

public class movePaddle : MonoBehaviour
{
    public float speed;
    public GameObject[] paddles;

    private float originY;

    void Start()
    {
        originY = paddles[0].transform.position.y;
        Debug.Log(originY);
    }
    // Upd
    void Update()
    {
        if (Input.GetKey(KeyCode.W))
        {
            if (paddles[0].transform.position.y < 93 + originY)
                paddles[0].transform.position += Vector3.up * speed * Time.deltaTime;
            if (paddles[1].transform.position.y < 93 + originY)
                paddles[1].transform.position += Vector3.up * speed * Time.deltaTime;
        }
        if (Input.GetKey(KeyCode.S))
        {
            if (paddles[0].transform.position.y > -93 + originY)
                paddles[0].transform.position -= Vector3.up * speed* Time.deltaTime;
            if (paddles[1].transform.position.y > -93 + originY)
                paddles[1].transform.position -= Vector3.up * speed* Time.deltaTime;
        }
    }
}
