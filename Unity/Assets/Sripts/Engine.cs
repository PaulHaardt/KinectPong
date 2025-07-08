using UnityEngine;

public class Engine : MonoBehaviour
{
    public GameObject ball;
    public GameObject Canvas;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {

    }

    public void ClickStart()
    {
        Instantiate(ball, Canvas.transform);
        GameObject.FindGameObjectWithTag("Restart").transform.position = new Vector3(0,-200,0);
    }
}
