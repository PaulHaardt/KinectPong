using UnityEngine;

public class Engine : MonoBehaviour
{
    public GameObject ball;
    public GameObject Canvas;
    public void ClickStart()
    {
        Instantiate(ball, Canvas.transform);
        GameObject.FindGameObjectWithTag("Restart").transform.position = new Vector3(0,-200,0);
    }
}
