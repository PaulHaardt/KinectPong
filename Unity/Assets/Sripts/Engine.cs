using UnityEngine;
using UnityEngine.UI;
using System.Threading.Tasks;

public class Engine : MonoBehaviour
{
    public GameObject ball;
    public GameObject Canvas;
    public GameObject paddlesLeft;
    public GameObject paddlesRight;
    public GameObject Warning;
    public void ClickStart()
    {
        Instantiate(ball, Canvas.transform);
        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, -50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, -50000, 0);
    }

    async public void ClickCalibrate()
    {
        paddlesLeft.SetActive(false);
        paddlesRight.SetActive(false);

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, -50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, -50000, 0);

        Warning.SetActive(true);

        await Task.Delay(10000);

        //while (true)
        {
            
        }
        // Do your things

        paddlesLeft.SetActive(true);
        paddlesRight.SetActive(true);

        Warning.SetActive(false);

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, 50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, 50000, 0);
    }
}
