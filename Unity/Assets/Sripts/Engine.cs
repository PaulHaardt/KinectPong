using UnityEngine;
using UnityEngine.UI;
using System.Threading.Tasks;
using Sripts;

public class Engine : MonoBehaviour
{
    public GameObject ball;
    public GameObject Canvas;
    public GameObject paddlesLeft;
    public GameObject paddlesRight;
    public GameObject Warning;
    public KinectHandTracker KinectHandTracker;
    private GameObject instantiatedBall;
    public void ClickStart()
    {
        instantiatedBall = Instantiate(ball, Canvas.transform);
        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, -50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, -50000, 0);
    }

    public async void ClickCalibrate()
    {
        paddlesLeft.SetActive(false);
        paddlesRight.SetActive(false);

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, -50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, -50000, 0);

        Warning.SetActive(true);
        
        await Task.Delay(3000);
        
        KinectHandTracker.SendUDPMessage("DEPTH_CALIBRATION");

        await Task.Delay(10000);

        paddlesLeft.SetActive(true);
        paddlesRight.SetActive(true);

        Warning.SetActive(false);

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, 50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, 50000, 0);
    }

    public void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space) && !instantiatedBall)
        {
            ClickStart();
            Debug.Log("Ball instantiated");
        }
        
        if (Input.GetKeyDown(KeyCode.C))
        {
            ClickCalibrate();
            Debug.Log("Calibration started");
        }
    }
}
