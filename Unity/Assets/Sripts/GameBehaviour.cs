using UnityEngine;
using UnityEngine.UI;
using System.Threading.Tasks;

//a
//1

public class GameBehaviour : MonoBehaviour
{
    public Rigidbody2D rb;
    bool isFrozen = false;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private Text Text1;
    private Text Text2;
    private Text Timer;
    private GameObject restart;
    private float targetTime = 30;
    public int RedScore = 0;
    public int BlueScore = 0;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Text1 = GameObject.FindGameObjectWithTag("Red").GetComponent<Text>();
        Text2 = GameObject.FindGameObjectWithTag("Blue").GetComponent<Text>();
        Timer = GameObject.FindGameObjectWithTag("Timer").GetComponent<Text>();
        restart = GameObject.FindGameObjectWithTag("Restart");
        Text1.text = "0";
        Text2.text = "0";
        Timer.text = "2:30";
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = new Vector3(80,80,0);
    }

    // Update is called once per frame
    void Update()
    {
        if (isFrozen)
        {
            rb.linearVelocity = new Vector3(0, 0, 0);
        }
        else
        {
            float x = rb.linearVelocity.x;
            float y = rb.linearVelocity.y;
            if (x < 0)
            {
                x = Mathf.Clamp(x, -500, -50);
            }
            else
            {
                x = Mathf.Clamp(x, 50, 500);
            }
            if (y < 0)
            {
                y = Mathf.Clamp(y, -500, -50);
            }
            else
            {
                y = Mathf.Clamp(y, 50, 500);
            }
            rb.linearVelocity = new Vector3(x, y, 0);
        }

        targetTime -= Time.deltaTime;

        if (targetTime <= 0 && !isFrozen)
            Stop();
        else
            Timer.text = ((int)targetTime / 60) + ":" + ((((int)targetTime % 60) < 10) ? "0" : "") + ((int)targetTime % 60);
    }

    async void Stop()
    {
        Debug.Log("HOHOHOHHO");
        isFrozen = true;

        if (RedScore > BlueScore)
        {
            Text1.text = "YOU WIN";
            Text2.text = "YOU LOSE";
            Timer.text = "RED WIN!";
        }
        else if (BlueScore > RedScore)
        {
            Text2.text = "YOU WIN";
            Text1.text = "YOU LOSE";
            Timer.text = "BLUE WIN!";
        }
        else
        {
            Text2.text = "Draw";
            Text1.text = "Draw";
            Timer.text = "It's a draw!";
        }

        await Task.Delay(10000);

        Text1.text = "";
        Text2.text = "";
        Timer.text = "";

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, 220, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, 220, 0);
        await Task.Delay(1000);
        Destroy(this.gameObject);
    }
    
    async void OnTriggerEnter2D(Collider2D collider)
    {
        if (collider.gameObject.tag == "Right")
        {
            transform.position = new Vector3(Screen.width/2,Screen.height/2,0);
            isFrozen = true;
            BlueScore++;
            Text2.text = "" + BlueScore;
            await Task.Delay(1000);
            isFrozen = false;
            rb.linearVelocity = new Vector3(80,80,0);
        }
        else if (collider.gameObject.tag == "Left")
        {
            transform.position = new Vector3(Screen.width/2,Screen.height/2,0);
            isFrozen = true;
            RedScore++;
            Text1.text = "" + RedScore;
            await Task.Delay(1000);
            isFrozen = false;
            rb.linearVelocity = new Vector3(-80,-80,0);

        }
    }
}
