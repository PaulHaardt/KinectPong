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
    private Image TimerRound;
    private GameObject restart;
    private float maxTimer = 150;
    private float targetTime;
    public int RedScore = 0;
    public int BlueScore = 0;

    [Range(100f, 500f)] public float minSpeed = 300f; // Speed of the ball, can be adjusted in the inspector
    [Range(100f, 500f)] public float maxSpeed = 500f; // Maximum speed of the ball, can be adjusted in the inspector

    private enum LastWon
    {
        Red,
        Blue,
        None
    }

    private LastWon lastWon = LastWon.None;

    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        Text1 = GameObject.FindGameObjectWithTag("Red").GetComponent<Text>();
        Text2 = GameObject.FindGameObjectWithTag("Blue").GetComponent<Text>();
        Timer = GameObject.FindGameObjectWithTag("Timer").GetComponent<Text>();
        TimerRound = GameObject.FindGameObjectWithTag("TimerRound").GetComponent<Image>();
        restart = GameObject.FindGameObjectWithTag("Restart");
        Text1.text = "0";
        Text2.text = "0";
        Timer.text = "";
        TimerRound.fillAmount = 1.0f;
        targetTime = maxTimer;
        rb = GetComponent<Rigidbody2D>();
        rb.linearVelocity = GetStartingBallVelocity();
    }

    private Vector3 GetStartingBallVelocity()
    {
        // Let's choose a random angle between 30 and 60 degrees for the ball's initial velocity
        float angle = Random.Range(30f, 60f) * Mathf.Deg2Rad; // Convert degrees to radians
        float speed =
            Random.Range(minSpeed, Mathf.Lerp(minSpeed, maxSpeed, 0.3f)); // Random speed between 300 and maxSpeed
        float x = Mathf.Cos(angle) * speed;
        float y = Mathf.Sin(angle) * speed;
        Vector3 initialVelocity = new Vector3(x, y, 0);

        return lastWon switch
        {
            LastWon.Red => new Vector3(-initialVelocity.x, initialVelocity.y, 0),
            LastWon.Blue => new Vector3(initialVelocity.x, -initialVelocity.y, 0),
            _ => Random.value < 0.5f
                ? new Vector3(initialVelocity.x, initialVelocity.y, 0)
                : new Vector3(-initialVelocity.x, -initialVelocity.y, 0)
        };
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
                x = Mathf.Clamp(x, -maxSpeed, -minSpeed);
            }
            else
            {
                x = Mathf.Clamp(x, minSpeed, maxSpeed);
            }

            if (y < 0)
            {
                y = Mathf.Clamp(y, -maxSpeed, -minSpeed);
            }
            else
            {
                y = Mathf.Clamp(y, minSpeed, maxSpeed);
            }

            rb.linearVelocity = new Vector3(x, y, 0);
        }

        targetTime -= Time.deltaTime;

        if (targetTime <= 0 && !isFrozen)
            Stop();
        else
            TimerRound.fillAmount = targetTime / maxTimer;
    }

    async void Stop()
    {
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

        GameObject.FindGameObjectWithTag("Restart").transform.position += new Vector3(0, 50000, 0);
        GameObject.FindGameObjectWithTag("Calibrate").transform.position += new Vector3(0, 50000, 0);
        await Task.Delay(1000);
        Destroy(this.gameObject);
    }

    async void OnTriggerEnter2D(Collider2D collider)
    {
        if (collider.gameObject.tag == "Right")
        {
            transform.position = new Vector3(Screen.width / 2, Screen.height / 2, 0);
            isFrozen = true;
            BlueScore++;
            Text2.text = "" + BlueScore;
            await Task.Delay(1000);
            isFrozen = false;
            lastWon = LastWon.Blue;
            rb.linearVelocity = GetStartingBallVelocity();
        }
        else if (collider.gameObject.tag == "Left")
        {
            transform.position = new Vector3(Screen.width / 2, Screen.height / 2, 0);
            isFrozen = true;
            RedScore++;
            Text1.text = "" + RedScore;
            await Task.Delay(1000);
            isFrozen = false;
            lastWon = LastWon.Red;
            rb.linearVelocity = GetStartingBallVelocity();
        }
    }
}