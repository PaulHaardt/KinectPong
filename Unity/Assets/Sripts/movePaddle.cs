using UnityEngine;

public class movePaddle : MonoBehaviour
{
    public float speed;
    public GameObject[] paddles;
    public Canvas canvas; // Add canvas reference

    private float originY;
    private float canvasHeight;
    private float paddleHeight;
    private float topLimit;
    private float bottomLimit;
    private float normalizedSpeed;

    void Start()
    {
        originY = paddles[0].transform.position.y;
        
        // Get canvas dimensions
        RectTransform canvasRect = canvas.GetComponent<RectTransform>();
        canvasHeight = canvasRect.rect.height;
        
        // Get paddle height (assuming paddles have RectTransform or Collider)
        RectTransform paddleRect = paddles[0].GetComponent<RectTransform>();
        paddleHeight = paddleRect != null ? paddleRect.rect.height : 
                      paddles[0].GetComponent<Collider2D>().bounds.size.y;
        
        // Calculate limits with paddle height consideration
        topLimit = originY + (canvasHeight / 2) - (paddleHeight / 2);
        bottomLimit = originY - (canvasHeight / 2) + (paddleHeight / 2);
        
        // Normalize speed based on canvas height (reference height = 200)
        normalizedSpeed = speed * (canvasHeight / 200f);
    }

    private void Update()
    {
        if (Input.GetKey(KeyCode.W))
        {
            MovePaddlesUp();
        }
        if (Input.GetKey(KeyCode.S))
        {
            MovePaddlesDown();
        }
    }
    
    private void MovePaddlesUp()
    {
        foreach (GameObject paddle in paddles)
        {
            if (paddle.transform.position.y < topLimit)
            {
                paddle.transform.position += Vector3.up * (normalizedSpeed * Time.deltaTime);
                
                // Clamp to prevent overshooting
                if (paddle.transform.position.y > topLimit)
                {
                    Vector3 pos = paddle.transform.position;
                    pos.y = topLimit;
                    paddle.transform.position = pos;
                }
            }
        }
    }
    
    private void MovePaddlesDown()
    {
        foreach (GameObject paddle in paddles)
        {
            if (paddle.transform.position.y > bottomLimit)
            {
                paddle.transform.position -= Vector3.up * (normalizedSpeed * Time.deltaTime);
                
                // Clamp to prevent overshooting
                if (paddle.transform.position.y < bottomLimit)
                {
                    Vector3 pos = paddle.transform.position;
                    pos.y = bottomLimit;
                    paddle.transform.position = pos;
                }
            }
        }
    }
}