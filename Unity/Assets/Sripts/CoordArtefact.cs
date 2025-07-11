using UnityEngine;
using UnityEngine.UI;

public class CoordArtefact : MonoBehaviour
{
    public Image image;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private void Awake()
    {
        image = GetComponent<Image>();
        if (image == null)
        {
            Debug.LogError("Image component not found on the GameObject.");
            return;
        }

        image.color = new Color(image.color.r, image.color.g, image.color.b, 1f);
    }

    // Update is called once per frame
    void Update()
    {
        if (image.color.a == 0)
        {
            Destroy(gameObject);
        }
        else
        {
            var color = image.color;
            color.a -= Time.deltaTime * 0.5f;
            image.color = color; // Decrease alpha over time
        }
    }
}
