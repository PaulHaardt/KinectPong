using UnityEngine;
using UnityEngine.UI;

public class CoordArtefact : MonoBehaviour
{
    private Image _image;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    private void Awake()
    {
        _image = GetComponent<Image>();
        if (_image == null)
        {
            Debug.LogError("Image component not found on the GameObject.");
            return;
        }

        _image.color = new Color(_image.color.r, _image.color.g, _image.color.b, 1f);
    }

    // Update is called once per frame
    void Update()
    {
        if (_image.color.a == 0)
        {
            Destroy(gameObject);
        }
        else
        {
            var color = _image.color;
            color.a -= Time.deltaTime * 0.5f;
            _image.color = color; // Decrease alpha over time
        }
    }
}
