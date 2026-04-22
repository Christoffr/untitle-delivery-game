using UnityEngine;

public class GUIControl : MonoBehaviour
{
    [SerializeField]
    private GameObject _panel;

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            _panel.SetActive(!_panel.activeSelf);
        }
    }
}
