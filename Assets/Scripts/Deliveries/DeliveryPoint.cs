using System;
using UnityEngine;

[System.Serializable]
public class DeliveryPoint : MonoBehaviour
{
    [SerializeField] private string _name;
    [SerializeField] private GameObject _visual;

    public Action<DeliveryPoint> OnPlayerEnter;
    public string Name { get { return _name; } }

    public void SetVisual(bool visualBool)
    {
        _visual.SetActive(visualBool);
    }

    private void OnTriggerEnter(Collider collision)
    {
        if (collision.transform.CompareTag("Player"))
        {
            OnPlayerEnter?.Invoke(this);
        }
    }
}
