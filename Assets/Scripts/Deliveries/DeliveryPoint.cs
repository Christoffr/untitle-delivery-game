using System;
using UnityEngine;

[System.Serializable]
public class DeliveryPoint : MonoBehaviour
{

    [SerializeField]
    private string _locationName;
    [SerializeField]
    private GameObject _marker;

    public string Location => _locationName;
    public Action<DeliveryPoint> OnPlayerArrived;

    // This method can be called to show or hide the marker for this delivery point
    public void SetMarker(bool active)
    {
        _marker.SetActive(active);
    }

    public void OnTriggerEnter(Collider other)
    {
        // Check if the player has entered the delivery point
        if (other.CompareTag("Player"))
        {
            // Notify the Player that they have reached the delivery point
            OnPlayerArrived?.Invoke(this);
        }
    }
}