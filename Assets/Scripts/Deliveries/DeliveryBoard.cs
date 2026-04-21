using System;
using System.Collections.Generic;
using UnityEngine;

public class DeliveryBoard : MonoBehaviour
{
    [SerializeField] 
    private int _maxDeliveries = 3;
    [SerializeField] 
    private List<DeliveryPoint> _deliveryPoints = new();
    [SerializeField] 
    private List<Delivery> _availableDeliveries = new();

    public Action<Delivery> OnDeliveryGenerated;
    public Action OnBoardEmpty;

    public void Update()
    {
        // For testing: Press G to generate a new delivery
        if (Input.GetKeyDown(KeyCode.Q))
        {
            GenerateDelivery();
        }
    }

    public bool TryAcceptDelivery(out Delivery delivery)
    {
        delivery = null;

        if (_availableDeliveries.Count == 0)
        {
            OnBoardEmpty?.Invoke();
            return false;
        }

        delivery = _availableDeliveries[0];
        _availableDeliveries.RemoveAt(0);
        return true;
    }

    public void GenerateDelivery()
    {
        if (_availableDeliveries.Count >= _maxDeliveries) return;
        if (_deliveryPoints.Count < 2) return;

        int startIndex = UnityEngine.Random.Range(0, _deliveryPoints.Count);
        int endIndex;
        do
        {
            endIndex = UnityEngine.Random.Range(0, _deliveryPoints.Count);
        } while (endIndex == startIndex);

        Delivery newDelivery = new Delivery(_deliveryPoints[startIndex], _deliveryPoints[endIndex]);
        _availableDeliveries.Add(newDelivery);
        OnDeliveryGenerated?.Invoke(newDelivery);
    }
}