using System;
using System.Collections.Generic;
using UnityEngine;

public class DeliveryBoard : MonoBehaviour
{
    [SerializeField] 
    private int _maxDeliveries = 3;
    [SerializeField]
    private float _deliveryGenerationInterval = 30f;
    [SerializeField] 
    private List<DeliveryPoint> _deliveryPoints = new();
    [SerializeField] 
    private List<Delivery> _availableDeliveries = new();

    public Action<Delivery> OnDeliveryGenerated;

    private float _currentTime;
    private Delivery _activeDelivery;

    public void Update()
    {
        if (TimeToGenerateDelivery() || NoDeliveries())
        {
            GenerateDelivery();
        }
    }

    public bool TryAcceptDelivery(Delivery delivery)
    {
        if (!_availableDeliveries.Contains(delivery)) return false;
        _availableDeliveries.Remove(delivery);
        return true;
    }

    private void GenerateDelivery()
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

    private bool TimeToGenerateDelivery()
    {
        // Implement logic to determine when to generate a new delivery
        // For example, you could use a timer or check if the board is empty
        _currentTime += Time.deltaTime;
        if (_currentTime >= _deliveryGenerationInterval)
        {
            _currentTime = 0;
            return true;
        }
        return false;
    }

    private bool NoDeliveries()
    {
        return _availableDeliveries.Count == 0;
    }
}