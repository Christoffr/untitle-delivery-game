using JetBrains.Annotations;
using System.Collections.Generic;
using UnityEngine;

public class DeliveryManager : MonoBehaviour
{
    [SerializeField]
    private Arrow _arrow;
    [SerializeField]
    private List<DeliveryPoint> _deliverypoints = new();
    [SerializeField]
    private List<Delivery> _availableDeliveries = new();
    [SerializeField]
    private Delivery _currentDelivery;
    [SerializeField] private BoxHandler _boxHandler;
    [SerializeField] private Rigidbody _carRigidbody;

    private void Start()
    {
        _boxHandler.Init(_carRigidbody);
        _boxHandler.OnBoxDropped += HandleBoxDropped;
        _boxHandler.OnBoxDestroid += HandleBoxDestroid;

        foreach (var point in _deliverypoints)
        {
            point.OnPlayerEnter += HandleDeliveryPointContact;
        }

        if (_availableDeliveries.Count < 1)
        {
            GenerateDelivery();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Q))
        {
            SetCurrentDelivery(_availableDeliveries[0]);
        }
    }

    public void HandleDeliveryPointContact(DeliveryPoint deliveryPoint)
    {
        if (_currentDelivery == null) return;

        // Started the delivery
        if (deliveryPoint == _currentDelivery.Start)
        {
            deliveryPoint.SetVisual(false);
            _currentDelivery.End.SetVisual(true);
            _arrow.LookAtTarget = _currentDelivery.End.transform;
            // Box appear
            _boxHandler.SpawnBox(this);
        }
        // Ended the delivery
        else if (deliveryPoint == _currentDelivery.End && _boxHandler.HasBoxAttached)
        {
            _currentDelivery.End.SetVisual(false);
            _currentDelivery = null;
            _arrow.LookAtTarget = null;
            // Box diappear
            _boxHandler.RemoveBox();
        }
    }

    public void ReattachBox(Box box)
    {
        _boxHandler.ReattachBox(box);
        _arrow.LookAtTarget = _currentDelivery.End.transform;
    }

    private void HandleBoxDropped(Box box)
    {
        _arrow.LookAtTarget = box.transform;
    }

    private void HandleBoxDestroid(Box box)
    {
        _currentDelivery.End.SetVisual(false);
        _currentDelivery = null;

    }

    private void GenerateDelivery()
    {
        int indexA = Random.Range(0, _deliverypoints.Count);
        int indexB = Random.Range(0, _deliverypoints.Count - 1);

        if (indexB >= indexA)
        {
            indexB += 1;
        }

        DeliveryPoint start = _deliverypoints[indexA];
        DeliveryPoint end = _deliverypoints[indexB];

        Delivery delivery = new(start, end, (int)Vector3.Magnitude(end.transform.position - start.transform.position));

        _availableDeliveries.Add(delivery);
    }

    private void SetCurrentDelivery(Delivery delivery)
    {
        _currentDelivery = delivery;
        _currentDelivery.Start.SetVisual(true);
        _arrow.LookAtTarget = _currentDelivery.Start.transform;
    }
}
