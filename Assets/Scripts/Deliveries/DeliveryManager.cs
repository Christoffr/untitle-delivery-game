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
    [SerializeField] private GameObject _boxPrefab;
    [SerializeField] private Transform _boxMount;
    [SerializeField] private Rigidbody _carRigidbody;
    private GameObject _boxInstance;

    private void Start()
    {
        _carRigidbody = GetComponent<Rigidbody>();

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
            SpawnBox();
        }
        // Ended the delivery
        else if (deliveryPoint == _currentDelivery.End && _boxInstance != null)
        {
            _currentDelivery.End.SetVisual(false);
            _currentDelivery = null;
            _arrow.LookAtTarget = null;
            // Box diappear
            RemoveBox();
        }

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

    private void SpawnBox()
    {
        if (_boxInstance != null) return;

        _boxInstance = Instantiate(_boxPrefab, _boxMount.position, _boxMount.rotation);

        Rigidbody boxRb = _boxInstance.GetComponent<Rigidbody>();

        FixedJoint joint = _boxInstance.GetComponent<FixedJoint>();
        joint.connectedBody = _carRigidbody;
        joint.breakForce = 600f;
        joint.breakTorque = 600f;
    }

    private void RemoveBox()
    {
        if (_boxInstance != null)
        {
            Destroy(_boxInstance);
            _boxInstance = null;
        }
    }
}
