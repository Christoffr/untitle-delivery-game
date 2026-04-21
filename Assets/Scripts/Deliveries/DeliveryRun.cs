using UnityEngine;

public enum DeliveryState
{
    Idle,
    HeadingToPickup,
    CargoAttached,
    CargoDropped,
    Completed
}

public class DeliveryRun : MonoBehaviour
{
    [SerializeField] 
    private DeliveryBoard _deliveryBoard;
    [SerializeField] 
    private DeliveryCargo _deliveryCargo;

    private Delivery _delivery;
    private DeliveryState _state = DeliveryState.Idle;

    private void OnEnable()
    {
        _deliveryCargo.OnCargoLost += HandleCargoLost;
        _deliveryCargo.OnCargoReattached += HandleCargoReattached;
        _deliveryCargo.OnCargoDestroyed += HandleCargoDestroyed;
    }

    private void OnDisable()
    {
        _deliveryCargo.OnCargoLost -= HandleCargoLost;
        _deliveryCargo.OnCargoReattached -= HandleCargoReattached;
        _deliveryCargo.OnCargoDestroyed -= HandleCargoDestroyed;
        if (_delivery != null) ClearDelivery();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.E) && _state == DeliveryState.Idle)
            TryAcceptDelivery();
    }

    private void SetState(DeliveryState newState)
    {
        _state = newState;
        DeliveryEvents.OnDeliveryStateChanged?.Invoke(newState);
    }

    private void TryAcceptDelivery()
    {
        if (_deliveryBoard.TryAcceptDelivery(out var delivery))
        {
            if (_delivery != null) ClearDelivery();
            SetDelivery(delivery);
        }
    }

    private void SetDelivery(Delivery delivery)
    {
        _delivery = delivery;
        _delivery.StartPoint.OnPlayerArrived += HandlePlayerArrived;
        _delivery.EndPoint.OnPlayerArrived += HandlePlayerArrived;
        _delivery.StartPoint.SetMarker(true);
        SetState(DeliveryState.HeadingToPickup);
        DeliveryEvents.OnDeliveryAccepted?.Invoke(_delivery);
        DeliveryEvents.OnHeadingToPickup?.Invoke(_delivery.StartPoint);
    }

    private void ClearDelivery()
    {
        _delivery.StartPoint.OnPlayerArrived -= HandlePlayerArrived;
        _delivery.EndPoint.OnPlayerArrived -= HandlePlayerArrived;
        _delivery.StartPoint.SetMarker(false);
        _delivery.EndPoint.SetMarker(false);
        _delivery = null;
        SetState(DeliveryState.Idle);
    }

    private void HandlePlayerArrived(DeliveryPoint point)
    {
        if (_state == DeliveryState.HeadingToPickup && point == _delivery.StartPoint)
        {
            _delivery.StartPoint.SetMarker(false);
            _delivery.EndPoint.SetMarker(true);
            _deliveryCargo.SpawnAndAttach();
            SetState(DeliveryState.CargoAttached);
            DeliveryEvents.OnHeadingToDropoff?.Invoke(_delivery.EndPoint);
        }
        else if (_state == DeliveryState.CargoAttached && point == _delivery.EndPoint)
        {
            _deliveryCargo.Deliver();
            SetState(DeliveryState.Completed);
            DeliveryEvents.OnDeliveryCompleted?.Invoke(_delivery);
            ClearDelivery();
        }
    }

    private void HandleCargoLost(Box box)
    {
        SetState(DeliveryState.CargoDropped);
        DeliveryEvents.OnCargoLost?.Invoke(box);
    }

    private void HandleCargoReattached(Box box)
    {
        SetState(DeliveryState.CargoAttached);
        DeliveryEvents.OnCargoReattached?.Invoke(box);
        DeliveryEvents.OnHeadingToDropoff?.Invoke(_delivery.EndPoint);
    }

    private void HandleCargoDestroyed(Box box)
    {
        DeliveryEvents.OnDeliveryFailed?.Invoke(_delivery);
        ClearDelivery();
    }
}