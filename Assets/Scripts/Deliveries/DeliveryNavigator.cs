using UnityEngine;

public class DeliveryNavigator : MonoBehaviour
{
    [SerializeField] 
    private Arrow _arrow;

    private void OnEnable()
    {
        DeliveryEvents.OnHeadingToPickup += HandleHeadingToPickup;
        DeliveryEvents.OnHeadingToDropoff += HandleHeadingToDropoff;
        DeliveryEvents.OnCargoLost += HandleCargoLost;
        DeliveryEvents.OnCargoReattached += HandleCargoReattached;
        DeliveryEvents.OnDeliveryCompleted += HandleDeliveryCompleted;
    }

    private void OnDisable()
    {
        DeliveryEvents.OnHeadingToPickup -= HandleHeadingToPickup;
        DeliveryEvents.OnHeadingToDropoff -= HandleHeadingToDropoff;
        DeliveryEvents.OnCargoLost -= HandleCargoLost;
        DeliveryEvents.OnCargoReattached -= HandleCargoReattached;
        DeliveryEvents.OnDeliveryCompleted -= HandleDeliveryCompleted;
    }

    private void HandleHeadingToPickup(DeliveryPoint point)
    {
        _arrow.SetTarget(point.transform);
    }

    private void HandleHeadingToDropoff(DeliveryPoint point)
    {
        _arrow.SetTarget(point.transform);
    }

    private void HandleCargoLost(Box box)
    {
        if (box != null)
            _arrow.SetTarget(box.transform);
    }

    private void HandleCargoReattached(Box box)
    {
        // DeliveryRun will fire OnHeadingToDropoff immediately after,
        // so the arrow will update on its own
    }

    private void HandleDeliveryCompleted(Delivery delivery)
    {
        _arrow.SetTarget(null);
    }
}
