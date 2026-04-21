using System;
using UnityEngine;

public static class DeliveryEvents
{
    // Delivery lifecycle
    public static Action<Delivery> OnDeliveryAccepted;
    public static Action<Delivery> OnDeliveryCompleted;
    public static Action<DeliveryState> OnDeliveryStateChanged;
    public static Action<Delivery> OnDeliveryFailed;
    public static Action<Box, float> OnDamaged;

    // Navigation
    public static Action<DeliveryPoint> OnHeadingToPickup;
    public static Action<DeliveryPoint> OnHeadingToDropoff;

    // Cargo
    public static Action<Box> OnCargoAttached;
    public static Action<Box> OnCargoLost;
    public static Action<Box> OnCargoReattached;
}
