using System;
using UnityEngine;

public class DeliveryScore : MonoBehaviour
{
    [SerializeField] 
    private float _damageRewardMultiplier = 0.5f;
    [SerializeField] 
    private int _destructionPenalty = 50;

    private Delivery _currentDelivery;
    private float _totalDamage;

    public Action<DeliveryResult> OnDeliveryResult;

    private void OnEnable()
    {
        DeliveryEvents.OnDeliveryAccepted += HandleDeliveryAccepted;
        DeliveryEvents.OnDeliveryCompleted += HandleDeliveryCompleted;
        DeliveryEvents.OnDeliveryFailed += HandleDeliveryFailed;
        DeliveryEvents.OnDamaged += HandleDamaged;
    }

    private void OnDisable()
    {
        DeliveryEvents.OnDeliveryAccepted -= HandleDeliveryAccepted;
        DeliveryEvents.OnDeliveryCompleted -= HandleDeliveryCompleted;
        DeliveryEvents.OnDeliveryFailed -= HandleDeliveryFailed;
        DeliveryEvents.OnDamaged -= HandleDamaged;
    }

    private void HandleDeliveryAccepted(Delivery delivery)
    {
        _currentDelivery = delivery;
        _totalDamage = 0;
    }

    private void HandleDamaged(Box box, float currentHealth)
    {
        if (_currentDelivery == null) return;
        float damageTaken = box.HealthPercent < 1 ? _currentDelivery.Reward * (1 - box.HealthPercent) : 0;
        _totalDamage += damageTaken;
    }

    private void HandleDeliveryCompleted(Delivery delivery)
    {
        int penalty = Mathf.RoundToInt(_totalDamage * _damageRewardMultiplier);
        int finalReward = Mathf.Max(0, delivery.Reward - penalty);

        OnDeliveryResult?.Invoke(new DeliveryResult(
            success: true,
            baseReward: delivery.Reward,
            damagePenalty: penalty,
            finalReward: finalReward
        ));

        _currentDelivery = null;
    }

    private void HandleDeliveryFailed(Delivery delivery)
    {
        OnDeliveryResult?.Invoke(new DeliveryResult(
            success: false,
            baseReward: delivery.Reward,
            damagePenalty: delivery.Reward,
            finalReward: - _destructionPenalty * Mathf.CeilToInt(delivery.Reward / 100f)
        ));

        _currentDelivery = null;
    }
}