using System;
using UnityEngine;

public class DeliveryScore : MonoBehaviour
{
    [SerializeField]
    private float _perfectDeliveryMultiplier = 1.5f;
    [SerializeField]
    private float _dropPenalty = 0.1f;
    [SerializeField] 
    private float _damageRewardMultiplier = 0.5f;
    [SerializeField] 
    private int _destructionPenalty = 50;

    private Delivery _currentDelivery;
    private float _totalDamage;
    private int _dropCount;

    public Action<DeliveryResult> OnDeliveryResult;

    private void OnEnable()
    {
        DeliveryEvents.OnDeliveryAccepted += HandleDeliveryAccepted;
        DeliveryEvents.OnDeliveryCompleted += HandleDeliveryCompleted;
        DeliveryEvents.OnDeliveryFailed += HandleDeliveryFailed;
        DeliveryEvents.OnDamaged += HandleDamaged;
        DeliveryEvents.OnCargoLost += HandleCargoLost;
    }

    private void OnDisable()
    {
        DeliveryEvents.OnDeliveryAccepted -= HandleDeliveryAccepted;
        DeliveryEvents.OnDeliveryCompleted -= HandleDeliveryCompleted;
        DeliveryEvents.OnDeliveryFailed -= HandleDeliveryFailed;
        DeliveryEvents.OnDamaged -= HandleDamaged;
        DeliveryEvents.OnCargoLost -= HandleCargoLost;
    }

    private void HandleDeliveryAccepted(Delivery delivery)
    {
        _currentDelivery = delivery;
        _totalDamage = 0;
        _dropCount = 0;
    }

    private void HandleDamaged(Box box, float currentHealth)
    {
        if (_currentDelivery == null) return;
        _totalDamage += _currentDelivery.Reward * (1 - box.HealthPercent);
    }

    private void HandleCargoLost(Box box)
    {
        if (_currentDelivery == null) return;
        _dropCount++;
    }

    private void HandleDeliveryCompleted(Delivery delivery)
    {
        float modifier = CalculateModifier();
        int penalty = Mathf.RoundToInt(_totalDamage * _damageRewardMultiplier);
        int baseReward = delivery.Reward;
        int finalReward = Mathf.RoundToInt(Mathf.Max(0, (baseReward - penalty) * modifier));

        OnDeliveryResult?.Invoke(new DeliveryResult(
            success: true,
            baseReward: baseReward,
            damagePenalty: penalty,
            modifier: modifier,
            finalReward: finalReward
        ));

        Debug.Log($"Delivery Completed: Base={baseReward}, DamagePenalty={penalty}, Modifier={modifier:F2}, Final={finalReward}");

        _currentDelivery = null;
    }

    private void HandleDeliveryFailed(Delivery delivery)
    {
        OnDeliveryResult?.Invoke(new DeliveryResult(
            success: false,
            baseReward: delivery.Reward,
            damagePenalty: delivery.Reward,
            modifier: 0,
            finalReward: -_destructionPenalty
        ));

        Debug.Log($"Delivery Failed: Base={delivery.Reward}, DamagePenalty={delivery.Reward}, Modifier=0, Final={-_destructionPenalty}");

        _currentDelivery = null;
    }

    private float CalculateModifier()
    {
        bool isPerfect = _dropCount == 0 && _totalDamage == 0;
        if (isPerfect) return _perfectDeliveryMultiplier;

        float modifier = 1.0f;
        modifier -= _dropCount * _dropPenalty;
        modifier -= (_totalDamage / (_currentDelivery?.Reward ?? 1)) * _damageRewardMultiplier;

        return Mathf.Max(0, modifier);
    }
}