using System;
using UnityEngine;

public class Box : MonoBehaviour
{
    [SerializeField]
    private float _maxHealth = 100f;
    [SerializeField]
    private float _damageThreshold = 5f;
    [SerializeField]
    private float _damageMultiplier = 0.5f;

    private DeliveryCargo _deliveryCargo;
    private float _dropTime;
    private float _pickupDelay = 1.5f;
    private float _currentHealth;

    public Action<Box> OnDropped;
    public Action<Box> OnDestroyed;

    public float HealthPercent { get { return _currentHealth / _maxHealth; } }

    public void Initialize(DeliveryCargo deliveryCargo)
    {
        _deliveryCargo = deliveryCargo;
        _currentHealth = _maxHealth;
        Debug.Log($"Box initialized with {_currentHealth} health.");
    }

    private void OnJointBreak(float breakForce)
    {
        _dropTime = Time.time;
        OnDropped?.Invoke(this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("Player") && Time.time - _dropTime > _pickupDelay)
        {
            _deliveryCargo.Reattach();
            return;
        }

        float impact = collision.impulse.magnitude;
        if (impact > _damageThreshold)
            TakeDamage(impact * _damageMultiplier);
    }

    private void TakeDamage(float damage)
    {
        _currentHealth = Mathf.Max(0, _currentHealth - damage);

        DeliveryEvents.OnDamaged?.Invoke(this, _currentHealth);

        if (_currentHealth <= 0)
            OnDestroyed?.Invoke(this);
    }
}
