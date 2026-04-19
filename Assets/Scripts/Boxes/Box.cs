using System;
using UnityEngine;

public class Box : MonoBehaviour
{
    public Action<Box> OnDropped;
    public Action<Box> OnDestroied;
    public bool IsAttached { get; private set; } = true;
    
    [SerializeField] private float _pickupDelay = 1f;
    [SerializeField] private float _damageThreshold = 5f;

    private int _health = 3;

    private float _dropTime;
    private DeliveryManager _deliveryManager;

    public void Init(DeliveryManager deliveryManager)
    {
        _deliveryManager = deliveryManager;
    }

    public void SetAttached()
    {
        IsAttached = true;
    }

    private void OnJointBreak(float breakForce)
    {
        IsAttached = false;
        _dropTime = Time.time;
        OnDropped?.Invoke(this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        if (IsAttached) return;

        bool isPlayer = collision.collider.CompareTag("Player");
        float impactForce = collision.relativeVelocity.magnitude;

        if (!isPlayer && impactForce > _damageThreshold)
        {
            int damage = Mathf.FloorToInt(impactForce / 5f);
            TakeDamage(damage);
        }

        if (isPlayer && Time.time - _dropTime >= _pickupDelay)
        {
            _deliveryManager.ReattachBox(this);
        }
    }

    private void TakeDamage(int amount)
    {
        _health -= amount;

        if (_health <= 0)
        {
            Destroy(gameObject);
            OnDestroied?.Invoke(this);
        }
    }

}
