using System;
using UnityEngine;

public class DeliveryCargo : MonoBehaviour
{
    [SerializeField] 
    private GameObject _cargoPrefab;
    [SerializeField]
    private Transform _attachPoint;
    [SerializeField]
    private Rigidbody _carRb;

    private Box _box;

    private GameObject _currentCargo;
    private FixedJoint _joint;
    
    public Action<Box> OnCargoLost;
    public Action<Box> OnCargoReattached;
    public Action<Box> OnCargoDestroyed;

    // This method spawns the cargo and attaches it to the car using a FixedJoint
    public void SpawnAndAttach()
    {
        if (_currentCargo != null) return;

        _currentCargo = Instantiate(_cargoPrefab, _attachPoint.position, _attachPoint.rotation);

        _box = _currentCargo.GetComponent<Box>();
        _box.Initialize(this);
        _box.OnDropped += HandleCargoDropped;
        _box.OnDestroyed += HandleCargoDestroyed;

        _currentCargo.GetComponent<Rigidbody>().isKinematic = true;

        _joint = _currentCargo.AddComponent<FixedJoint>();
        _joint.connectedBody = _carRb;
        _joint.breakForce = 500;
        _joint.breakTorque = 500;

        _currentCargo.GetComponent<Rigidbody>().isKinematic = false;
    }

    // This method can be called to reattach the cargo to the car after it has been dropped
    public void Reattach()
    {
        if (_currentCargo == null) return;

        _currentCargo.GetComponent<Rigidbody>().isKinematic = true;

        _currentCargo.transform.SetPositionAndRotation(_attachPoint.position, _attachPoint.rotation);

        _joint = _currentCargo.AddComponent<FixedJoint>();

        _joint.connectedBody = _carRb;
        _joint.breakForce = 500;
        _joint.breakTorque = 500;

        OnCargoReattached?.Invoke(_box);

        _currentCargo.GetComponent<Rigidbody>().isKinematic = false;
    }

    private void HandleCargoDropped(Box box)
    {
        if (_currentCargo == null) return;
        if (_joint != null)
            Destroy(_joint);
        OnCargoLost?.Invoke(_box);
    }

    private void HandleCargoDestroyed(Box box)
    {
        box.OnDropped -= HandleCargoDropped;
        box.OnDestroyed -= HandleCargoDestroyed;
        if (_joint != null) Destroy(_joint);
        Destroy(_currentCargo);
        _currentCargo = null;
        OnCargoDestroyed?.Invoke(box);
    }

    // This method can be called to deliver the cargo, which destroys it and clears the reference
    public void Deliver()
    {
        if (_currentCargo == null) return;

        _box.OnDropped -= HandleCargoDropped;

        if (_joint != null)
            Destroy(_joint);

        Destroy(_currentCargo);
        _currentCargo = null;
    }
}