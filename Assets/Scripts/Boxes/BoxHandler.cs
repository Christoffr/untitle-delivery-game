using System;
using UnityEngine;

public class BoxHandler : MonoBehaviour
{
    public Action<Box> OnBoxDropped;
    public Action<Box> OnBoxDestroid;
    public bool HasBoxAttached => _boxInstance != null && _boxInstance.GetComponent<Box>().IsAttached;

    [SerializeField] private GameObject _boxPrefab;
    [SerializeField] private Transform _boxMount;
    private Box _box;

    private Rigidbody _carRigidbody;
    private GameObject _boxInstance;

    public void Init(Rigidbody carRigidbody)
    {
        _carRigidbody = carRigidbody;
    }

    public void SpawnBox(DeliveryManager manager)
    {
        if (_boxInstance != null) return;

        _boxInstance = Instantiate(_boxPrefab, _boxMount.position, _boxMount.rotation);

        Rigidbody rb = _boxInstance.GetComponent<Rigidbody>();

        FixedJoint joint = _boxInstance.GetComponent<FixedJoint>();
        joint.connectedBody = _carRigidbody;
        joint.breakForce = 400f;
        joint.breakTorque = 400f;

        _box = _boxInstance.GetComponent<Box>();

        _box.Init(manager);
        _box.OnDropped += HandleBoxDropped;
        _box.OnDestroied += HandleBoxDestroid;
    }

    public void RemoveBox()
    {
        if (_boxInstance != null)
        {
            Destroy(_boxInstance);
            _boxInstance = null;
        }
    }

    public void ReattachBox(Box box)
    {
        _boxInstance = box.gameObject;

        _boxInstance.GetComponent<Box>().SetAttached();

        Rigidbody rb = _boxInstance.GetComponent<Rigidbody>();

        _boxInstance.transform.position = _boxMount.position;
        _boxInstance.transform.rotation = _boxMount.rotation;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        FixedJoint joint = _boxInstance.AddComponent<FixedJoint>();
        joint.connectedBody = _carRigidbody;
        joint.breakForce = 400f;
        joint.breakTorque = 400f;
    }

    private void HandleBoxDropped(Box box)
    {
        OnBoxDropped?.Invoke(box);
    }
    private void HandleBoxDestroid(Box box)
    {
        OnBoxDestroid?.Invoke(box);
    }
}
