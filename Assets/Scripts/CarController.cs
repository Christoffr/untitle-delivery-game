using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Rigidbody _carRigidBody;
    [SerializeField] private Transform[] _tireTransforms;
    [SerializeField] private Transform[] _frontTireTransforms;
    [SerializeField] private LayerMask _drivableLayer;

    [Header("Suspension Settings")]
    [SerializeField] private float _suspensionRestDist = 0.7f;
    [SerializeField] private float _springStrength = 30000f;
    [SerializeField] private float _springDamper = 2000f;

    [Header("Steering Settings")]
    [SerializeField] private float _tireGripFactor = 0.8f;
    [SerializeField] private float _tireMass = 20f;
    [SerializeField] private float _maxSteerAngle = 30f;

    [Header("Acceleration Settings")]
    [SerializeField] private float _topSpeedMS = 30f;
    [SerializeField] private AnimationCurve _powerCurve;
    [SerializeField] private float _brakeForce = 8000f;

    private float _steerInput;
    private float _accelerationInput;

    private void Start()
    {
        _carRigidBody = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        _steerInput = Input.GetAxis("Horizontal");
        _accelerationInput = Input.GetAxis("Vertical");
    }

    private void FixedUpdate()
    {
        Suspension();
        Steering();
        Acceleration();
        ApplySteeringRotation();
    }

    private void Suspension()
    {
        foreach (Transform tireTransform in _tireTransforms)
        {
            RaycastHit hit;

            if (Physics.Raycast(tireTransform.position, -tireTransform.up, out hit, _suspensionRestDist + 1f, _drivableLayer))
            {
                // World-space direction of the spring force
                Vector3 springDir = tireTransform.up;

                // World-space velocity of this tire
                Vector3 tireWorldVel = _carRigidBody.GetPointVelocity(tireTransform.position);

                // Calculate offset from the raycast
                float offset = _suspensionRestDist - hit.distance;

                // Calculate velocity along the spring direction
                float vel = Vector3.Dot(springDir, tireWorldVel);

                // Calculate the magnitude of the dampened spring force
                float force = (offset * _springStrength) - (vel * _springDamper);

                // Apply the force at the location of this tire
                _carRigidBody.AddForceAtPosition(springDir * force, tireTransform.position);

                Debug.DrawLine(tireTransform.position, hit.point, Color.red);
            }
            else
            {
                Debug.DrawLine(tireTransform.position, tireTransform.position - tireTransform.up * (_suspensionRestDist + 1f), Color.green);
            }
        }
    }

    private void Steering()
    {
        foreach (Transform tireTransform in _tireTransforms)
        {
            RaycastHit hit;
            if (Physics.Raycast(tireTransform.position, -tireTransform.up, out hit, _suspensionRestDist + 1f, _drivableLayer))
            {
                // World-space direction of the steering force (tire's right direction)
                Vector3 steeringDir = tireTransform.right;

                // World-space velocity of the tire
                Vector3 tireWorldVel = _carRigidBody.GetPointVelocity(tireTransform.position);

                // What is the tire's velocity in the steering direction?
                // steeringDir is a unit vector, so this returns the magnitude of tireWorldVel as projected onto steeringDir
                float steeringVel = Vector3.Dot(steeringDir, tireWorldVel);

                // The change in velocity that we're looking for is -steeringVel * gripFactor
                // gripFactor is in range 0-1, where 0 means no grip, 1 means full grip
                float desiredVelChange = -steeringVel * _tireGripFactor;

                // Turn change in velocity into an acceleration (acceleration = change in vel / time)
                // This will produce the acceleration necessary to change the velocity by desiredVelChange in 1 physics step
                float desiredAccel = desiredVelChange / Time.fixedDeltaTime;

                // Force = Mass * Acceleration, so multiply by the mass of the tire and apply as a force!
                _carRigidBody.AddForceAtPosition(steeringDir * _tireMass * desiredAccel, tireTransform.position);

                // Debug visualization
                Debug.DrawLine(tireTransform.position, tireTransform.position + steeringDir * steeringVel, Color.blue);
            }
        }
    }

    private void Acceleration()
    {
        foreach (Transform tireTransform in _tireTransforms)
        {
            RaycastHit hit;
            if (Physics.Raycast(tireTransform.position, -tireTransform.up, out hit, _suspensionRestDist + 1f, _drivableLayer))
            {
                // World-space direction of the acceleration/braking force
                Vector3 accelDir = tireTransform.forward;

                // Acceleration torque
                if (_accelerationInput > 0.0f)
                {
                    // Forward speed of the car (in the direction of driving)
                    float carSpeed = Vector3.Dot(transform.forward, _carRigidBody.linearVelocity);

                    if (carSpeed < _topSpeedMS)
                    {
                        // Normalized car speed (0 to 1)
                        float normalizedSpeed = Mathf.Clamp01(Mathf.Abs(carSpeed) / _topSpeedMS);

                        // Available torque from the power curve
                        float availableTorque = _powerCurve.Evaluate(normalizedSpeed) * _accelerationInput;

                        _carRigidBody.AddForceAtPosition(accelDir * availableTorque, tireTransform.position);
                    }

                    // DEBUG
                    if (tireTransform == _tireTransforms[0])
                    {
                        Debug.Log($"Speed : {carSpeed}");
                    }
                }
                // Braking
                else if (_accelerationInput < 0.0f)
                {
                    _carRigidBody.AddForceAtPosition(accelDir * _accelerationInput * _brakeForce, tireTransform.position);
                }
            }
        }
    }

    private void ApplySteeringRotation()
    {
        // Rotate the front tire transforms based on steering input
        float targetSteerAngle = _steerInput * _maxSteerAngle;

        foreach (Transform frontTire in _frontTireTransforms)
        {
            // Smoothly rotate the tire transform around its local up axis
            frontTire.localRotation = Quaternion.Euler(0, targetSteerAngle, 0);
        }
    }

    private void OnDrawGizmos()
    {
        if (_tireTransforms == null) return;

        foreach (var tireTransform in _tireTransforms)
        {
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireSphere(tireTransform.position, .1f);
        }

        if (_carRigidBody != null && Application.isPlaying)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(transform.TransformPoint(_carRigidBody.centerOfMass), 0.15f);
        }
    }
}