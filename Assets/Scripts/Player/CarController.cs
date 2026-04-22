using UnityEngine;

public class CarController : MonoBehaviour
{
    // Struct for caching wheel hit information to avoid redundant raycasts in each method
    private struct WheelHit
    {
        public Transform transform;
        public bool isGrounded;
        public RaycastHit hit;
    }

    [Header("References")]
    [SerializeField] private Rigidbody _carRigidBody;
    [SerializeField] private Transform[] _tireTransforms;
    [SerializeField] private Transform[] _frontTireTransforms;
    [SerializeField] private LayerMask _drivableLayer;

    [Header("Suspension Settings")]
    [SerializeField] private float _suspensionRestDist = 0.7f;
    [SerializeField] private float _springStrength = 15000f;
    [SerializeField] private float _springDamper = 500f;

    [Header("Steering Settings")]
    [SerializeField] private float _tireGripFactor = 0.8f;
    [SerializeField] private float _tireMass = 20f;
    [SerializeField] private float _maxSteerAngle = 30f;

    [Header("Acceleration Settings")]
    [SerializeField] private float _topSpeedMS = 30f;
    [SerializeField] private AnimationCurve _powerCurve;
    [SerializeField] private float _brakeForce = 8000f;
    [SerializeField] private float _enginePower = 5000f;

    [Header("Air Control")]
    [SerializeField] private float _airTorque = 100f;
    [SerializeField] private float _airLiftForce = 10f;
    [SerializeField] private float _gravityModifier = 0.7f;

    private float _steerInput;
    private float _accelerationInput;
    private bool _isGrounded = false;
    private WheelHit[] _wheelHits;

    private void Update()
    {
        _steerInput = Input.GetAxis("Horizontal");
        _accelerationInput = Input.GetAxis("Vertical");
    }

    private void FixedUpdate()
    {
        CacheWheelHits();

        Suspension();
        Steering();
        Acceleration();
        ApplySteeringRotation();

        if (!_isGrounded)
        {
            Aircontrol();
        }
    }

    private void Suspension()
    {
        foreach (var wheel in _wheelHits)
        {
            if (!wheel.isGrounded) continue;

            // World-space direction of the spring force
            Vector3 springDir = wheel.transform.up;

            // World-space velocity of this tire
            Vector3 tireWorldVel = _carRigidBody.GetPointVelocity(wheel.transform.position);

            // Calculate offset from the raycast
            float offset = _suspensionRestDist - wheel.hit.distance;

            // Calculate velocity along the spring direction
            float vel = Vector3.Dot(springDir, tireWorldVel);

            // Calculate the magnitude of the dampened spring force
            float force = (offset * _springStrength) - (vel * _springDamper);
            // Prevent spring force from going negative (which would cause the spring to "flip" and start pulling instead of pushing)
            force = Mathf.Max(0, force);

            // Apply the force at the location of this tire
            _carRigidBody.AddForceAtPosition(springDir * force, wheel.transform.position);

            Debug.DrawLine(wheel.transform.position, wheel.hit.point, Color.red);
        }
    }

    private void Steering()
    {
        foreach (var wheel in _wheelHits)
        {
            if (!wheel.isGrounded) continue;

            // World-space direction of the steering force (tire's right direction)
            Vector3 steeringDir = wheel.transform.right;

            // World-space velocity of the tire
            Vector3 tireWorldVel = _carRigidBody.GetPointVelocity(wheel.transform.position);

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
            _carRigidBody.AddForceAtPosition(steeringDir * _tireMass * desiredAccel, wheel.transform.position);

            // Debug visualization
            Debug.DrawLine(wheel.transform.position, wheel.transform.position + steeringDir * steeringVel, Color.blue);
        }
    }

    private void Acceleration()
    {
        foreach (var wheel in _wheelHits)
        {
            if (!wheel.isGrounded) continue;
            
            // World-space direction of the acceleration/braking force
            Vector3 accelDir = wheel.transform.forward;

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
                    float availableTorque = _powerCurve.Evaluate(normalizedSpeed) * _enginePower * _accelerationInput;

                    _carRigidBody.AddForceAtPosition(accelDir * availableTorque, wheel.transform.position);
                }
            }
            // Braking
            else if (_accelerationInput < 0.0f)
            {
                _carRigidBody.AddForceAtPosition(accelDir * _accelerationInput * _brakeForce, wheel.transform.position);
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
            var target = Quaternion.Euler(0, targetSteerAngle, 0);
            frontTire.localRotation = Quaternion.Slerp(frontTire.localRotation, target, Time.fixedDeltaTime * 10f);
        }
    }

    private void Aircontrol()
    {
        _carRigidBody.AddForce(Vector3.up * _airLiftForce, ForceMode.Impulse);
        _carRigidBody.AddForce(Physics.gravity * _gravityModifier, ForceMode.Acceleration);

        _carRigidBody.AddTorque(transform.right * _accelerationInput * _airTorque, ForceMode.Impulse);
        _carRigidBody.AddTorque(transform.up * _steerInput * _airTorque, ForceMode.Impulse);
    }

    private void CacheWheelHits()
    {
        _isGrounded = false;

        if (_wheelHits == null || _wheelHits.Length != _tireTransforms.Length)
            _wheelHits = new WheelHit[_tireTransforms.Length];

        for (int i = 0; i < _tireTransforms.Length; i++)
        {
            var tire = _tireTransforms[i];

            _wheelHits[i].transform = tire;
            _wheelHits[i].isGrounded = Physics.Raycast(tire.position, -tire.up, out _wheelHits[i].hit, _suspensionRestDist + 1f, _drivableLayer);

               if (_wheelHits[i].isGrounded)
                    _isGrounded = true;
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