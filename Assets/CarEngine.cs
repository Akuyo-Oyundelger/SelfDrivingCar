using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CarEngine : MonoBehaviour
{
    public Transform path;
    public float maxSteerAngle = 30f;
    public float nodeReachThreshold = 2f;
    public float maxMotorTorque = 50f;
    public float maxBrakeTorque = 150f;
    public float currentSpeed;
    public float maxSpeed = 150f;
    public Vector3 centerOfMass;
    public WheelCollider wheelRL;
    public WheelCollider wheelRR;
    public WheelCollider wheelFL;
    public WheelCollider wheelFR;
    public bool Braking = false;

    [Header("Sensors")]
    public float sensorLength = 5f;
    public float frontSensorPosition = 1.5f;
    public float frontSideSensorPosition = 0.5f;

    private bool isObstacleFront = false;
    private bool isObstacleRight = false;
    private bool isObstacleLeft = false;

    private List<Transform> nodes;
    private int currectNode = 0;

    void Start()
    {
        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }

        Rigidbody rb = GetComponent<Rigidbody>();
        rb.centerOfMass = centerOfMass; // Lower the center of mass
        AdjustWheelFriction();
    }

    private void FixedUpdate()
    {
        Sensors(); // Get sensor data

        // If there's an obstacle in front
        if (isObstacleFront)
        {
            Braking = true; // Apply braking
            if (isObstacleRight && !isObstacleLeft)
            {
                SteerLeft(); // Steer left if there's an obstacle on the right
            }
            else if (isObstacleLeft && !isObstacleRight)
            {
                SteerRight(); // Steer right if there's an obstacle on the left
            }
            else
            {
                SteerLeft(); // Default to steering left if both sides have obstacles
            }
        }
        else
        {
            Braking = false; // No obstacle, continue normal driving
            ApplySteer();
            Drive();
        }

        // Move towards the next node
        CheckWaypointDistance();

        // Apply brakes if needed
        ApplyBraking();

        // Apply anti-roll forces to stabilize the car
        ApplyAntiRoll();
    }

    private void Sensors()
    {
        RaycastHit hit;

        // Reset obstacle flags
        isObstacleFront = false;
        isObstacleRight = false;
        isObstacleLeft = false;

        // Base position for sensors (front bumper)
        Vector3 sensorBasePos = transform.position + transform.up * 0.5f; // Height adjustment
        sensorBasePos += transform.forward * frontSensorPosition; // Move forward

        // 1. Front-center sensor
        if (Physics.Raycast(sensorBasePos, transform.forward, out hit, sensorLength))
        {
            isObstacleFront = true;
            Debug.DrawLine(sensorBasePos, hit.point, Color.red);
        }
        else
        {
            Debug.DrawRay(sensorBasePos, transform.forward * sensorLength, Color.white);
        }

        // 2. Front-right sensor
        Vector3 rightSensorPos = sensorBasePos + transform.right * frontSideSensorPosition;
        if (Physics.Raycast(rightSensorPos, transform.forward, out hit, sensorLength))
        {
            isObstacleRight = true;
            Debug.DrawLine(rightSensorPos, hit.point, Color.red);
        }
        else
        {
            Debug.DrawRay(rightSensorPos, transform.forward * sensorLength, Color.white);
        }

        // 3. Front-left sensor
        Vector3 leftSensorPos = sensorBasePos - transform.right * frontSideSensorPosition;
        if (Physics.Raycast(leftSensorPos, transform.forward, out hit, sensorLength))
        {
            isObstacleLeft = true;
            Debug.DrawLine(leftSensorPos, hit.point, Color.red);
        }
        else
        {
            Debug.DrawRay(leftSensorPos, transform.forward * sensorLength, Color.white);
        }

        // 4. Angled-right sensor (30 degrees)
        Vector3 angledRightPos = sensorBasePos + transform.right * (frontSideSensorPosition * 0.7f);
        Vector3 angledRightDirection = Quaternion.AngleAxis(30, transform.up) * transform.forward;
        if (Physics.Raycast(angledRightPos, angledRightDirection, out hit, sensorLength))
        {
            isObstacleRight = true;
            Debug.DrawLine(angledRightPos, hit.point, Color.blue);
        }
        else
        {
            Debug.DrawRay(angledRightPos, angledRightDirection * sensorLength, Color.cyan);
        }

        // 5. Angled-left sensor (30 degrees)
        Vector3 angledLeftPos = sensorBasePos - transform.right * (frontSideSensorPosition * 0.7f);
        Vector3 angledLeftDirection = Quaternion.AngleAxis(-30, transform.up) * transform.forward;
        if (Physics.Raycast(angledLeftPos, angledLeftDirection, out hit, sensorLength))
        {
            isObstacleLeft = true;
            Debug.DrawLine(angledLeftPos, hit.point, Color.blue);
        }
        else
        {
            Debug.DrawRay(angledLeftPos, angledLeftDirection * sensorLength, Color.cyan);
        }
    }

    private void ApplySteer()
    {
        Vector3 relativeVector = transform.InverseTransformPoint(nodes[currectNode].position);
        float steerAngle = Mathf.Atan2(relativeVector.x, relativeVector.z) * Mathf.Rad2Deg;
        steerAngle = Mathf.Clamp(steerAngle, -maxSteerAngle, maxSteerAngle);

        wheelFL.steerAngle = Mathf.Lerp(wheelFL.steerAngle, steerAngle, Time.deltaTime * 5f);
        wheelFR.steerAngle = Mathf.Lerp(wheelFR.steerAngle, steerAngle, Time.deltaTime * 5f);
    }

    private void SteerLeft()
    {
        wheelFL.steerAngle = -maxSteerAngle;
        wheelFR.steerAngle = -maxSteerAngle;
        Debug.Log("Steering Left to Avoid Obstacle");
    }

    private void SteerRight()
    {
        wheelFL.steerAngle = maxSteerAngle;
        wheelFR.steerAngle = maxSteerAngle;
        Debug.Log("Steering Right to Avoid Obstacle");
    }

    private void Drive()
    {
        float distanceToNode = Vector3.Distance(transform.position, nodes[currectNode].position);

        // Calculate the steering angle
        float steerAngle = Mathf.Abs(wheelFL.steerAngle);

        // Calculate a speed factor based on the steering angle
        float speedFactor = Mathf.Clamp(1f - (steerAngle / maxSteerAngle), 0.5f, 1f);

        // Adjust motor torque based on the distance to the node and steering factor
        float adjustedTorque = Mathf.Lerp(100f, maxMotorTorque, distanceToNode / 10f) * speedFactor;
        adjustedTorque = Mathf.Clamp(adjustedTorque, 50f, maxMotorTorque);

        wheelFL.motorTorque = adjustedTorque;
        wheelFR.motorTorque = adjustedTorque;

        // Apply brake torque if turning sharply
        float brakeFactor = steerAngle / maxSteerAngle;
        wheelRL.brakeTorque = Mathf.Lerp(wheelRL.brakeTorque, maxBrakeTorque * brakeFactor, Time.deltaTime * 5f);
        wheelRR.brakeTorque = Mathf.Lerp(wheelRR.brakeTorque, maxBrakeTorque * brakeFactor, Time.deltaTime * 5f);

        Debug.Log($"Motor Torque: {adjustedTorque}, Brake Torque: {wheelRL.brakeTorque}, Speed Factor: {speedFactor}, Steering: {steerAngle}");
    }

    private void CheckWaypointDistance()
    {
        if (Vector3.Distance(transform.position, nodes[currectNode].position) < nodeReachThreshold)
        {
            currectNode++;
            if (currectNode >= nodes.Count)
            {
                currectNode = 0;
            }
        }
    }

    private void ApplyAntiRoll()
    {
        float antiRollForce = 5000f;

        ApplyAntiRollForce(wheelFL, wheelFR, antiRollForce);
        ApplyAntiRollForce(wheelRL, wheelRR, antiRollForce);
    }

    private void ApplyAntiRollForce(WheelCollider wheelL, WheelCollider wheelR, float antiRollForce)
    {
        WheelHit hit;
        float travelL = 1.0f;
        float travelR = 1.0f;

        if (wheelL.GetGroundHit(out hit))
            travelL = (-wheelL.transform.InverseTransformPoint(hit.point).y - wheelL.radius) / wheelL.suspensionDistance;

        if (wheelR.GetGroundHit(out hit))
            travelR = (-wheelR.transform.InverseTransformPoint(hit.point).y - wheelR.radius) / wheelR.suspensionDistance;

        float force = (travelL - travelR) * antiRollForce;

        if (wheelL.isGrounded)
            GetComponent<Rigidbody>().AddForceAtPosition(wheelL.transform.up * -force, wheelL.transform.position);

        if (wheelR.isGrounded)
            GetComponent<Rigidbody>().AddForceAtPosition(wheelR.transform.up * force, wheelR.transform.position);
    }

    private void AdjustWheelFriction()
    {
        WheelFrictionCurve forwardFriction = new WheelFrictionCurve
        {
            extremumSlip = 0.4f,
            extremumValue = 1f,
            asymptoteSlip = 0.8f,
            asymptoteValue = 0.5f,
            stiffness = 2f
        };

        WheelFrictionCurve sidewaysFriction = new WheelFrictionCurve
        {
            extremumSlip = 0.2f,
            extremumValue = 1f,
            asymptoteSlip = 0.5f,
            asymptoteValue = 0.75f,
            stiffness = 3f
        };

        wheelFL.forwardFriction = forwardFriction;
        wheelFR.forwardFriction = forwardFriction;
        wheelRL.forwardFriction = forwardFriction;
        wheelRR.forwardFriction = forwardFriction;

        wheelFL.sidewaysFriction = sidewaysFriction;
        wheelFR.sidewaysFriction = sidewaysFriction;
        wheelRL.sidewaysFriction = sidewaysFriction;
        wheelRR.sidewaysFriction = sidewaysFriction;
    }

    private void ApplyBraking()
    {
        if (Braking)
        {
            wheelRL.brakeTorque = maxBrakeTorque;
            wheelRR.brakeTorque = maxBrakeTorque;
        }
        else
        {
            wheelRL.brakeTorque = 0;
            wheelRR.brakeTorque = 0;
        }
    }

    private void OnDrawGizmos()
    {
        // Ensure the path exists
        if (path == null) return;

        Transform[] pathTransforms = path.GetComponentsInChildren<Transform>();
        List<Transform> nodes = new List<Transform>();

        for (int i = 0; i < pathTransforms.Length; i++)
        {
            if (pathTransforms[i] != path.transform)
            {
                nodes.Add(pathTransforms[i]);
            }
        }

        // Draw a sphere at each node position
        Gizmos.color = Color.red; // Set color for the nodes
        foreach (Transform node in nodes)
        {
            Gizmos.DrawSphere(node.position, 0.5f); // Adjust size of the sphere
        }

        // Draw lines connecting the nodes
        Gizmos.color = Color.blue; // Set color for the path lines
        for (int i = 0; i < nodes.Count; i++)
        {
            if (i + 1 < nodes.Count)
            {
                Gizmos.DrawLine(nodes[i].position, nodes[i + 1].position);
            }
            else if (nodes.Count > 1) // Connect the last node to the first to complete the loop
            {
                Gizmos.DrawLine(nodes[i].position, nodes[0].position);
            }
        }
    }
}
