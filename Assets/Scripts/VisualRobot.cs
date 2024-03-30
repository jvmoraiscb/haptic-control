using UnityEngine;
public class VisualRobot : MonoBehaviour {
    [SerializeField] private Transform robot;
    [SerializeField] private float lerpSpeed = 10f;
    [SerializeField] private int throttleAngleSpeed = 360;
    [SerializeField] private float steerMaxAngle = 30;
    [SerializeField] private AckermannMiddleware ackermannMid;
    [SerializeField] private GameObject frontRightWheel;
    [SerializeField] private GameObject frontLeftWheel;
    [SerializeField] private GameObject rearRightWheel;
    [SerializeField] private GameObject rearLeftWheel;
    private void FixedUpdate(){
        var lerpPosition = Vector3.Lerp(transform.position, robot.position, lerpSpeed * Time.deltaTime);
        var lerpRotation = Quaternion.Lerp(transform.rotation, robot.rotation, lerpSpeed * Time.deltaTime);
        transform.SetPositionAndRotation(lerpPosition, lerpRotation);
        ThrottleAnimation();
        SteerAnimation();
    }

    private void ThrottleAnimation(){
        var throttleRotation = new Vector3 (throttleAngleSpeed * ackermannMid.Throttle * Time.deltaTime, 0f, 0f);
        frontRightWheel.transform.Rotate(throttleRotation);
        rearRightWheel.transform.Rotate(throttleRotation);
        frontLeftWheel.transform.Rotate(throttleRotation);
        rearLeftWheel.transform.Rotate(throttleRotation);
    }

    private void SteerAnimation(){
        var frontRightRotation = new Vector3(frontRightWheel.transform.rotation.eulerAngles.x, ackermannMid.Throttle > 0f ? -1 * ackermannMid.Steer * steerMaxAngle : ackermannMid.Steer * steerMaxAngle, frontRightWheel.transform.rotation.eulerAngles.z);
        var frontLeftRotation = new Vector3(frontLeftWheel.transform.rotation.eulerAngles.x, ackermannMid.Throttle > 0f ? -1 * ackermannMid.Steer * steerMaxAngle : ackermannMid.Steer * steerMaxAngle, frontLeftWheel.transform.rotation.eulerAngles.z);
        frontRightWheel.transform.localRotation = Quaternion.Euler(frontRightRotation);
        frontLeftWheel.transform.localRotation = Quaternion.Euler(frontLeftRotation);
    }
}