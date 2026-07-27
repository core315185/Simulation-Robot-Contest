using UnityEngine;
using UnityEngine.UI;

namespace SimulationRobotContest
{
    /// <summary>
    /// 走行中の状態（経過時間・速度・加速度・レース状態）を画面上のUI Textへ表示する。
    /// RobotController / CameraControllerの公開プロパティを読み取るのみで、
    /// 制御には一切関与しない（UIは表示専任、SOLID: SRP）。
    /// </summary>
    public class UIManager : MonoBehaviour
    {
        #region Inspector Fields

        [Header("参照")]
        [SerializeField] private RobotController robotController;
        [SerializeField] private CameraController cameraController;

        [Header("UI要素")]
        [Tooltip("経過時間を表示するText")]
        [SerializeField] private Text elapsedTimeText;

        [Tooltip("現在速度を表示するText")]
        [SerializeField] private Text currentSpeedText;

        [Tooltip("現在の水平合成加速度を表示するText")]
        [SerializeField] private Text currentAccelText;

        [Tooltip("最大水平合成加速度を表示するText")]
        [SerializeField] private Text maxAccelText;

        [Tooltip("レース状態を表示するText")]
        [SerializeField] private Text raceStateText;

        [Header("表示フォーマット")]
        [SerializeField] private string timeFormat = "走破時間: {0:F2} s";
        [SerializeField] private string speedFormat = "速度: {0:F2} m/s";
        [SerializeField] private string accelFormat = "加速度: {0:F3} m/s^2";
        [SerializeField] private string maxAccelFormat = "最大加速度: {0:F3} m/s^2";

        #endregion

        #region Unity Lifecycle

        private void Reset()
        {
            robotController = FindFirstObjectByType<RobotController>();
            cameraController = FindFirstObjectByType<CameraController>();
        }

        private void Update()
        {
            if (robotController != null)
            {
                UpdateRobotDisplays();
            }

            if (cameraController != null)
            {
                UpdateCameraDisplays();
            }
        }

        #endregion

        #region Private Methods

        private void UpdateRobotDisplays()
        {
            if (elapsedTimeText != null)
            {
                elapsedTimeText.text = string.Format(timeFormat, robotController.ElapsedTime);
            }

            if (currentSpeedText != null)
            {
                currentSpeedText.text = string.Format(speedFormat, Mathf.Abs(robotController.CurrentForwardSpeed));
            }

            if (raceStateText != null)
            {
                raceStateText.text = BuildRaceStateLabel();
            }
        }

        private void UpdateCameraDisplays()
        {
            if (currentAccelText != null)
            {
                currentAccelText.text = string.Format(accelFormat, cameraController.CurrentHorizontalAcceleration);
            }

            if (maxAccelText != null)
            {
                maxAccelText.text = string.Format(maxAccelFormat, cameraController.MaxObservedHorizontalAcceleration);
            }
        }

        private string BuildRaceStateLabel()
        {
            switch (robotController.CurrentState)
            {
                case RobotController.RaceState.NotStarted:
                    return "状態: スタート待機中";
                case RobotController.RaceState.Running:
                    return "状態: 走行中";
                case RobotController.RaceState.Finished:
                    return "状態: ゴール成功";
                case RobotController.RaceState.Disqualified:
                    return $"状態: 失格（{robotController.DisqualificationReason}）";
                default:
                    return string.Empty;
            }
        }

        #endregion
    }
}
