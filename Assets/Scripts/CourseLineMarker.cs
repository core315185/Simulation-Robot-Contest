using UnityEngine;

namespace SimulationRobotContest
{
    /// <summary>
    /// スタートライン／ゴールラインに付与し、「どちら向きに通過したら有効か」を
    /// RobotController（ゴール判定・スタート判定）が参照するための進行方向ベクトルを保持する。
    /// CourseGeneratorが自動生成する際にアタッチする。
    /// </summary>
    public class CourseLineMarker : MonoBehaviour
    {
        [Tooltip("このラインを正しく通過する際の進行方向（ワールド座標、XZ平面）")]
        [SerializeField]
        private Vector3 travelDirection = Vector3.forward;

        [Tooltip("進行方向に対して直交する横方向の許容半幅 [m]（通路幅の半分）。"
            + "ラインが引かれた通路の外（別の場所で進行方向の座標だけ一致してしまうケース）を"
            + "誤って判定しないようにするために使用する")]
        [SerializeField]
        private float lateralHalfWidth = 0.3f;

        /// <summary>このラインを正しく通過する際の進行方向</summary>
        public Vector3 TravelDirection
        {
            get => travelDirection;
            set => travelDirection = value;
        }

        /// <summary>進行方向に対して直交する横方向の許容半幅 [m]</summary>
        public float LateralHalfWidth
        {
            get => lateralHalfWidth;
            set => lateralHalfWidth = value;
        }
    }
}
