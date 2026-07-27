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

        /// <summary>このラインを正しく通過する際の進行方向</summary>
        public Vector3 TravelDirection
        {
            get => travelDirection;
            set => travelDirection = value;
        }
    }
}
