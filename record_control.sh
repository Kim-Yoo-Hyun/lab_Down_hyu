#!/bin/bash
# AAEON (lab / root) - control bag
#   usage: ./record_control.sh [episode_name]
#   Jetson 의 record_camera.sh 와 같은 episode_name 을 쓸 것 (후처리에서 짝을 찾습니다)
set -u

EP="${1:-episode_$(date +%Y%m%d_%H%M%S)}"
OUTDIR="${KUAVO_DATASET_DIR:-/home/lab/kuavo_dataset/raw}"
MIN_FREE_GB=20

TOPICS=(
  # --- 로봇 실측 상태 (observation) ---
  /sensors_data_raw                 # joint_q / joint_v / joint_torque / imu / FT  <- 실측의 원천
  /dexhand/state
  /drake_ik/real_arm_hand_pose      # 이족: 실제 양손 EE pose
  # --- 로봇 명령 (action) ---
  /joint_cmd                        # 모터로 나간 최종 저수준 명령 (전신)
  /kuavo_arm_traj
  /control_robot_hand_position
  /dexhand/command
  /joint_states                     # 주의: ik_ros_uni 의 팔 목표각(도 단위). 실측 아님
  # --- VR 입력 ---
  /leju_quest_bone_poses
  /quest_joystick_data
  /quest3/triger_arm_mode
  /cmd_torso_pose_vr
  /vr_whole_torso_ctrl
  /robot_head_motion_data
  # --- IK ---
  /ik/two_arm_hand_pose_cmd
  /ik/result
  /ik/result_free
  /drake_ik/eef_pose
  # --- 휠형 전환 대비 (이족에서는 발행자 없음, 그냥 건너뜀) ---
  /mobile_manipulator_eef_poses
  # --- TF ---
  /tf
  /tf_static
)
# 이게 비면 데이터셋이 무의미한 것들
CRITICAL=( /sensors_data_raw /joint_cmd /kuavo_arm_traj )

# ROS setup 이 ROS_MASTER_URI 를 localhost 로 덮어쓰므로 먼저 붙잡아 둔다
_PRE_MASTER="${ROS_MASTER_URI:-}"
_PRE_IP="${ROS_IP:-}"
set +u   # ROS setup 스크립트가 미정의 변수를 참조함
source /opt/ros/noetic/setup.bash
[ -f /home/lab/kuavo-ros-opensource/devel/setup.bash ] && source /home/lab/kuavo-ros-opensource/devel/setup.bash
export ROS_MASTER_URI="${_PRE_MASTER:-http://kuavo_master:11311}"
export ROS_IP="${_PRE_IP:-kuavo_master}"
set -u

echo "=============================================="
echo " CONTROL BAG (AAEON)   episode: $EP"
echo "=============================================="

if ! timeout 5 rostopic list >/dev/null 2>&1; then
  echo "[!] ROS 마스터에 연결 불가 ($ROS_MASTER_URI). 중단."; exit 1
fi

mkdir -p "$OUTDIR" || { echo "[!] $OUTDIR 생성 실패. 중단."; exit 1; }
FREE_GB=$(df -BG --output=avail "$OUTDIR" | tail -1 | tr -dc '0-9')
echo "[i] 저장 위치: $OUTDIR  (${FREE_GB}G 여유)"
[ "$FREE_GB" -lt "$MIN_FREE_GB" ] && { echo "[!] 여유 부족. 중단."; exit 1; }

# --- 발행자 확인 ---
echo "[i] 발행자(publisher) 확인 중..."
LIST=$(rostopic list 2>/dev/null)
MISSING=()
for t in "${TOPICS[@]}"; do
  if echo "$LIST" | grep -qx "$t"; then
    n=$(rostopic info "$t" 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c '^ \*')
    [ "$n" -eq 0 ] && MISSING+=("$t (pub=0)")
  else
    MISSING+=("$t (토픽 없음)")
  fi
done
if [ ${#MISSING[@]} -gt 0 ]; then
  echo "[!] 발행자가 없는 토픽 ${#MISSING[@]}개 - bag 에 빈 채로 남습니다:"
  for m in "${MISSING[@]}"; do echo "      $m"; done
fi
CRIT_BAD=0
for t in "${CRITICAL[@]}"; do
  n=$(rostopic info "$t" 2>/dev/null | sed -n '/Publishers/,/Subscribers/p' | grep -c '^ \*')
  [ "${n:-0}" -eq 0 ] && { echo "[!!] 핵심 토픽에 발행자 없음: $t"; CRIT_BAD=1; }
done
if [ "$CRIT_BAD" = "1" ]; then
  echo "     -> load_kuavo_real.launch / 손 드라이버 / ik_ros_uni 상태를 확인하세요."
  read -r -p "[?] 그래도 녹화할까요? [y/N] " a
  [ "$a" = "y" ] || [ "$a" = "Y" ] || { echo "중단."; exit 1; }
fi

BAG="$OUTDIR/${EP}_control"
echo
echo "[i] 녹화 시작. 종료는 Ctrl+C 를 '한 번만'."
echo "[i] 순서: control 먼저 시작 -> camera 시작 -> ... -> camera 먼저 종료 -> control 종료"
echo
trap 'echo; echo "[i] Ctrl+C 감지 - bag 마무리 중..."' INT
rosbag record --buffsize=2048 -O "$BAG" "${TOPICS[@]}"
trap - INT

sleep 1
if [ -f "${BAG}.bag.active" ]; then
  echo "[!] .active 발견 - reindex 시도"
  rosbag reindex "${BAG}.bag.active" && mv "${BAG}.bag.active" "${BAG}.bag"
fi
echo
echo "=============================================="
if [ -f "${BAG}.bag" ]; then
  rosbag info "${BAG}.bag"
  echo "----------------------------------------------"
  for t in "${TOPICS[@]}"; do
    rosbag info "${BAG}.bag" 2>/dev/null | grep -q " ${t} " || echo "[!] 비어 있음: $t"
  done
  # root 로 녹화했을 때 lab 계정이 못 읽는 문제 방지
  chmod -R a+rX "$OUTDIR" 2>/dev/null
  echo "[i] 저장 완료: ${BAG}.bag"
else
  echo "[!] bag 파일이 생성되지 않았습니다: ${BAG}.bag"
fi
