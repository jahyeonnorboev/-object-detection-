#!/bin/bash
SESSION="robot"


export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger


GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

log()  { echo -e "${GREEN}[✓]${NC} $1"; }
warn() { echo -e "${YELLOW}[~]${NC} $1"; }
err()  { echo -e "${RED}[✗]${NC} $1"; }
info() { echo -e "${CYAN}[i]${NC} $1"; }

echo ""
echo -e "${CYAN}╔══════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║   TurtleBot3 Control System Launcher     ║${NC}"
echo -e "${CYAN}╚══════════════════════════════════════════╝${NC}"
echo ""


warn "기존 tmux 세션 정리 중..."
tmux kill-session -t $SESSION 2>/dev/null
sleep 1


log "tmux 세션 '$SESSION' 생성..."
tmux new-session -d -s $SESSION -x 220 -y 50



# 창 0: bringup   창 1: lidar     창 2: camera
# 창 3: follower  창 4: voice     창 5: rosbridge
# 창 6: webapp



tmux rename-window -t $SESSION:0 "bringup"
tmux send-keys -t $SESSION:0 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [1/6] TurtleBot3 Bringup'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_bringup robot.launch.py
" ENTER

log "[1/6] Bringup 시작"
sleep 5


tmux new-window -t $SESSION:1 -n "lidar"
tmux send-keys -t $SESSION:1 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [2/6] LiDAR LD14'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch ldlidar_ros2 ld14.launch.py
" ENTER

log "[2/6] LiDAR 시작"
sleep 4

tmux new-window -t $SESSION:2 -n "camera"
tmux send-keys -t $SESSION:2 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [3/6] Camera OV5647'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
ros2 run camera_ros camera_node --ros-args \
  -p format:=BGR888 \
  -p width:=320 \
  -p height:=270 \
  -p camera:=0
" ENTER

log "[3/6] 카메라 시작"
sleep 4

# 창 3: YOLO 추적 노드
tmux new-window -t $SESSION:3 -n "follower"
tmux send-keys -t $SESSION:3 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [4/6] YOLO Human Follower'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=30
python3 ~/ros2_ws/src/human_following/human_following/last_following.py
" ENTER

log "[4/6] YOLO Follower 시작"
sleep 4

tmux new-window -t $SESSION:4 -n "voice"
tmux send-keys -t $SESSION:4 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [5/6] Voice Control (Vosk)'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
cd ~
python3 voice_robot_control.py
" ENTER

log "[5/6] 음성 인식 시작"
sleep 3

tmux new-window -t $SESSION:5 -n "rosbridge"
tmux send-keys -t $SESSION:5 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [6/6] rosbridge WebSocket'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
" ENTER

log "[6/6] rosbridge 시작"
sleep 3

tmux new-window -t $SESSION:6 -n "webapp"
tmux send-keys -t $SESSION:6 "
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
echo '  [webapp] HTTP Server'
echo '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
cd ~/webapp
ROBOT_IP=\$(hostname -I | awk '{print \$1}')
echo \"\"
echo \"  🌐 웹앱 주소: http://\$ROBOT_IP:8080/turtlebot3_control_v4.html\"
echo \"\"
python3 -m http.server 8080
" ENTER

log "[webapp] 웹앱 서버 시작"


sleep 2
echo ""
echo -e "${GREEN}╔══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║           전체 시스템 시작 완료!                 ║${NC}"
echo -e "${GREEN}╚══════════════════════════════════════════════════╝${NC}"
echo ""
ROBOT_IP=$(hostname -I | awk '{print $1}')
echo -e "${CYAN}로봇 IP    :${NC} $ROBOT_IP"
echo -e "${CYAN}웹앱 주소  :${NC} http://$ROBOT_IP:8080/turtlebot3_control_v4.html"
echo -e "${CYAN}rosbridge  :${NC} ws://$ROBOT_IP:9090"
echo ""

echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}  tmux 창 전환 단축키${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  tmux attach -t robot     ${CYAN}← 세션 접속${NC}"
echo -e "  Ctrl+B → 0               ${CYAN}← bringup 창${NC}"
echo -e "  Ctrl+B → 1               ${CYAN}← lidar 창${NC}"
echo -e "  Ctrl+B → 2               ${CYAN}← camera 창${NC}"
echo -e "  Ctrl+B → 3               ${CYAN}← follower 창${NC}"
echo -e "  Ctrl+B → 4               ${CYAN}← voice 창${NC}"
echo -e "  Ctrl+B → 5               ${CYAN}← rosbridge 창${NC}"
echo -e "  Ctrl+B → 6               ${CYAN}← webapp 창${NC}"
echo -e "  Ctrl+B → d               ${CYAN}← 세션 detach (백그라운드)${NC}"
echo -e "  Ctrl+B → &               ${CYAN}← 현재 창 종료${NC}"
echo ""
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${YELLOW}  개별 재시작 명령어${NC}"
echo -e "${YELLOW}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "  ${GREEN}bringup   :${NC} source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=30 && export TURTLEBOT3_MODEL=burger && ros2 launch turtlebot3_bringup robot.launch.py"
echo -e "  ${GREEN}lidar     :${NC} source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=30 && ros2 launch ldlidar_ros2 ld14.launch.py"
echo -e "  ${GREEN}camera    :${NC} source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=30 && ros2 run camera_ros camera_node --ros-args -p format:=BGR888 -p width:=320 -p height:=270 -p camera:=0"
echo -e "  ${GREEN}follower  :${NC} source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=30 && python3 ~/ros2_ws/src/human_following/human_following/last_following.py"
echo -e "  ${GREEN}voice     :${NC} source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && cd ~ && python3 voice_robot_control.py"
echo -e "  ${GREEN}rosbridge :${NC} source /opt/ros/humble/setup.bash && export ROS_DOMAIN_ID=30 && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
echo ""

# tmux 세션으로 자동 진입
info "tmux 세션으로 진입합니다..."
sleep 1
tmux attach -t $SESSION
