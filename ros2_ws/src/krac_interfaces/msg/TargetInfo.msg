std_msgs/Header header

bool detected           # 인식 성공 여부
string target_type      # "survivor" or "marker_v"
float32 image_x         # 영상 내 X 좌표 (-1.0 ~ 1.0)
float32 image_y         # 영상 내 Y 좌표 (-1.0 ~ 1.0)
float32 distance        # 타겟까지의 거리 (m)
float32[] location_xyz  # 3차원 좌표 (상대 좌표)
