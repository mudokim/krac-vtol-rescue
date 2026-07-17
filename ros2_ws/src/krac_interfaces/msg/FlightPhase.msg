# Mission Phases based on KRAC Rulebook
uint8 READY=0
uint8 TAKEOFF_VERTICAL=1        # ① 버티포트 수직 이륙
uint8 HOVERING_WP1=2            # ② WP1 이동 및 호버링
uint8 TRANSITION_TO_FW=3        # ③ 전진 비행 및 고정익 천이 (WP2 이동 중)
uint8 CRUISE_WP2_TO_WP3=4       # ④ WP2 -> WP3 (고정익)
uint8 CRUISE_WP3_TO_WP4=5       # ⑤ WP3 -> WP4 (고정익)
uint8 CRUISE_WP4_TO_WP2=6       # ⑥ WP4 -> WP2 (고정익)
uint8 TRANSITION_TO_MC=7        # ⑦ 조난자 위치로 감속 및 회전익 천이
uint8 RESCUE_OPERATION=8        # ⑧ 조난자 구조 (정밀 접근 및 구조)
uint8 DROP_RESCUEE=9            # ⑨ 하기 위치 이동 및 하기
uint8 LANDING_VERTICAL=10       # ⑩ 버티포트 수직 착륙
uint8 MISSION_COMPLETE=11

uint8 current_phase
