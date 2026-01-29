./grasp_object 172.16.0.2 1 0.015

./generate_cartesian_pose_motion 172.16.0.2

./grasp_object 172.16.0.2 1 0.015




./QPC_controller 172.16.0.2 data.txt

./QPC_controller 172.16.0.2 /home/cdsl/libfranka/a_plot_data/QP6-1.txt

./QPC_controller 172.16.0.2 /home/cdsl/libfranka/a2_data/sim1/pd6.txt


./grasp_object 172.16.0.2 1 0.015

./grasp_object 172.16.0.2 1 0.035
예: 5cm 짜리 물체라면 -> 0.05

예: 3cm 짜리 물체라면 -> 0.03

1. V 구하는거 + nominal model의 mass matrix 구하기
2. 데이터 읽히는지?
3. qp 함수 작성하기
4. 전체 통합하기


(saturation: -0.01~0.01)
2번 시뮬 : // position_target << 0.3, 0.005, 0.5; : 5차 다항식
3번 시뮬 : position_target << 0.3, 0.005, 0.65; : 5차 다항식
4번 시뮬 : 3번에서 외란(무게 추가하기)

(saturation: -2~2)
4-1번 시뮬 : 3번에서 외란(무게 추가(더 무거운거)하기)

5번 시뮬 :3번에서 gain, 목표지점 바꾸기
FL: 

------------------------------------------
// git ignore 반영이 안된 경우 해결방법
git rm -r --cached .  # 현재 담긴 파일 목록 초기화 (파일은 안 지워짐)
git add .             # .gitignore 규칙대로 다시 담기
git commit -m "Apply gitignore"
git push -f origin main


------------------------------------------
./QPC_controller 172.16.0.2 /home/cdsl/libfranka/3dof_sim1/pd1.txt
