# 개요

CDSL@UoS의 franka emika panda 로봇 구동을 위한 코드입니다.

Original source: "libfranka" repo

Language: C++

Implemented theories: 

  - ('26 Spring) Yesol: QPC
  - ('26 Spring) Haechan: Sigmoid CLBF



### Sigmoid CLBF

**완료**

A-1. 먼저, refactoring한 코드가 작동되는지 확인하기..

A-2. RPY 기반 orientation 좌표가 출력되는지 확인하기..

C-1. Fire monitor control 시 사용했던 Reference Generator를 library화하여 코드에 반영

C-2. 생성한 reference generator로 CLBF 코드의 reference를 생성 

**WIP**

B-1. EE body RPY를 base frame RPY로 바꾸기

B-2. 각속도 기반의 Jacobian을 base frame RPY 기반 Jacobian으로 바꾸기.. (교재의 변환 참조)

**논의사항**

