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

**WIP**

B-1. 각속도 기반의 Jacobian을 global RPY로 바꾸기.. (교재의 변환 참조)

**논의사항**

C-1. FL 제어 자체가 position에서부터 너무 구리게 (...) 된다. gain을 높이면 더 잘 따라가는게 아니고, 더 크게 요동친다..

해결책 1) 현재는 레퍼런스가 충분히 느리다는 가정 하에 가속도 값을 0으로 넣고 있는데(libfranka에서 사용되던 방식), LPF 근사를 통한 가속도를 사용하기.

해결책 2) 모르겠는데..
