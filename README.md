# 개요

CDSL@UoS의 franka emika panda 로봇 구동을 위한 코드입니다.

Original source: "libfranka" repo

Language: C++

Implemented theories: 

  - ('26 Spring) Yesol: QPC
  - ('26 Spring) Haechan: Sigmoid CLBF


## Ongoing Process_HP

(Mar 5) WIP
gain tuning을 했더니 적당히 잘 돌아감.. Euler angle (X-Y-Z) 적당히 잘 돌아감.
- 다만 geometric <> analytic 변환 테이블은 정확하게 찾아서 쓰진 않았음... 

(Mar 5) What's next?
외란의 형태? 어떻게 "unsafe situation"을 만들 것인가..

